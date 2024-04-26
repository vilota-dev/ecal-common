#!/usr/bin/env python3

import sys
import numpy as np
import subprocess
import signal
import csv
import os
import cv2  
from threading import Timer
from mcap.reader import make_reader

import capnp
capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage

# Topics list
topics = ["S1/cama", "S1/camd", "S1/stereo2_r", "S1/stereo1_l"]

# Base directory for saving files
base_dir = "postProcessedLog"
os.makedirs(base_dir, exist_ok=True)

# Global FFmpeg processes and timers
ffmpeg_processes = {}
no_frame_timers = {}

def sanitize_topic(topic):
    return topic.replace("/", "_")

def start_ffmpeg(topic, frame_width, frame_height):
    if topic not in ffmpeg_processes:
        sanitized_topic = sanitize_topic(topic)
        output_path = os.path.join(base_dir, f"{sanitized_topic}.mp4")
        command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pixel_format', 'bgr24' if topic in ["S1/stereo2_r", "S1/stereo1_l"] else 'gray',
            '-video_size', f"{frame_width}x{frame_height}",
            '-framerate', '16',
            '-i', '-',
            '-c:v', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'fast',
            '-crf', '23',
            output_path
        ]
        ffmpeg_processes[topic] = subprocess.Popen(command, stdin=subprocess.PIPE)

def stop_ffmpeg(topic):
    if topic in ffmpeg_processes:
        ffmpeg_processes[topic].stdin.close()
        ffmpeg_processes[topic].wait()
        del ffmpeg_processes[topic]
        print(f"FFmpeg process for {topic} closed and video file saved.")

def reset_timeout(topic):
    if topic in no_frame_timers:
        no_frame_timers[topic].cancel()
    no_frame_timers[topic] = Timer(3.0, lambda: stop_ffmpeg(topic))
    no_frame_timers[topic].start()

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    for topic in topics:
        stop_ffmpeg(topic)
    sys.exit(0)

def log_to_csv(topic, frame_id, timestamp):
    sanitized_topic = sanitize_topic(topic)
    csv_filename = os.path.join(base_dir, f"{sanitized_topic}.csv")
    with open(csv_filename, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([frame_id, timestamp])

def callback(topic, msg, ts):
    with eCALImage.Image.from_bytes(msg) as imageMsg:
        frame_id = imageMsg.header.seq

        if imageMsg.encoding == "mono8":
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8).reshape((imageMsg.height, imageMsg.width))
        elif imageMsg.encoding == "jpeg":
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = cv2.imdecode(mat, cv2.IMREAD_COLOR)

        if topic not in ffmpeg_processes:
            start_ffmpeg(topic, mat.shape[1], mat.shape[0])
        ffmpeg_processes[topic].stdin.write(mat.tobytes())
        reset_timeout(topic)
        log_to_csv(topic, frame_id, ts)

def main():
    if len(sys.argv) < 2:
        print("Usage: script.py <MCAP file>")
        sys.exit(1)

    mcap_file_path = sys.argv[1]

    with open(mcap_file_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(topics=topics):
            callback(channel.topic, message.data, message.publish_time)

    signal.signal(signal.SIGINT, signal_handler)

    for topic in topics:
        stop_ffmpeg(topic)

if __name__ == "__main__":
    main()
