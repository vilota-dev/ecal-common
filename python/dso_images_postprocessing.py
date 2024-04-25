#!/usr/bin/env python3

import sys
import time
import numpy as np
import subprocess
import signal
import csv
import os
from threading import Timer

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

import capnp
capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage


# Global FFmpeg process
ffmpeg_process = None

# Timer for timeout
no_frame_timer = None

# Base directory for saving files
base_dir = "postProcessedLog"

# Ensure the directory exists
os.makedirs(base_dir, exist_ok=True)

def start_ffmpeg(topic, frame_width, frame_height):
    global ffmpeg_process
    output_path = os.path.join(base_dir, f"{topic}.mp4")
    command = [
        'ffmpeg',
        '-y',  # Overwrite output files
        '-f', 'rawvideo',  # Input format
        '-pixel_format', 'gray',  # Input pixel format for mono8 (grayscale)
        '-video_size', f"{frame_width}x{frame_height}",  # Input video size
        '-framerate', '16',  # Frame rate
        '-i', '-',  # Input comes from a pipe
        '-c:v', 'libx264',  # Encoder to use for H264
        '-pix_fmt', 'yuv420p',  # Output pixel format
        '-preset', 'fast',  # Encoding preset for balance of speed and quality
        '-crf', '23',  # Quality, lower means better
        output_path  # Output file name
    ]
    ffmpeg_process = subprocess.Popen(command, stdin=subprocess.PIPE)

def stop_ffmpeg():
    global ffmpeg_process
    if ffmpeg_process:
        ffmpeg_process.stdin.close()
        ffmpeg_process.wait()
        ffmpeg_process = None
        print("FFmpeg process closed and video file saved.")

def reset_timeout():
    global no_frame_timer
    if no_frame_timer:
        no_frame_timer.cancel()
    no_frame_timer = Timer(3.0, stop_ffmpeg)
    no_frame_timer.start()

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    stop_ffmpeg()
    sys.exit(0)

def log_to_csv(filename, frame_id, timestamp):
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([frame_id, timestamp])

def callback(topic, msg, ts):
    global ffmpeg_process
    with eCALImage.Image.from_bytes(msg) as imageMsg:
        frame_id = imageMsg.header.seq

        if imageMsg.encoding == "mono8":
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8).reshape((imageMsg.height, imageMsg.width))
            if ffmpeg_process is None:
                start_ffmpeg(topic, imageMsg.width, imageMsg.height)
            ffmpeg_process.stdin.write(mat.tobytes())
            reset_timeout()

        # Log frame ID and timestamp to CSV
        csv_filename = os.path.join(base_dir, f"{topic}.csv")
        log_to_csv(csv_filename, frame_id, ts)

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <topic>")
        sys.exit(1)
    
    topic = sys.argv[1]

    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))

    ecal_core.initialize(sys.argv, "video_subscriber")
    ecal_core.set_process_state(1, 1, "I feel good")

    sub = CapnpSubscriber("Image", topic)
    sub.set_callback(lambda type, tname, msg, ts: callback(topic, msg, ts))

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while ecal_core.ok():
            time.sleep(0.1)
    finally:
        stop_ffmpeg()
        ecal_core.finalize()

if __name__ == "__main__":
    main()
