#!/usr/bin/env python3

import sys
import time
import subprocess
from datetime import datetime, timedelta

import capnp

import ecal.core.core as ecal_core
from capnp_subscriber import CapnpSubscriber

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage

camera_status = {
    "S0/cama": {"status": "no messages received", "timestamp": None},
    "S0/camb": {"status": "no messages received", "timestamp": None},
    "S0/camc": {"status": "no messages received", "timestamp": None},
    "S0/camd": {"status": "no messages received", "timestamp": None},
    "S1/cama": {"status": "no messages received", "timestamp": None},
    "S1/camb": {"status": "no messages received", "timestamp": None},
    "S1/camc": {"status": "no messages received", "timestamp": None},
    "S1/camd": {"status": "no messages received", "timestamp": None}
}

def install_gpiod():
    try:
        print("Installing gpiod...")
        subprocess.run(['sudo', 'apt-get', 'update'], check=True)
        subprocess.run(['sudo', 'apt-get', 'install', '-y', 'gpiod'], check=True)
        print("gpiod installed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to install gpiod: {e.stderr.decode().strip()}")
        sys.exit(1)

def check_gpioget():
    try:
        subprocess.run(['gpioget', '--help'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("gpioget is already installed.")
    except FileNotFoundError:
        install_gpiod()

def check_vpu_power():
    try:
        result = subprocess.run(['sudo', 'gpioget', '0', '1'], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode == 0:
            return "VPU power check: OK"
        else:
            return "VPU power check: FAILED"
    except subprocess.CalledProcessError as e:
        return f"Failed to check VPU power: {e.stderr.decode().strip()}"

def callback(type, topic_name, msg, ts):
    with eCALImage.Image.from_bytes(msg) as imageMsg:
        camera_status[topic_name]["status"] = "ok"
        camera_status[topic_name]["timestamp"] = datetime.now()
        # print(f"Received message from {topic_name}")

def main():
    check_gpioget()

    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))

    ecal_core.initialize(sys.argv, "camera_status_checker")
    ecal_core.set_process_state(1, 1, "I feel good")

    topics = [
        "S0/cama", "S0/camb", "S0/camc", "S0/camd",
        "S1/cama", "S1/camb", "S1/camc", "S1/camd"
    ]

    subs = []
    for topic in topics:
        print(f"Subscribing to topic {topic}")
        sub = CapnpSubscriber("Image", topic)
        sub.set_callback(callback)
        subs.append(sub)

    time.sleep(2)

    while ecal_core.ok():
        vpu_status = check_vpu_power()
        print(vpu_status)

        now = datetime.now()
        for topic in topics:
            if camera_status[topic]["timestamp"] and (now - camera_status[topic]["timestamp"]) > timedelta(seconds=1):
                camera_status[topic]["status"] = "no messages received"
            print(f"{topic} status: {camera_status[topic]['status']}")

        time.sleep(1)
        print("-----")

    ecal_core.finalize()

if __name__ == "__main__":
    main()
