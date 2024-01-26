#!/usr/bin/env python3

#import nicegui as ui
#https://github.com/zauberzeug/nicegui/blob/main/examples/opencv_webcam/main.py
import cv2
import depthai as dai
import time

def is_version_greater_or_equal(version, target_version):
    # Split the version strings into components
    version_components = list(map(int, version.split('.')))
    target_components = list(map(int, target_version.split('.')))

    # Compare each component
    for v, target in zip(version_components, target_components):
        if v < target:
            return False
        elif v > target:
            return True

    # If all components are equal or the version has additional components
    return True

with dai.Device() as device:
    print('Device Name:', device.getDeviceName())
    print('Product Name: ', device.getProductName())

    # check for bootloader
    bootloader_version = device.getBootloaderVersion().toStringSemver()
    minimum_bootloader_version = "0.0.26"
    print('Bootloader Version: ', bootloader_version);

    calib = device.readCalibration()
    print("Board Options string: ", calib.getEepromData().boardOptions)

    if is_version_greater_or_equal(bootloader_version, minimum_bootloader_version):
        print("Bootloader version check pass!")
    else:
        raise RuntimeError("bootloader version too low, please use Depthai Python device_manager.py to update");

    product_name = input("enter the new product name [VK180, VKL] : ")

    if product_name == "VK180":
        print("to setup the device as a VK180 device")
        product_name = "VK180-V2-FFC"
    elif product_name == "VKL":
        print("to setup the device as a VKL-M7 or VKL-M12 device")
        product_name = "VKL-V2-FFC"
    else:
        raise RuntimeError("invalid product name: ")


    
    calib.setBoardInfo(product_name, "", "", "", "", "", 0, 128)

    print("Flashing calibration data into the device")
    
    device.flashCalibration(calib)

    print("Success!")

    