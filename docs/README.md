# Current Stable Release

## AMD64 (x86_64)

## ARM64 (aarch64)


# Installation (Tested on Ubuntu 20.04 only)

## VK Camera Driver

### 1. Install Package

eCAL ([official website](https://eclipse-ecal.github.io/ecal/getting_started/setup.html))
```bash
# for cpp
sudo add-apt-repository ppa:ecal/ecal-latest
# sudo apt-get update
sudo apt-get install ecal

sudo apt install libprotobuf-dev libprotoc-dev protobuf-compiler

# for python
sudo apt install python3 python3-pip
sudo apt install python3-ecal5
```

CapnProto ([official website](https://capnproto.org/install.html))
```bash
# for python
pip3 install pycapnp
```

In a terminal, install the .deb file using `apt`.
```bash
# remove previous package, if installed
sudo apt remove vk_camera_driver
# cd into the folder containing the .deb file
sudo apt install ./vk_camera_driver_0.1.0-20230409-af4ccc1_20.04_amd64_Release.deb
```

The same .deb file should automatically help you install the middleware eCAL. Applications like `eCAL Monitor` should show up in your Ubuntu Application panel.

### 2. First Time Setup

Run the following script, to allow USB device driver for Vilota products.
```bash
sudo /opt/vilota/bin/apply_udev_rules.sh
```

### 3. Test functionality

Use a good quality USB3 Type-C cable to connect Vilota product to host computer. After plugging in, in a terminal run
```bash
lsusb
```
In the output, you should be able to see a device being detected, with ID `03e7:f63c` or `03e7:2485`. If it does not appear:
- check cable quality, and check there are green and red LED lights lit up on Vilota product's PCB board;
- check udev rule has been applied properly. 

Now test functionality, product specific:

#### VKL Product
For `VKL-M12 (Color)` product at 15fps, with auto exposure turned on, run:
```bash
cd /opt/vilota/bin
./vk_camera_driver vkl-m12-color.json

# if you would like to output 270 degree depth, try
./vk_camera_driver vkl-m12-color_depth.json
```

For `VKL-M12 (Mono)` product at 20fps with auto exposure turned on, run:
```bash
cd /opt/vilota/bin
./vk_camera_driver vkl-m12-mono.json
```

#### VK180 Product

```bash
cd /opt/vilota/bin
./vk_camera_driver vk180.json
```

For successful run, you should be able to see continuous stream of status like:
- `average latency of imu xxx`
- `average latency of camera stream cama/b/c/d xxx`

You can also observe the status of message in `eCAL Monitor` or command line `ecal_mon_gui`.

## VK VIO

### 1. Install Package

In a terminal, install the .deb file using `apt`.
```bash
# remove previous package, if installed
sudo apt remove vk_vio
# cd into the folder containing the .deb file
sudo apt install ./vk_vio_0.1.0-20230408-08b0019_20.04_amd64_Release.deb
```

## Run Full Demo of VK Camera Driver and VK VIO

### Product: VKL-M12 (Color)

Terminal 1
```bash
cd /opt/vilota/bin
./vk_camera_driver vkl-m12-color.json

# if you would like to output 270 degree depth, try
./vk_camera_driver vkl-m12-color_depth.json

```

Terminal 2
```bash
cd /opt/vilota/bin
./vk_vio_ecal config_host_vkl.json
```

### Product: VKL-M12 (Mono)

Terminal 1
```bash
cd /opt/vilota/bin
./vk_camera_driver vkl-m12-mono.json
```

Terminal 2
```bash
cd /opt/vilota/bin
./vk_vio_ecal config_host_vkl.json
```