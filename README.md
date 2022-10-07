## eCAL Common Headers and Definitions

This includes schema definitions for serialisation.

### ByteSubscriber Patch

To apply patch, after installing `sudo apt install python3-ecal5`
```bash
sudo patch -d /usr/lib/python3/dist-packages/ecal/core < bytesubscriber.patch
```