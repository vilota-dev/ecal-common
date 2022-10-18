## eCAL Common Headers and Definitions

This includes schema definitions for serialisation.

### [Obsolete] ByteSubscriber Patch

NOTE: This step is no longer needed, as the change is put into `byte_subscriber.py file`

To apply patch, after installing `sudo apt install python3-ecal5`
```bash
sudo patch -d /usr/lib/python3/dist-packages/ecal/core < bytesubscriber.patch
```