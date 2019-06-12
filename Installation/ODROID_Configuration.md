# ODROID Setup
Simple, but thorough, instructions to first configure the ODROID-XU4.

### Hardware
- Required
  - [ODROID-XU4](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Technology/ODROID_XU4.md)
  - Power Adapter
    - 5V/4A
    - Barrel jack: 2.1mm inner and 5.5mm outer
  - HDMI Cord
  - HDMI Compatible Monitor
  - USB Compatible Keyboard
  - microSD Card
    - ~16GB sufficient
  - [WiFi Adapter](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Technology/Edimax_N150.md)

- Optional
  - USB Compatible Mouse (highly recommended/necessary)
  - eMMC: Faster than uSD
    - ~16GB sufficient
  - Ethernet Cord/Connection: The ODROID has a very fast ethernet port

### Download the OS
Download the XU4 Ubuntu MATE 16.04.2 image from the [ODROID wiki](https://wiki.odroid.com/odroid-xu4/os_images/linux/ubuntu/ubuntu).
The **20170731** version downloaded from the main server was used. Direct Link: [ubuntu-16.04.2-mate-odroid-xu4](https://odroid.in/ubuntu_16.04lts/ubuntu-16.04.2-mate-odroid-xu4-20170510.img.xz)

Assure the downloaded file has the same [.xz md5 checksum](https://odroid.in/ubuntu_16.04lts/ubuntu-16.04.2-mate-odroid-xu4-20170510.img.xz.md5sum) as given.

Unzip the .xz archive and assure the unpacked file has the same [unpacked md5 checksum](https://odroid.in/ubuntu_16.04lts/ubuntu-16.04.2-mate-odroid-xu4-20170510.img.md5sum). Flash and format the image to the storage device (uSD or eMMc).

### First OS Boot
As described in the "Getting Started with ODROID-XU4" that comes with the physical package, perform these steps in this order. Insert the storage device into the ODROID; set up keyboard, monitor, and internet; and plug in the device. It will boot up then reboot. Configure the internet; date and time; and any personal preferences. *Date and time is required for CA certification for some internet connections (e.g. university wifi).* Additionally, check the box (yes/accept/allow) for the multiverse repositories in the "Software and Updates" application.

### Updates
Update repositories and upgrade packages. The upgrade may take *hours* (~10hrs for our setup) so assure you have time and a secure internet connection.

```
sudo apt update
sudo apt upgrade
```

### Continue...
See the [Python_and_Pip](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/Python_and_Pip.md) instructions for the next installation steps. 
