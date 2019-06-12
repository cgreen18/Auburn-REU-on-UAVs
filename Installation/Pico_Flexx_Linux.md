# Setting Up Pico Flexx in Linux
Configured in Ubuntu MATE 16.04.2 on an ARM 32bit processor with the Royale SDK 3.23.0.86

### Download and Unzip
As outlined in the guide that comes with the Pico Flexx, navigate to the [software page of the PMD website](https://pmdtec.com/picofamily/software/) and request the software to your email. Go through the link and enter the password given with your Pico Flexx. Simply download the full SDK (not android) labeled "libroyale.zip" and unpack into your desired directory.
Unzip the next file, in this case "20190430_royale_3.23.0.86.zip"

### Unpack Correct Folder
Unzip the zipped folder that corresponds to your processor. The ODROID has an arm-32bit.
In this case, that is "libroyale-3.23.0.86-LINUX-arm-32Bit.zip"

**Important** This must be unzipped into a path *without* spaces, i.e. /media/user/disk will work.  Later in the install, the driver installer will have trouble navigating to a directory with spaces, i.e. /media/user/disk/this folder will not work

### Add Driver Rules
Navigate to driver/udev within the newly unpacked libroyale-3.23.0.86-LINUX-arm-32Bit

```
cd driver/udev
```

(README will describe the next steps as well)

Assure you are in plugdev group and copy the file "10-royale-ubuntu.rules" to your system's settings for drivers by

```
sudo cp 10-royale-ubuntu.rules /etc/udev/rules.d
```

If you have the camera plugged in, unplug it and reinsert.

### Royale SDK Viewer
**Does not apply to the arm 32 bit package**
For a x86-64bit processor, the method below is the simplest way to assure the camera works. It will run the Pico Flexx in a preset environment.

Return to the "libroyale-3.23.0.86-LINUX-x86-64Bit" directory. If currently in /driver/udev folder simply "cd ../.." out.

Navigate to bin and execute royaleviewer.sh (sudo may not be necesary) to set the right paths,

```
cd bin
sudo ./royaleviewer.sh
```

Executing ./royaleviewer.sh again will run the application. Simply turn on the camera using the "Start" button in the bottom right and assure the camera works!
