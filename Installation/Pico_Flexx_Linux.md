# Setting Up Pico Flexx in Linux
Configured in Ubuntu 18.04LTS on an x86-64bit processor with the Royale SDK 3.23.0.86 as of June 4th, 2019.

### Download and Unzip
Simply download the full SDK (not android) labeled "libroyale.zip" and unpack into your desired directory.
Unzip the next file, in this case "20190430_royale_3.23.0.86.zip"

### Unpack Correct Folder
Unzip the zipped folder that corresponds to your processor.
In this case, that is "libroyale-3.23.0.86-LINUX-x86-64Bit"

**Important** This must be unzipped into a path *without* spaces, i.e. /media/user/disk  .  Later in the install, the driver installer will have trouble navigating to a directory with spaces, i.e. /media/user/disk/this folder

### Add Driver Rules
Navigate to driver/udev

```
cd driver/udev
```

(README will describe the next steps as well)

Assure you are in plugdev group and copy the file "10-royale-ubuntu.rules" to your system's settings for drivers by

```
sudo cp 10-royale-ubuntu.rules /etc/udev/rules.d
```

If you have the camera plugged in, unplug it and reinsert.

### Configure Royale SDK Viewer
The viewer is the simplest way to assure the camera works. It will run the Pico Flexx in a preset environment.

Return to the "libroyale-3.23.0.86-LINUX-x86-64Bit" directory. If currently in /driver/udev folder simply "cd ../.." out.

Navigate to bin and execute royaleviewer.sh (sudo may not be necesary) to set the right paths,

```
cd bin
sudo ./royaleviewer.sh
```

Executing ./royaleviewer.sh again will run the application. Simply turn on the camera using the "Start" button in the bottom right and assure the camera works!
