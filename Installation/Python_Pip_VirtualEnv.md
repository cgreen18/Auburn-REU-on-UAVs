# Python, pip, and virtualenv install instructions
Continuation of the setup instructions from [ODROID_Configuration](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/ODROID_Configuration.md). These instructions will assume you are starting fresh.

As always, update the repositories

### Python3
Ubuntu MATE 16.04 (shorthand UM16) comes installed with Python3.5 but the Pico Flexx requires Python3.6 (and an incredible mess of dependencies/libraries). Install Python3 through J Fernyhough's PPA

```
sudo apt sudo add-apt-repository ppa:jonathonf/python-3.6
```
He says that only backporting Python3.6 (i.e. deleting Python3.5) may not work. Press <kbd>ENTER</kbd> to accept. Update the repositories and install python3.6

```
sudo apt-get update
sudo apt-get install python3.6
```

### Virtual Environment
It is recommended to create a virtual environment because the libraries and dependencies are going to get a little hairy.
Use the pip3 (8.1.1) that comes with UM16 to install virtualenv; when creating the Python3.6 virtual environment, it will automatically install the newest (19.1.1 at this time) anyway. Install virtualenv (16.6.0 at this time) through pip.

```
pip install virtualenv
```

Test that virtualenv setup the executable correctly by typing

```
virtualenv
```

If the manual does not appear then you must add the executable to your .bashrc (not the only method but it's simple) and reload the profile.

```
echo -e '\nexport PATH="/home/$USER/.local/bin:$PATH"' >> ~/.bashrc
source .bashrc
```

### Creating a Python3.6 Virtual Environment
First, find your Python3.6 executable. It will almost certainly be /etc/bin/python3.6 but if not, then a quick "whereis python3.6" will give a good idea. Move into the directory you want this environment to reside (it is like a directory.) Create the environment --called "uav3.6" in this guide-- and initialize it.

```
virtualenv --python=/usr/bin/python3.6 uav3.6
source uav3.6/bin/activate
```
where /usr/bin/python3.6 is the path to your Python3.6 executable and uav3.6 is the name of the virtual environment.

To exit the virtual environment, simply type
```
deactivate
```

### Further Installation
The Pico Flexx requires various libraries and dependencies that will be a mess. First, probably download the Royale SDK kit as in the guide [Pico_Flexx_Linux](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/Pico_Flexx_Linux.md). Then, see the guide [Lib_Python](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/Libpython.md) for how to actually make everything work.
