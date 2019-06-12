# Python & Pip install instructions
Continuation of the setup instructions from [ODROID_Configuration](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/ODROID_Configuration.md). These instructions will assume you are starting fresh.

As always, update the repositories

### Python3
Ubuntu MATE 16.04 comes installed with Python3.5 but the Pico Flexx requires Python3.6 (and an incredible mess of dependencies/libraries). Install Python3 through J Fernyhough's PPA

```
sudo apt sudo add-apt-repository ppa:jonathonf/python-3.6
```
He says that only backporting Python3.6 (i.e. deleting Python3.5) may not work. Press <kbd>ENTER</kbd>

```
sudo apt-get update
sudo apt-get install python3.6
```

### Pip3
Assure you have Pip3 installed with

```
pip3 -version
```

If that does not return
```
pip x.x.x from /usr/lib/python3/dist-packages (python 3.x)
```

where x are specific version numbers then type

```
sudo apt install python3-pip
```

### Virtual Environment
**This section is not necessary**

In order to contain certain dependencies and modules to this project alone (i.e. using Python 3 here but maintaining Python on the rest of your computer), virtual environments will be used.

Assure you have virtualenv installed with

```
pip3 
```

If it is not installed then type

```
pip3 install virtualenv
```
