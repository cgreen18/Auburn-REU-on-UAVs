# Python & Pip install instructions

### Python3
Assure that you have Python 3 installed with

```
python3 --version
```

If that does not return
```
Python 3.x.x
```
where x are specific version numbers then type

```
sudo apt upgrade
sudo apt update
sudo apt install python3
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
