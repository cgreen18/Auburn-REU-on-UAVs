```
python --version
```
Python 2.7.12

```
python3 --version
```
Python 3.5.2

```
sudo apt install python3-pip
sudo apt install python-pip
sudo apt install git-core
```


```
pip install virtualenv

cd ~
echo -e '\nexport PATH="/home/$USER/.local/bin:$PATH"' >> .bashrc
source .bashrc

virtualenv uav
source uav/bin/activate

```

Within the uav virtual environment,
```
python --version
```
Python 3.5.2

Environment is based on Python3


```
pip install numpy
pip install matplotlib
sudo apt install libfreetype6-dev
pip install matplotlib
```

```
deactivate

```


```
virtualenv --python=/usr/bin/python2.7 ~/Desktop/REU/uav2.7
source ~/Desktop/REU/uav2.7/bin/activate
```

Downloaded python 3.4 by instructions given in tutorials point.
