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
