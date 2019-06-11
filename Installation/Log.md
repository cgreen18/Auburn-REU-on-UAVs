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
```


```
pip install virtualenv

cd ~
echo -e '\nexport PATH="home/$USER/bin:$PATH"' >> .bashrc

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
deactivate

```
