# Pico Flexx - libpython3.4m.so.1.0
*This took me 20+ hours to solve. I hope this guide works and saves that time.*
This project will use the Pico Flexx in Python for the best integration with the rest of the technology. If you navigate to the libroyale python directory "~/blah/blah/libroyale-3.23.0.86-LINUX-arm-32Bit/python" there are libraries and sample programs. Activate the Python3.6 virtual environment try to run one of the samples. It will return the error "ImportError: libpython3.4m.so.1.0: cannot open shared object file: No such file or directory"  The README in the python directory notes this problem but just says to "Install that library." It is not that simple --this problem plagues the internet-- and is *particularly* hard on Ubuntu 16 (PMD recommended) which is too new for Python3.4  If you were theoretically running Ubuntu 14, it would be easier but then you would have other problems. This is an entire guide to fix that issue.

### Option 1
If you are not running Ubuntu MATE 16.04 (or a variant of Ubuntu 16) then you may be able to simply apt install the library by
```
sudo apt install libpython3.4-dev
```

### Option 2(a)
libpython3.4 comes with Python3.4 but unfortunately, Python3.4 was taken out of the repositories in Ubuntu 16 so it was necessary to install from source. Sairam Krishna's guide on [How to Install Python 3.4.4 on Ubuntu](https://www.tutorialspoint.com/articles/how-to-install-python-3-4-4-on-ubuntu) explains it very well. **HOWEVER**, change how you configure the install to aid in library sharing problems later on.
In the section "**Compiling Python Source**," instead of "sudo ./configure" , run with share enabled as explained in this [Stackoverflow Query](https://stackoverflow.com/questions/7880454/python-executable-not-finding-libpython-shared-library/19402112).
```
./configure --enable-shared --prefix=/usr/local LDFLAGS="-Wl,-rpath=/usr/local/lib"
```

### Option 2(b)
Many answers online say to export the directory that contains the libpython3.4m.so.1.0 to your LD_LIBRARY_PATH.
```
export LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/usr/src/Python-3.4.4
```
and maybe even put the above line in your .bashrc (manually or the following command.)
```
echo "export LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/usr/src/Python-3.4.4" >> ~/.bashrc
source ~/.bashrc
```

### Option 2(c)
Even this may not work. If so, what eventually worked for me was to manually add the path to the libpython3.4m.1.0 to configuration files. The path to libpython3.4m.1.0 should be where you manually installed Python3.4 in /usr/src/Python-3.4.4

```
cd /etc/ld.so.conf.d
sudo nano libpython.conf
```
and simply type the path to the Python3.4 install, namely, "/usr/src/Python-3.4.4"
Reload the configuration in verbose mode and pipe to search "libpython3.4m.so.1.0" to assure it worked.
```
ldconfig -v | grep lilbpython3.4m.so.1.0
```
and assure you see the exact line "libpython3.4m.so.1.0 -> libpython3.4m.so.1.0"  Otherwise, ldconfig verbose again (without grep) and scroll up for the error message.

### Test the Library
Navigate to the libroyale python directory "~/blah/blah/libroyale-3.23.0.86-LINUX-arm-32Bit/python" and enter your Python3.6 virtual environment. Attempt to run one of the samples.
```
python sample_retrieve_data.py
```
If you do not immediately get the libpython error then you have successfully connected that library! You will still run into a simple matplotlib import error. See the [Further_Installs](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/Further_Installs.md) guide to import them (not a simple pip install.)
