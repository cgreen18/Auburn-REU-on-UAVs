# Create the Virtual Environment
### Instructions to create the virtual environment and download necessary dependencies
This tutorial will require virtualenv
See [Python and pip install instructions](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Installation/Python_and_Pip.md) to download/install virtualenv

### Create virtual environment "UAV"
First navigate to the desired directory and type

```
virtualenv uav
source uav/bin/activate
```

The bash shell should now have (uav) before the user identification. This will look like
```
(uav) user@computer:~/dir1/dir2/dir3$
```
where ~/dir1/dir2/dir3 is the file path you chose to create the environment.

To deactivate the environment (resume normal operation), type

```
deactivate
```

*to delete this environment, deactivate the virtualenv and simply remove the created directory through a "sudo rm -rf uav"

### Installing required dependencies/modules
