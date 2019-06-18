# Python Scripts to Control Quadcopter
The scripts in this file will perform all the operations for the project. The files will be organized into folders for organization and readability.

**All Scripts**:
Place to put the scripts that import others (i.e. a principal script importing a flight script.)

**Localization**:
Holds scripts to query, process, and display navigational data

![Drone Reality](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Python/20190618_175119.gif)
![Drone Animated](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/Python/drone_animated_2.gif)

**Flight**:
Holds scripts to fly the drone. Only manual control is available at the moment.

**Principal**:
Holds scripts to call scripts in Localization and Flight. It will serve as the main user interface.

## Ps Drone
[PS Drone Website](http://www.playsheep.de/drone/index.html)

[PS Drone Documentation](https://sourceforge.net/projects/ps-drone/files/PS-Drone-Documentation.pdf/download)

The backbone of this project is the ps_drone library (as uploaded in this repository) created by J. Philipp de Graaff. Huge thanks to him for creating such a powerful, user-friendly library and not only that, but he maintains it to this date since it's creation in 2012-2015. In fact, the version of ps_drone.py in this repository is an *updated* version he sent me the other day since the code was having a unique problem. He also wrote extensive documentation on the Parrot AR Drone2.0 along with the documentation for his code (docs linked above). His software is available under the Artistic License 2.0 , where in this case, it is being used without modification. More information can be found at the link above.
