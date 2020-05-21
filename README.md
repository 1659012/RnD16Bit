# RnD16Bit - ArduPilot
##Requirements
The source codes are based on python3.7 

Install initial dependencies 
```
$ python-dev python-opencv python-wxgtk3.0 python-matplotlib python-pygame python-lxml python-yaml vim git
```
Or using the pre-made [executable file](https://github.com/1659012/RnD16Bit/tree/ArduPilot/Source%20code/ArduPilot/Notes/Environment%20setup
)

#####(Recommended) Install LinuxOS Virtual Machine (Follow these [NOTES](https://github.com/1659012/RnD16Bit/tree/ArduPilot/Source%20code/ArduPilot/Notes/Virtual%20Machine%20setup))
Clone and submodule the ArduPilot git repository using
```
$ git clone https://github.com/ardupilot/ardupilot
$ cd ardupilot
$ git submodule update --init --recursive
```
Navigate into Tools/autotest and copy the PATH

Modify your <bashrc> (system path) file
```
$ sudo vim ~/.bashrc
$ export PATH="<your path here>":$PATH
$ source ~/.bashrc
```