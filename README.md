# DJI Tello drone web controller

This is a python package which controlls DJI toy drone 'Tello' via a web controller. 


##How to install
Make sure python3.7 or above is available and functioning within your system
Install the requirement packages using pip command 
```
	$ pip install Flask
	$ pip install numpy
	$ pip install av
	$ pip install opencv-python
	$ pip install image
	$ pip install pygame
```
Or install from the requirements.txt
```
        $ pip install -r requirements.txt
```

Install PyCharm from https://www.jetbrains.com/pycharm/ 
(PyCharm is a python IDE to control the source paths and executing python codes)

##Usage
Remember to create a run config for your file with the path directed to the [main.py](https://github.com/1659012/RnD16Bit/blob/Tello/Source_code/Tello/Y/droneapp/main.py) file


### Tello lights

- flashing blue - charging
- solid blue - charged
- flashing purple - booting up
- flashing yellow fast - wifi network set up, waiting for connection
- flashing yellow - User connected