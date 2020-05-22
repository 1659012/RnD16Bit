# DJI Tello drone web controller

This is a python package which controlls DJI toy drone 'Tello' via a web controller. 


## How to install
Make sure python3.7 or above is available and functioning within your system

Install the requirement packages using pip command 
```
        $ pip install Flask
        $ pip install numpy
        $ pip install av
        $ pip install opencv-python
        $ pip install image
        $ pip install tellopy
        $ pip install imutils
        $ pip install pygame
```
Or install from the requirements.txt
```
        $ pip install -r requirements.txt
```

Install PyCharm from https://www.jetbrains.com/pycharm/ 

(PyCharm is a python IDE to control the source paths and executing python codes)

## Usage
Remember to create a run config for your file with the path directed to the [main.py](https://github.com/1659012/RnD16Bit/blob/Tello/Source_code/Tello/Y/droneapp/main.py) file

Data bits is around 360kb/s

### Tello lights

- flashing blue - charging
- solid blue - charged
- flashing purple - booting up
- flashing yellow fast - wifi network set up, waiting for connection
- flashing yellow - User connected

## Occurring bugs
1) Traditional automating commands like forward(0.3) is still dependent on environment
2) Tello will sometimes completely ignore commands.
(I don't know is it because of the Tello itself or not)
3) Object detection is still under researching, the base codes are finished but have not tested out.