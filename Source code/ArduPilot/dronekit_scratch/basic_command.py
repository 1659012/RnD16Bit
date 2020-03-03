##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse


#########FUNCTIONS#################

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string,wait_ready=True)

	return vehicle

def arm_and_takeoff(targetHHeight):
    while vehicle.is_armable != True:
        print("Wait to be armable")
        time.sleep(5)
    print("Vehicle is armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Wait for drone to be set as GUIDED flight mode")
        time.sleep(5)
    print("Vehicle is in GUIDE MODE")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Wait to be armed")
        time.sleep(5)
    print("Virtual props are spinning")

##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()
