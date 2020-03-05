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



##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()

gps_type = vehicle.parameters['GPS_TYPE']
vehicle.parameters['GPS_TYPE']=3
gps_type = vehicle.parameters['GPS_TYPE']

if vehicle.parameters['GPS_TYPE']!=4:
	vehicle.parameters['GPS_TYPE']=4
	gps_type = vehicle.parameters['GPS_TYPE']

print("GPS_TYPE param value is %s"%str(gps_type))

# rtl_speed = vehicle.parameters['RTL_SPEED']
# vehicle.parameters['RTL_SPEED'] <= 100
# rtl_speed = vehicle.parameters['RTL_SPEED']

# if vehicle.parameters['RTL_SPEED'] > 100
# 	vehicle.parameters['RTL_SPEED'] < 50
# 	rtl_speed = vehicle.parameters['RTL_SPEED']

# print("RTL_SPEED param value is %s"%str(rtl_speed))


fs_gcs_enable = vehicle.parameters['FS_GCS_ENABLE']
vehicle.parameters['FS_GCS_ENABLE']=2 #### Enabled always RTL
fs_gcs_enable = vehicle.parameters['FS_GCS_ENABLE']

if vehicle.parameters['FS_GCS_ENABLE']==0: #### Disabled
	vehicle.parameters['FS_GCS_ENABLE']=2
	fs_gcs_enable = vehicle.parameters['FS_GCS_ENABLE']

print("FS_GCS param value is %s"%str(fs_gcs_enable))