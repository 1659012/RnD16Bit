#####DEPENDENCY#####
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time 
import socket
import exceptions
import math
import argparse

#####FUNCTION#####
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

#####MAIN EXECUTABLE#####
vehicle = connectMyCopter()

#Version and atrributes
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)

#Does the firmware support the companion pc to set the attitude
print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

#Read actual position
print('Position: %s'%vehicle.location.global_relative_frame)

#Read the actual attitude roll, pitch, yaw
print('Attitude: %s'%vehicle.attitude) 

#Read the actual velocity (m/s)
print('Velocity: %s'%vehicle.velocity) #North, East, Down

#When receive last heartbeat
print('Last Heartbeat: %s'%vehicle.last_heartbeat)

#Meet condition to arm
print('Is vehicle armable: %s'%vehicle.is_armable)

#Total Groundspeed
print('Groundspeed: %s'%vehicle.groundspeed) #Settable

#Current flight mode
print('Mode: %s'%vehicle.mode.name) #Settable

#Is vehicle armed yet
print('Armed: %s'%vehicle.armed) #Settable

#state estimation filter ok
print('EKF Ok: %s'%vehicle.ekf_ok)

vehicle.close()