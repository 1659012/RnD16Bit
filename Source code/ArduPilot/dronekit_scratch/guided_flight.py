#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.
Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(3)

#Set velocity

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

    """
Fly the vehicle in a SQUARE path using velocity vectors (the underlying code calls the 
SET_POSITION_TARGET_LOCAL_NED command with the velocity parameters enabled).
The thread sleeps for a time (DURATION) which defines the distance that will be travelled.
The code also sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method in each segment
so that the front of the vehicle points in the direction of travel
"""


#Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 2
SOUTH = -2

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 2
WEST = -2

# Note for vz: 
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5


# Square path using velocity
print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

print("Yaw 180 absolute (South)")
condition_yaw(180)

print("Velocity South & up")
send_ned_velocity(SOUTH,0,UP,DURATION)
send_ned_velocity(0,0,0,1)


print("Yaw 270 absolute (West)")
condition_yaw(270)

print("Velocity West & down")
send_ned_velocity(0,WEST,DOWN,DURATION)
send_ned_velocity(0,0,0,1)


print("Yaw 0 absolute (North)")
condition_yaw(0)

print("Velocity North")
send_ned_velocity(NORTH,0,0,DURATION)
send_ned_velocity(0,0,0,1)


print("Yaw 90 absolute (East)")
condition_yaw(90)

print("Velocity East")
send_ned_velocity(0,EAST,0,DURATION)
send_ned_velocity(0,0,0,1)

#

print("Set default/target airspeed to 3")
vehicle.airspeed = 1

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(10.760669, 106.686042, 1)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(30)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-35.363244, 149.168801, 1)
vehicle.simple_goto(point2, groundspeed=1)

# sleep so we can see the change in map
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()