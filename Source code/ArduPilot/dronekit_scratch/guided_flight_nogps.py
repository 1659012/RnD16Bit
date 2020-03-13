import sched, time, math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

    def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
		print("Can't connect to drone IP address")

	vehicle = connect(connection_string,wait_ready=True)

	return vehicle

	flightalt = 20 #This is how high you want it to fly to
	time.sleep(10)
	vehicle.channels.overrides = {}
	
	def arm_and_takeoffNoGPS(Target):

	#Arm the vehicle
        vehicle.mode    = VehicleMode("Guided_NoGPS")
	while not vehicle.is_armable: #Wait for the vehicle to be armable
		time.sleep(.5)

	vehicle.armed = True
	while not vehicle.armed:
		time.sleep(1)#Waiting for armed
	#Take Off
	thrust = 0.7
	while True:
		current_altitude = vehicle.location.global_relative_frame.alt
		if current_altitude >= Target*0.95:
			break
		elif current_altitude >= Target*0.6:
			thrust = 0.6
		set_attitude(thrust = thrust)
		time.sleep(0.2)

	def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):

	#Duration is seconds to do this for

	msg = vehicle.message_factory.set_attitude_target_encode(
		0,
		0,                                         #target system
		0,                                         #target component
		0b00000000,                                #type mask: bit 1 is LSB
		to_quaternion(roll_angle, pitch_angle),    #q
		0,                                         #body roll rate in radian
		0,                                         #body pitch rate in radian
		math.radians(yaw_rate),                    #body yaw rate in radian
		thrust)                                    #thrust

	vehicle.send_mavlink(msg)

	if duration != 0:
        	# Divide the duration into the frational and integer parts
        	modf = math.modf(duration)
        	
        	# Sleep for the fractional part
        	time.sleep(modf[0])
        	
        	# Send command to vehicle on 1 Hz cycle
        	for x in range(0,int(modf[1])):
        	    time.sleep(1)
        	    vehicle.send_mavlink(msg)

	def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
		"""
		Convert degrees to quaternions
		"""
		t0 = math.cos(math.radians(yaw * 0.5))
		t1 = math.sin(math.radians(yaw * 0.5))
		t2 = math.cos(math.radians(roll * 0.5))
		t3 = math.sin(math.radians(roll * 0.5))
		t4 = math.cos(math.radians(pitch * 0.5))
		t5 = math.sin(math.radians(pitch * 0.5))
    
		w = t0 * t2 * t4 + t1 * t3 * t5
		x = t0 * t3 * t4 - t1 * t2 * t5
		y = t0 * t2 * t5 + t1 * t3 * t4
		z = t1 * t2 * t4 - t0 * t3 * t5
    
		return [w, x, y, z]


	arm_and_takeoffNoGPS(flightalt)
