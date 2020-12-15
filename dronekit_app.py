from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print("Connecting")
connection_string = '127.0.0.1:14540'
vehicle = connect(connection_string, wait_ready=True)

aTargetAltitude = 3

# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)

# Arm vehicle
print("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

print("Taking off!")

vehicle.simple_takeoff(aTargetAltitude)

 # Check that vehicle has reached takeoff altitude
while True:
	print(" Altitude: ", vehicle.location.global_relative_frame.alt)
	#Break and return from function just below target altitude.        
	if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
		print("Reached target altitude")
		break
	time.sleep(1)

print("Take off complete")

time.sleep(10)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Disarm vehicle
print("Motors disarming...")
vehicle.armed = False

# Close vehicle object
vehicle.close()