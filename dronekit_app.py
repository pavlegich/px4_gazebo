from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

aTargetAltitude = 3

# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)

# Arm vehicle
print("Arming motors")
vehicle.armed = True
while not vehicle.armed:
	print(" Waiting for arming...")
	time.sleep(1)

print("Taking off!")

vehicle.simple_takeoff(aTargetAltitude)

#  # Check that vehicle has reached takeoff altitude
# while True:
# 	print(" Altitude: ", vehicle.location.global_relative_frame.alt)
# 	#Break and return from function just below target altitude.        
# 	if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
# 		print("Reached target altitude")
# 		break
# 	time.sleep(1)

# print("Take off complete")

# time.sleep(10)

# print("Now let's land")
# vehicle.mode = VehicleMode("LAND")

# # Disarm vehicle
# print("Motors disarming...")
# vehicle.armed = False

# Close vehicle object
# vehicle.close()