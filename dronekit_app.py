from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print("Connecting")
connection_string = '127.0.0.1:14540'
vehicle = connect(connection_string, wait_ready=True)

# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)

# Arm vehicle
print("Motors arming...")
vehicle.armed = True


time.sleep(10)

# Disarm vehicle
print("Motors disarming...")
vehicle.armed = False