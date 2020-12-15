from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

vehicle = mavutil.mavlink_connection('udpin:localhost:14550')

altitude = 3

vehicle.mav.command_long_send(
	vehicle.settings.target_system,  # target_system
	mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
	mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
	0, # confirmation
	0, # param1
	0, # param2
	0, # param3
	0, # param4
	0, # param5
	0, # param6
	altitude) # param7

