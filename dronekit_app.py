from pymavlink import mavutil
import time
from MAVProxy.modules.lib import mp_module


vehicle = mavutil.mavlink_connection('udpin:localhost:14550')

altitude = 3

vehicle.mav.command_long_send(
	vehicle.target_system,  # target_system
	mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL, # target_component
	mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
	0,0,0,0,0,0,0,altitude)

