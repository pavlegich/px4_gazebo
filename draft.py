import asyncio
from pymavlink import mavutil
from mavsdk import System
import time, sys, argparse, math


vehicle = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
					
vehicle.wait_heartbeat()
																																				
print("---Arm")
vehicle.arducopter_arm()

time.sleep(10)



lat0 = vehicle.messages["GPS_RAW_INT"].lat*1e-7
lon0 = vehicle.messages["GPS_RAW_INT"].lon*1e-7
alt0 = vehicle.messages["GPS_RAW_INT"].alt*1e-3

print(lat0,lon0,alt0)

print("---Takeoff")



print("---Land")

print("---Disarm")

vehicle.arducopter_disarm()
