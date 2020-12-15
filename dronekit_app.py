from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

vehicle = mavutil.mavlink_connection('udpin:localhost:14550')

vehicle.wait_heartbeat()
lat = vehicle.messages["GPS_RAW_INT"].lat*1e-7
lon = vehicle.messages["GPS_RAW_INT"].lon*1e-7
alt = vehicle.messages["GPS_RAW_INT"].alt*1e-3
sv = vehicle.messages['GPS_RAW_INT'].satellites_visible

print(lat,lon,alt)

