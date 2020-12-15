from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

vehicle = mavutil.mavlink_connection('udpin:localhost:14550')

armed = vehicle.motors_armed()
print(armed)

