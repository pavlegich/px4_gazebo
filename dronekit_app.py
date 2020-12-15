from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

vehicle = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

vehicle.arducopter_arm()

