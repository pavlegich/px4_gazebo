from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math

vehicle = mavutil.mavlink_connection('udp:0.0.0.0:14550')

vehicle.arducopter_arm()

