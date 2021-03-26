# make px4_sitl gazebo_iris_opt_flow
# make px4_sitl gazebo

import asyncio
from pymavlink import mavutil
import time, sys, argparse, math

def get_location_offset_meters(original_location, dNorth, dEast, alt):
   
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location[0][0] + (dLat * 180/math.pi)
    newlon = original_location[0][1] + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)

# Запуск моторов
def arducopter_arm(vehicle):
	if vehicle.motors_armed():
		f = 1
		while f:
			print("Motors still armed!")
			vehicle.wait_heartbeat()
			f = vehicle.motors_armed()
			time.sleep(1)
	print("---Arming")
	vehicle.arducopter_arm()
	vehicle.motors_armed_wait()
	
# Посадка
def arducopter_land(vehicle):
	print("---Landing")
	vehicle.mav.command_long_send(
		vehicle.target_system,
		vehicle.target_component,
		mavutil.mavlink.MAV_CMD_NAV_LAND,
		0,0,0,0,0,0,0,0)

# Взлет
def arducopter_takeoff(vehicle, coordinates, altitude):
	print("---Takeoff")
	vehicle.mav.command_long_send(
		vehicle.target_system,
		vehicle.target_component,
		mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
		0,0,0,0,0,coordinates["lat"],coordinates["lon"],
		coordinates["alt"]+altitude)

# Переместиться
def arducopter_move(vehicle, coordinates, altitude):
	print("---Move")
	vehicle.mav.command_long_send(
		vehicle.target_system,
		vehicle.target_component,
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
		0,0,0,0,0,coordinates["lat"]+0.00001,coordinates["lon"]+0.00001,
		coordinates["alt"]+altitude)

def arducopter_hold(vehicle, location):
	print("---Hold")
	vehicle.mav.command_long_send(
		vehicle.target_system,
		vehicle.target_component,
		mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
		0,0,0,0,0,location[0][0],location[0][1],location[0][2])

# Текущая локация
def current_location(vehicle):
	vehicle.wait_heartbeat()
	return [vehicle.messages["GPS_RAW_INT"].lat*1e-7, 
	vehicle.messages["GPS_RAW_INT"].lon*1e-7,
	vehicle.messages["GPS_RAW_INT"].alt*1e-3]

# Проверка состояния спутникового сигнала
def gps_check(vehicle, last, current):
	status = 1
	vehicle.wait_heartbeat()
	st = vehicle.messages["GPS_RAW_INT"].satellites_visible
	if (not st):
		status = -1
	elif (st < 7 or abs(last[0]-current[0][0])>0.000038 or abs(last[1]-current[0][1])>0.000078 \
		 or abs(last[2]-current[0][2])>5):
		status = 0
	return status, st

# Ожидание нужной высоты
def wait_for_height(vehicle, gps0, altitude):
	loc = [[]]
	loc.pop(0)
	loc.append(current_location(vehicle))
	while (loc[0][2] - gps0["alt"] < altitude - 0.01):
		last_gps = [loc[0][0], loc[0][1], loc[0][2]]
		loc.pop(0)
		loc.append(current_location(vehicle))
		print(round(loc[0][2] - gps0["alt"],2), "m")
		print(gps_check(vehicle, last_gps, loc))
		# print("satellites_visible:", gps_check(vehicle)[1]) #10
		# print("lat:", abs(loc[0][0]-last[0]), " lon:", \
		# abs(loc[0][1]-last[1]), " alt:", abs(loc[0][2]-last[2]))
		time.sleep(0.3)
	return loc

# Ожидание нужной высоты при приземлении
def wait_for_height_down(vehicle, gps0):
	loc = [[]]
	loc.pop(0)
	loc.append(current_location(vehicle))
	while (loc[0][2] > gps0["alt"] + 0.01):
		last_gps = [loc[0][0], loc[0][1], loc[0][2]]
		loc.pop(0)
		loc.append(current_location(vehicle))
		print(round(loc[0][2] - gps0["alt"],2), "m")
		print(gps_check(vehicle, last_gps, loc))
		# print("satellites_visible:", gps_check(vehicle)[1])
		# print("lat:", abs(loc[0][0]-last[0]), " lon:", \
		# abs(loc[0][1]-last[1]), " alt:", abs(loc[0][2]-last[2]))
		time.sleep(0.3)
	return loc



async def run():

	gps0 = {"lat" : 0, "lon" : 0, "alt" : 0} # Начальные координаты
	altitude = 5 # Высота взлета
	altitude_goal = 3

	# Соединение с БВС, получение начальных координат
	vehicle = mavutil.mavlink_connection('udpin:localhost:14540')
	vehicle.wait_heartbeat()
	gps0["lat"] = vehicle.messages["GPS_RAW_INT"].lat*1e-7
	gps0["lon"] = vehicle.messages["GPS_RAW_INT"].lon*1e-7
	gps0["alt"] = vehicle.messages["GPS_RAW_INT"].alt*1e-3



	# st = gps_check(vehicle, gps0, gps0)

	print("Latitude:", round(gps0["lat"],2))
	print("Longitude:", round(gps0["lon"],2))
	print("Altitude:", round(gps0["alt"],2), "m")
	# print("GPS Status:", st)
	print("\n=========\n")
	
	arducopter_arm(vehicle) # Запуск моторов
	# t0 = time.time()
	arducopter_takeoff(vehicle, gps0, altitude) # Взлет
	
	location = wait_for_height(vehicle, gps0, altitude_goal) # Текущее положение
	# t1 = time.time()
	# time_takeoff  = t1 - t0
	# print("Takeoff time:", time_takeoff, "sec")

	arducopter_move(vehicle, gps0, altitude)

	# arducopter_hold(vehicle, location)
	# t0 = time.time()
	arducopter_land(vehicle) # Посадка
	location = wait_for_height_down(vehicle, gps0)
	# t1 = time.time()
	# time_land  = t1 - t0
	# print("Land time:", time_land, "sec")

if __name__ == "__main__":
	loop = asyncio.get_event_loop()
	loop.run_until_complete(run())