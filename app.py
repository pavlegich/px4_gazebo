# make px4_sitl gazebo_iris_opt_flow
# make px4_sitl gazebo

import asyncio
from pymavlink import mavutil
import time, sys, argparse, math

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
def gps_check(vehicle):
	status = 1
	vehicle.wait_heartbeat()
	st = vehicle.messages["GPS_RAW_INT"].satellites_visible
	if (not st):
		status = -1
	# elif (sv < 7 or abs(lat-myUAV['lat'])>0.000038 or abs(lon-myUAV['lon'])>0.000078 \
	# 	 or abs(alt-myUAV['alt'])>5))
	return status

# Ожидание нужной высоты
def wait_for_height(vehicle, gps0, altitude):
	loc = [[gps0["lat"], gps0["lon"], gps0["alt"]]]
	while (loc[0][2] - gps0["alt"] < altitude):
		loc.pop(0)
		loc.append(current_location(vehicle))
		print(round(loc[0][2] - gps0["alt"],2), "m")
		time.sleep(1)
	return loc


async def run():

	gps0 = {"lat" : 0, "lon" : 0, "alt" : 0} # Начальные координаты
	altitude = 3 # Высота взлета

	# Соединение с БВС, получение начальных координат
	vehicle = mavutil.mavlink_connection('udpin:localhost:14540')
	vehicle.wait_heartbeat()
	gps0["lat"] = vehicle.messages["GPS_RAW_INT"].lat*1e-7
	gps0["lon"] = vehicle.messages["GPS_RAW_INT"].lon*1e-7
	gps0["alt"] = vehicle.messages["GPS_RAW_INT"].alt*1e-3

	st = gps_check(vehicle)

	print("Latitude:", round(gps0["lat"],2))
	print("Longitude:", round(gps0["lon"],2))
	print("Altitude:", round(gps0["alt"],2), "m")
	print("GPS Status:", st)
	print("\n=========\n")
	
	arducopter_arm(vehicle) # Запуск моторов
	arducopter_takeoff(vehicle, gps0, altitude) # Взлет
	location = wait_for_height(vehicle, gps0, altitude) # Текущее положение
	# arducopter_hold(vehicle, location)
	arducopter_land(vehicle) # Посадка


if __name__ == "__main__":
	loop = asyncio.get_event_loop()
	loop.run_until_complete(run())