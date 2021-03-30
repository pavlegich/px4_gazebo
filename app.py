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
		0,0,0,0,0,coordinates[0][0],coordinates[0][1],
		coordinates[0][2]+altitude)

# Переместиться
def arducopter_move(vehicle, coordinates, altitude):
	print("---Move")
	vehicle.mav.command_long_send(
		vehicle.target_system,
		vehicle.target_component,
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
		0,0,0,0,0,coordinates[0][0]+0.00001,coordinates[0][1]+0.00001,
		coordinates[0][2]+altitude)

def arducopter_hold(vehicle):
	print("---Hold")
	vehicle.mav.command_long_send(
		vehicle.target_system,
		vehicle.target_component,
		mavutil.mavlink.MAV_CMD_NAV_DELAY,
		0,0,0,0,0,0,0,0)

# Текущая локация
def current_location(vehicle):
	vehicle.wait_heartbeat()
	return [vehicle.messages["GPS_RAW_INT"].lat*1e-7, 
	vehicle.messages["GPS_RAW_INT"].lon*1e-7,
	vehicle.messages["GPS_RAW_INT"].alt*1e-3]

# Проверка состояния спутникового сигнала
def gps_status(vehicle, last):
	status = 1
	vehicle.wait_heartbeat()
	lat = vehicle.messages["GPS_RAW_INT"].lat*1e-7
	lon = vehicle.messages["GPS_RAW_INT"].lon*1e-7
	alt = vehicle.messages["GPS_RAW_INT"].alt*1e-3
	st = vehicle.messages["GPS_RAW_INT"].satellites_visible
	if (not st):
		status = -1
	elif (st < 7 or abs(last[0][0]-lat)>0.000038 or abs(last[0][1]-lon)>0.000078 \
		 or abs(last[0][2]-alt)>5):
		status = 0
	return status, st

# Ожидание нужной высоты
def wait_for_height(vehicle, gps0, altitude):
	loc = [[]]
	loc.pop(0)
	loc.append(current_location(vehicle))
	while (loc[0][2] - gps0[0][2] < altitude - 0.03):
		# last_gps = [[loc[0][0], loc[0][1], loc[0][2]]]
		loc.pop(0)
		loc.append(current_location(vehicle))
		print(round(loc[0][2] - gps0[0][2],2), "m")
		# print(gps_status(vehicle, last_gps))
		time.sleep(0.3)
	print("---Height reached")

# Ожидание нужной высоты при приземлении
def wait_for_landing(vehicle, gps0):
	loc = [[]]
	loc.pop(0)
	loc.append(current_location(vehicle))
	while (loc[0][2] > gps0[0][2] + 0.015):
		# last_gps = [[loc[0][0], loc[0][1], loc[0][2]]]
		loc.pop(0)
		loc.append(current_location(vehicle))
		print(round(loc[0][2] - gps0[0][2],2), "m")
		# print(gps_status(vehicle, last_gps))
		time.sleep(0.3)
	print("---Landed")

# Проверка статуса сигнала
def gps_check(vehicle):
	loc = [[]]
	loc.pop(0)
	loc.append(current_location(vehicle))
	check = gps_status(vehicle, loc)
	if check[0] != 1:
		if check[0] == 0:
			print("GPS unstable!")
		else:
			print("GPS lost!")
			time.sleep(3)
			arducopter_land(vehicle)

async def run():

	gps0 = [[0, 0, 0]] # Начальные координаты
	altitude = 6 # Высота взлета
	altitude_goal = 4

	# Соединение с БВС, получение начальных координат
	vehicle = mavutil.mavlink_connection('udpin:localhost:14540')
	vehicle.wait_heartbeat()
	gps0[0][0] = vehicle.messages["GPS_RAW_INT"].lat*1e-7
	gps0[0][1] = vehicle.messages["GPS_RAW_INT"].lon*1e-7
	gps0[0][2] = vehicle.messages["GPS_RAW_INT"].alt*1e-3

	st = gps_status(vehicle, gps0)

	print("latitude:", round(gps0[0][0],2))
	print("longitude:", round(gps0[0][1],2))
	print("altitude:", round(gps0[0][2],2), "m")
	print("gps_status:", st[0])
	print("satellites_visible:", st[1])
	print("\n=========\n")
	
	arducopter_arm(vehicle) # Запуск моторов
	arducopter_takeoff(vehicle, gps0, altitude) # Взлет
	
	wait_for_height(vehicle, gps0, altitude_goal) # Текущее положение
	vehicle.set_mode_apm(2)
	print("current_location:", [round(v,2) for v in current_location(vehicle)])
	# arducopter_move(vehicle, gps0, altitude)
	# arducopter_hold(vehicle)

	t1 = time.time()
	interval = 0
	while (interval < 10):
		gps_check(vehicle)
		t2 = time.time()
		interval = t2 - t1
		time.sleep(0.3)

	# arducopter_hold(vehicle, location)
	arducopter_land(vehicle) # Посадка
	wait_for_landing(vehicle, gps0)
	print("current_location:", [round(v,2) for v in current_location(vehicle)])

if __name__ == "__main__":
	loop = asyncio.get_event_loop()
	loop.run_until_complete(run())