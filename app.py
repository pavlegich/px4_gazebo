# make px4_sitl gazebo_iris_opt_flow
# make px4_sitl gazebo

import asyncio
from pymavlink import mavutil
import time, sys, argparse, math

gps_ale = 0
gps_err = 0

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
		0,0,0,0,0,coordinates[0],coordinates[1],
		coordinates[2]+altitude)

# Текущая локация
def current_location(vehicle):
	vehicle.wait_heartbeat()
	return [vehicle.messages["GPS_RAW_INT"].lat*1e-7, 
	vehicle.messages["GPS_RAW_INT"].lon*1e-7,
	vehicle.messages["GPS_RAW_INT"].alt*1e-3]

# Проверка состояния спутникового сигнала
def gps_status(vehicle, last):
	status = 1
	loc = current_location(vehicle)
	st = vehicle.messages["GPS_RAW_INT"].satellites_visible
	if (not st):
		status = -1
	elif (st < 7 or abs(last[0]-loc[0])>0.000038 or abs(last[1]-loc[1])>0.000078 \
		 or abs(last[2]-loc[2])>5):
		status = 0
	return status, st

# Проверка статуса сигнала
def gps_check(vehicle, f1, f2, lid0):
	loc = current_location(vehicle)
	time.sleep(1)
	check = gps_status(vehicle, loc)
	if check[0] != 1:
		if check[0] == -1:
			f1 = 0
			if f2 < 5:
				print("GPS lost!")
				f2 += 1
			elif f2 < 12:
				vehicle.set_mode_apm(2)
				print("Prepare landing!")
				f2 += 1
			else:
				emergency_landing(vehicle, lid0)
		else:
			f2 = 0
			if f1 < 5:
				print("GPS unstable!")
				f1 += 1
			else:
				print("GPS unstable! Manual control recommended!")
				f1 += 1
	else:
		f1 = 0
		f2 = 0
	return f1, f2

# Экстренная посадка
def emergency_landing(vehicle, lid0):
	print("---Emergency landing")
	print("LiDAR used...")
	vehicle.set_mode_rtl()
	vehicle.wait_heartbeat()
	f = vehicle.motors_armed()
	t = 0
	l = 0
	while f:
		vehicle.wait_heartbeat()
		ground_distance = vehicle.messages["DISTANCE_SENSOR"].current_distance*1e-2 - lid0
		if not l:
			print(round(ground_distance, 2), "m")
			if ground_distance == 0:
				print("---Landed")
				l = 1
		f = vehicle.motors_armed()
		t += 1
		if t > 10 and t <= 20:
			print("Motors still armed!")
		elif t > 20:
			print("Motors still armed! Manual control recommended!")
		time.sleep(0.3)
	print("---Disarmed")

# Ожидание нужной высоты
def wait_for_height(vehicle, gps0, altitude):
	loc = current_location(vehicle)
	while (loc[2] - gps0[2] < altitude - 0.03):
		# last_gps = [[loc[0][0], loc[0][1], loc[0][2]]]
		loc = current_location(vehicle)
		print(round(loc[2] - gps0[2],2), "m")
		# print(gps_status(vehicle, last_gps))
		time.sleep(0.3)
	print("---Height reached")

# Ожидание нужной высоты при приземлении
def wait_for_landing(vehicle, gps0):
	loc = current_location(vehicle)
	while (loc[2] > gps0[2] + 0.015):
		# last_gps = [[loc[0][0], loc[0][1], loc[0][2]]]
		loc = current_location(vehicle)
		print(round(loc[2] - gps0[2],2), "m")
		# print(gps_status(vehicle, last_gps))
		time.sleep(0.3)
	print("---Landed")

async def run():

	altitude = 4 # Высота взлета
	altitude_goal = 3

	gps_ale = 0
	gps_err = 0

	# Соединение с БВС, получение начальных координат
	vehicle = mavutil.mavlink_connection('udpin:localhost:14540')
	vehicle.wait_heartbeat()
	gps0 = current_location(vehicle) # Начальные координаты
	lid0 = vehicle.messages["DISTANCE_SENSOR"].current_distance*1e-2
	st = gps_status(vehicle, gps0)

	print("latitude:", round(gps0[0],2))
	print("longitude:", round(gps0[1],2))
	print("altitude:", round(gps0[2],2), "m")
	print("gps_status:", st[0])
	print("satellites_visible:", st[1])
	print("\n=========\n")

	arducopter_arm(vehicle) # Запуск моторов
	arducopter_takeoff(vehicle, gps0, altitude) # Взлет
	
	wait_for_height(vehicle, gps0, altitude_goal) # Текущее положение
	vehicle.set_mode_apm(2)
	
	print("current_location:", [round(v,2) for v in current_location(vehicle)])

	t1 = time.time()
	interval = 0
	while (interval < 10):
		f = gps_check(vehicle, gps_ale, gps_err, lid0)
		gps_ale = f[0]
		gps_err = f[1]
		t2 = time.time()
		interval = t2 - t1
		time.sleep(0.3)

	# arducopter_land(vehicle)
	emergency_landing(vehicle, lid0) # Посадка
	# wait_for_landing(vehicle, gps0)
	print("current_location:", [round(v,2) for v in current_location(vehicle)])

if __name__ == "__main__":
	loop = asyncio.get_event_loop()
	loop.run_until_complete(run())