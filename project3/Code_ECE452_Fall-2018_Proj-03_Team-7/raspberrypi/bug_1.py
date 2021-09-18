import numpy as np
import AlphaBot
import math
import RPi.GPIO as GPIO
from utils import Ab, TR
import time

from client import coordinate, goal_location

cntl = 7
cntr = 8
EncL = 0
EncR = 0
speed = 30
last_proportional = 0
integral = 0


def updateEncoderL(channel):
	global EncL
	EncL += 1


def updateEncoderR(channel):
	global EncR
	EncR += 1


def get_angle(v1, v2):
	return math.acos(np.dot(v1, v2) / (norm2(v1, 0) * norm2(v2, 0))) * 360 / 2 / math.pi

def norm2(v1, v2):
	return np.linalg.norm(v1-v2, 2)


GPIO.setup(cntr, GPIO.IN)
GPIO.setup(cntl, GPIO.IN)
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


def rotation(angle, direction):
	global EncL, EncR
	ini_encl = EncL
	ini_encr = EncR
	# print ini_encl, ini_encr

	d = 4.75  # the distance between two wheel

	if direction:  # turn left
		Ab.setMotor(-speed, -speed)
	else:
		Ab.setMotor(speed, speed)
	while True:
		distance_l = float(EncL - ini_encl) / 40 * 2 * math.pi * 1.5
		distance_r = float(EncR - ini_encr) / 40 * 2 * math.pi * 1.5
		print distance_l, distance_r, angle
		if distance_l+distance_r >= d * angle * 2 * math.pi / 360:
			break
	Ab.setMotor(-speed, speed)
	time.sleep(0.3)
	Ab.stop()


def auto_rotation():
	ct, cr = coordinate()
	g = goal_location()

	gr = g - ct
	direction = np.cross(cr[[0, 2]], gr[[0, 2]]) > 0

	if direction:
		Ab.setMotor(-speed, -speed)
	else:
		Ab.setMotor(speed, speed)

	while True:
		ct, cr = coordinate()
		gr = g - ct
		angle = get_angle(cr[[0,2]], gr[[0,2]])
		if angle < 10:
			break
	Ab.stop()


def detect():
	sensor_values = TR.readCalibrated()
	on_line = 0
	for i in range(0, TR.numSensors):
		value = sensor_values[i]
		# keep track of whether we see the line at all
		if value > 200:
			on_line = 1
	if on_line == 1:
		return False
	else:
		return True


def linetracking():
	global last_proportional
	global integral
	maximum = 30
	position = TR.readLine()
	Ab.backward()
	# The "proportional" term should be 0 when we are on the line.
	proportional = position - 2000
	# Compute the derivative (change) and integral (sum) of the position.
	derivative = proportional - last_proportional
	# integral += proportional
	last_proportional = proportional
	power_difference = proportional / 15 + derivative / 100  #+ integral/1000
	if power_difference > maximum:
		power_difference = maximum
	if power_difference < - maximum:
		power_difference = - maximum
	if power_difference < 0:
		Ab.setPWMB(maximum + power_difference)
		Ab.setPWMA(maximum)
	else:
		Ab.setPWMB(maximum)
		Ab.setPWMA(maximum - power_difference)


def Bug_1(start, end, rvec):
	# for i in range(0,400):
	# TR.calibrate()
	vec = end - start
	vec = vec[[0,2]]
	rvec = rvec[[0,2]]
	angle = get_angle(rvec, vec)

	print angle
	time.sleep(1)
	direction = np.cross(rvec, vec) > 0
	rotation(angle, direction)

	# auto_rotation()

	raw_input("keep going?")


	while detect():
		Ab.setMotor(speed, -speed-2)

	Ab.stop()
	rotation(45, 0)
	print "start linetracking."
	time.sleep(2)

	af_encl = EncL
	af_encr = EncR

	hit_position, _ = coordinate()

	distmin = np.sqrt(np.sum(np.power(hit_position - end, 2)))
	target = hit_position
	straight_distance = 0

	current_position = coordinate()
	disthit = 0

	count = 0

	while straight_distance < 10 or disthit > 0.06:
		linetracking()
		current_position, _ = coordinate()
		current_position = current_position[:3]

		dist = norm2(current_position, end)
		disthit = norm2(current_position, hit_position)
		if dist < distmin:
			distmin = dist
			target = current_position

		straight_distance = float(EncL - af_encl + EncR - af_encr) / 2 / 40
		if count % 100 == 0:
			print straight_distance, distmin, disthit
		count += 1

	Ab.stop()
	print target
	raw_input("linetracking stop")

	distarget = norm2(current_position, target)

	count = 0
	while distarget > 0.03:
		linetracking()
		current_position, current_direction = coordinate()
		distarget = norm2(current_position, target)
		if count % 100 == 0:
			print distarget
		count += 1

	Ab.stop()

	current_position, current_direction = coordinate()
	target_direction = goal_location() - current_position

	goal_angle = get_angle(current_position, target_direction)
	goal_direction = np.cross(current_direction[[0, 2]], target_direction[[0, 2]]) > 0

	flag = raw_input("reach the target point.")
	if flag != "":
		goal_direction = int(flag)

	Ab.stop()
	rotation(goal_angle, goal_direction)
	# auto_rotation()
	raw_input("go to goal point.")

	count = 0
	Ab.setMotor(speed, -speed - 2)
	while True:
		current_position, _ = coordinate()
		distgoal = norm2(current_position, end)
		if distgoal <= 0.04:
			break
		if count % 100 == 0:
			print distgoal
		count += 1

		# if not detect():
		# 	print("fail")
		# 	return 0
	Ab.stop()
	raw_input("reach the goal.")


if __name__ == '__main__':
	g = goal_location()
	s, r = coordinate()
	Bug_1(s, g, r)