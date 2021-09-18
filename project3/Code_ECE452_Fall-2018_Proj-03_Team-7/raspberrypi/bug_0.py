import numpy as np
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

    d = 4.3  # the distance between two wheel

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
    # Ab.setMotor(-speed, speed)
    # time.sleep(0.3)
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
    maximum = 35
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


def Bug_0(start, end, rvec):
    vec = end - start
    angle = get_angle(rvec, vec)

    print angle
    time.sleep(1)
    direction = np.cross(rvec[[0, 2]], vec[[0, 2]]) > 0
    rotation(angle, direction)

    raw_input("keep going?")

    last_hit_position = start
    count = 0
    while True:
        flag = 0
        while detect():
            Ab.setMotor(speed, -speed)
            current_position, current_direction = coordinate()
            distgoal = norm2(current_position, end)
            if distgoal < 0.05:
                flag = 1
                break

        Ab.stop()
        if flag == 1:
            print "reach goal"
            break
        if count == 0:
            rotation(120, 1)  # assume that the car turns left every time we hit the obstacle
        else:
            rotation(120, 1)
        while True:
            Ab.setMotor(speed, -speed - 2)
            if detect():
                Ab.stop()
                break

        raw_input("start linetracking")

        hit_position, hit_direction = coordinate()

        if norm2(hit_position, last_hit_position) < 0.04:
            print "fail, reach hit point again."
            Ab.stop()
            return

        last_hit_position = hit_position

        current_position, current_direction = coordinate()
        goal_direction = end - current_position

        af_encl = EncL
        af_encr = EncR

        print "go straight"
        while True:
            linetracking()
            current_position, current_direction = coordinate()
            straight_distance = float(EncL - af_encl + EncR - af_encr) / 2 / 40
            if straight_distance >= 0.5 and get_angle(current_direction, hit_direction) >= 70:
                break

        af_encl = EncL
        af_encr = EncR

        print "off straight"
        while True:
            linetracking()
            straight_distance = float(EncL - af_encl + EncR - af_encr) / 2 / 40
            print straight_distance
            if straight_distance >= 0.8:
                break

        Ab.stop()
        raw_input("move torwards goal")

        current_position, current_direction = coordinate()
        angle = get_angle(current_direction, goal_direction)
        # direction = np.cross(current_direction[[0, 2]], goal_direction[[0, 2]]) < 0
        direction = 1
        rotation(angle, direction)
        count += 1


if __name__ == "__main__":
    st, sr = coordinate()
    g = goal_location()
    Bug_0(st, g, sr)

