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


def oneline(pone, ptwo, pthree):
    thr = 0.03
    d1 = norm2(pone,ptwo)
    d2 = norm2(pone,pthree)
    d3 = norm2(ptwo,pthree)
    print "[[on line:]]", d1, d2, d3
    if abs (d1 + d2 - d3) > thr and abs (d1 + d3 - d2) > thr and abs (d2 + d3 - d1) > thr:
        return False
    else:
        return True

def Bug_2(start, end, rvec):
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

    # Ab.setMotor(speed, -speed)
    # time.sleep(0.2)
    # Ab.stop()

    raw_input("keep going?")
    flag = 0
    while True:
        while detect():
            Ab.setMotor(speed, -speed-2)
            current_position, current_direction = coordinate()
            distgoal = norm2(current_position, end)
            if distgoal < 0.05:
                flag = 1
                break
        Ab.stop()
        if flag == 1:
            print "reach goal"
            break
        rotation(45, 0) # assume that the car turns right every time we hit the obstacle

        raw_input("start linetracking")

        hit_position, _ = coordinate()
        hitdist = norm2(hit_position, end)
        current_position, current_direction = coordinate()
        goal_direction = end - current_position

        while True:
            af_encl = EncL
            af_encr = EncR

            while True:
                current_position, current_direction = coordinate()
                linetracking()
                straight_distance = float(EncL - af_encl + EncR - af_encr) / 2 / 40
                if straight_distance > 2 and oneline(start, end, current_position):
                    break

            current_position, current_direction = coordinate()
            leavedist = norm2(current_position, end)
            print "hitdist, leavedist", hitdist, leavedist
            Ab.stop()
            raw_input("pause")
            if oneline(start, end, current_position):
                print "on line"
                angle = get_angle(current_direction, goal_direction)
                if hitdist > leavedist:
                    turndir = np.cross(current_direction[[0, 2]], goal_direction[[0, 2]]) > 0
                    rotation(angle, turndir)

                    # if not detect():
                    #     print "hit obstacle"
                    #     rotation(get_angle(current_direction, end - current_position), not turndir)
                    break

                if abs(hitdist - leavedist) < 0.01:
                    print "fail"
                    break



    Ab.stop()

if __name__ == '__main__':
    g = goal_location()
    s, r = coordinate()
    Bug_2(s, g, r)



