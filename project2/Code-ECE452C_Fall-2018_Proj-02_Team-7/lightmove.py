from __future__ import print_function
import math
import time
import sys

from utils import Ab, GPIO

cntl = 7
cntr = 8
EncL = 0
EncR = 0
DL = 19

def updateEncoderL(channel):
    global EncL
    EncL += 1


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def updateEncoderR(channel):
    global EncR
    EncR += 1


GPIO.setup(cntr, GPIO.IN)
GPIO.setup(cntl, GPIO.IN)
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


def detect_obst():
    DL_status = GPIO.input(DL)
    return DL_status == 1


def followlightmove(angle):
    global EncL, EncR
    ini_encl = EncL
    ini_encr = EncR
    speed = 30

    print("Starting.")

    time.sleep(0.5)
    d = 5  # the distance between two wheel
    threshold = 3 * math.pi
    k = 0.2

    if angle > 90:
        Ab.setMotor(0, speed)
        count = 0
        while True:
            distance_l = float(EncL - ini_encl) / 40 * 2 * math.pi * 1.5
            if count % 200 == 0:
                eprint("l:", distance_l)
            if distance_l >= d * abs(angle - 90) * 2 * math.pi / 360:
                break
            count += 1
        Ab.stop()

    else:
        Ab.setMotor(-speed, 0)
        count = 0
        while True:
            distance_r = float(EncR - ini_encr) / 40 * 2 * math.pi * 1.5
            if count % 200 == 0:
                eprint("r:", distance_r)
            if distance_r >= d * abs(angle - 90) * 2 * math.pi / 360:
                break
            count += 1
        Ab.stop()

    af_encl = EncL
    af_encr = EncR

    Ab.setMotor(-speed, speed+2)

    count = 0
    straight_distance = 0

    while detect_obst():
        straight_distance = float(EncL - af_encl + EncR - af_encr) / 2 / 40 * 2 * math.pi * 1
        if count % 100 == 0:
            eprint("straight:", straight_distance)
        count += 1
        if straight_distance > threshold:
            break

    Ab.stop()

    y = math.cos(angle * 2 * math.pi / 360) * straight_distance
    x = -math.sin(angle * 2 * math.pi / 360) * straight_distance / 1.5
    return x, y


    # ini_encl = EncL
    # ini_encr = EncR
    # speed = 35
    # print("Starting.")
    # time.sleep(0.5)
    #
    # k = 0.1
    #
    # Ab.forward()
    #
    # while True:
    #     distance = float(EncL - ini_encl + EncL - ini_encr) / 2 / 40 * 2 * math.pi * 1
    #     power_difference = abs(angle - 90)
    #
    #     if angle > 90:
    #         Ab.setPWMA(speed)
    #         Ab.setPWMB(speed + k * power_difference)
    #
    #     else:
    #         Ab.setPWMA(speed + k * power_difference)
    #         Ab.setPWMB(speed)
    #
    #     if distance > 2 * math.pi:
    #         Ab.stop()
    #         break


