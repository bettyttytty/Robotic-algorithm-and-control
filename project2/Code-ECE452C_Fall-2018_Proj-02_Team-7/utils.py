import RPi.GPIO as GPIO
import picamera as pc
from AlphaBot import AlphaBot
import numpy as np


Ab = AlphaBot()

Ab.stop()

S1 = 27
S2 = 22
DL = 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(S1, GPIO.OUT)
GPIO.setup(S2, GPIO.OUT)
GPIO.setup(DL, GPIO.IN, GPIO.PUD_UP)


PS1 = GPIO.PWM(S1, 50)
PS1.start(50)
PS2 = GPIO.PWM(S2, 50)
PS2.start(50)


def set_servo1(angle): # vertical
    PS1.ChangeDutyCycle(12.5 - 10.0 * float(angle) / 180)


def set_servo2(angle): # horizontal
    PS2.ChangeDutyCycle(12.5 - 10.0 * float(angle) / 180)