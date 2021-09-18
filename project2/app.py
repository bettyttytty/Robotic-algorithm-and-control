import capture
import light
import lightmove
import time
import math
import numpy as np

x, y = 0, 0

n = 7

# try:
while lightmove.detect_obst():
    capture.init()
    img = capture.capture(7)
    angle = light.Light(img)
    print angle
    time.sleep(1)
    dx, dy = lightmove.followlightmove(angle)

    x += dx
    y += dy

    print "angle: {} degree\nx: {}\ny: {}".format(angle, dx, dy)
#
# except KeyboardInterrupt:
#     lightmove.Ab.stop()

angle = math.atan2(y, x)

se2 = [math.cos(angle), -math.sin(angle), x,
       math.sin(angle), math.cos(angle), y,
       0, 0, 1]

se2 = np.array(se2).reshape([3, 3])

print "SE(2) for flashing light:\n", se2
