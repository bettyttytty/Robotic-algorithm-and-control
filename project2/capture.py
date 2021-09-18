from utils import pc, set_servo1, set_servo2, np
import time

weight = np.array([0.2126, 0.7152, 0.0722])

def init():
    set_servo2(0)
    time.sleep(0.5)


def capture(n):
    r = pc.PiResolution(480, 320).pad()
    imgs = np.zeros([n, 480*320], np.uint8)
    c = pc.PiCamera(resolution=r)
    try:
        for i in range(n):
            set_servo2(180/(n-1)*i)
            time.sleep(0.5)
            tmp = np.zeros(480*320*3, np.uint8)
            c.capture(tmp, 'rgb')
            tmp = np.uint8(np.dot(tmp.reshape([-1, 3]), weight))
            imgs[i, :] = tmp
    finally:
        c.close()
    return list(imgs)