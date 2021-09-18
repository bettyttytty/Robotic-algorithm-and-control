import RPi.GPIO as GPIO
import time
from AlphaBot import AlphaBot

Ab = AlphaBot();
Ab.stop();

cntl = 8;
cntr = 7;

# global variable to store encoder ticks
EncR = 0.0;
EncL = 0.0;

def updateEncoderL(channel):
    global EncL;
    EncL += 1;
    #print 'valEncL = %d' %EncL

    
def updateEncoderR(channel):
    global EncR;
    EncR += 1;
    #print 'valEncR = %d' %EncR


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False);
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


if __name__ == "__main__":
    '''
    1 turn of a wheel will result in 40 counts of encoder. 
    Hence, angular position of a wheel: angle = EncR/40*pi;
    The distance traveled by wheel: distance = angle * wheel_radius;
    Anologously, you can find speed of the wheel, using time library:
    t0 = time.time();
    Ab.forward();
    time.sleep(2);
    Ab.stop();
    tf = time.time();
    speed = EncR/(tf-t0); # measured in encoder ticks per second.
    ''' 
       
    R0 = EncR;
    nt = 2;
    # Make nt turns with right wheel
    while (EncR - R0)/40 < nt:
        Ab.right();
        print 'EncR = %d' %EncR;
    Ab.stop();
    

