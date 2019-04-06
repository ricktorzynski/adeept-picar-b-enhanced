#!/usr/bin/python3
#
# BlueDot controller

from bluedot import BlueDot
from gpiozero import Robot
from signal import pause
import motor
import turn
import led


pwm = Adafruit_PCA9685.PCA9685()    #Ultrasonic Control

dis_dir = []
distance_stay  = 0.4
distance_range = 2
led_status = 0

left_R = 15
left_G = 16
left_B = 18

right_R = 19
right_G = 21
right_B = 22

spd_ad     = 1          #Speed Adjustment
pwm0       = 0          #Camera direction 
pwm1       = 1          #Ultrasonic direction
status     = 1          #Motor rotation
forward    = 1          #Motor forward
backward   = 0          #Motor backward

left_spd   = 100         #Speed of the car
right_spd  = 100         #Speed of the car
left       = 100         #Motor Left
right      = 100         #Motor Right

spd_ad_1 = 1
spd_ad_2 = 1
spd_ad_u = 1

#Status of the car
auto_status   = 0
ap_status     = 0
turn_status   = 0

opencv_mode   = 0
findline_mode = 0
speech_mode   = 0
auto_mode     = 0

data = ''

dis_data = 0
dis_scan = 1




global hoz_mid,vtr_mid,ip_con,led_status,auto_status,opencv_mode,findline_mode,speech_mode,auto_mode,data,addr,footage_socket,ap_status,turn_status,wifi_status
led.setup()

def turn_left_led():         #Turn on the LED on the left
    led.turn_left(4)

def turn_right_led():        #Turn on the LED on the right
    led.turn_right(4)

def setup():                 #initialization
    motor.setup()            
    turn.ahead()
   

def destroy():               #Clean up
    GPIO.cleanup()
    


def move(pos):
    if pos.top:
        print("Forward")
        motor.motor_left(status, forward,left_spd*spd_ad)
        motor.motor_right(status,backward,right_spd*spd_ad)
            
    elif pos.bottom:
        print("Backward")
        motor.motor_left(status, backward, left_spd*spd_ad)
        motor.motor_right(status, forward, right_spd*spd_ad)
            
    elif pos.left:
        print("Left")
        if led_status == 0:
            led.side_color_on(left_R,left_G)
        else:
            led.side_off(left_B)
        turn.left()
        turn_status=1
    elif pos.right:
        print("Right")
        if led_status == 0:
            led.side_color_on(right_R,right_G)
        else:
            led.side_off(right_B)
        turn.right()
        turn_status=2

def stop():
    motor.motorStop()


def run():                   #Main loop
    bd = BlueDot()

    bd.when_pressed = move
    bd.when_moved = move
    bd.when_released = stop

    pause()

if __name__ == '__main__':
    colorLower = (24, 100, 100)               #The color that openCV find
    colorUpper = (44, 255, 255)               #USE HSV value NOT RGB

   

    # LED strip configuration:
    LED_COUNT      = 12      # Number of LED pixels.
    LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
    #LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
    LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
    LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
    LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
    LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
    LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

    
    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()
    setup()
    try:
        run()
    except KeyboardInterrupt:
        destroy()
 