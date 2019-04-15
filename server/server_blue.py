#!/usr/bin/python3
#
# BlueDot controller

import RPi.GPIO as GPIO
import ultra
import socket
import time
import threading
import Adafruit_PCA9685
import picamera
from picamera.array import PiRGBArray
import findline
import speech
import cv2
from collections import deque
import numpy as np
import argparse
import imutils
from rpi_ws281x import *
import argparse
import zmq
import base64
import os
import subprocess
import pygame.mixer
from time import sleep
from os import path

from bluedot import BlueDot
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






def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
    newline=""
    str_num=str(new_num)
    with open("set.txt","r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = initial+"%s" %(str_num+"\n")
            newline += line
    with open("set.txt","w") as f:
        f.writelines(newline)

def num_import_int(initial):        #Call this function to import data from '.txt' file
    with open("set.txt") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                r=line
    begin=len(list(initial))
    snum=r[begin:]
    n=int(snum)
    return n

vtr_mid    = num_import_int('E_C1:')
hoz_mid    = num_import_int('E_C2:')
look_up_max    = num_import_int('look_up_max:')
look_down_max  = num_import_int('look_down_max:')
look_right_max = num_import_int('look_right_max:')
look_left_max  = num_import_int('look_left_max:')
turn_speed     = num_import_int('look_turn_speed:')

vtr_mid_orig = vtr_mid
hoz_mid_orig = hoz_mid
ip_con     = ''

sound_path = '/home/pi/Adeept_PiCar-B/server/sounds'


def get_ram():
    try:
        s = subprocess.check_output(['free','-m'])
        lines = s.split('\n') 
        return ( int(lines[1].split()[1]), int(lines[2].split()[3]) )
    except:
        return 0

def get_temperature():
    try:
        s = subprocess.check_output(['/opt/vc/bin/vcgencmd','measure_temp'])
        return float(s.split('=')[1][:-3])
    except:
        return 0

def get_cpu_speed():
    f = os.popen('/opt/vc/bin/vcgencmd get_config arm_freq')
    cpu = f.read()
    return cpu



def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

def rainbowCycle(strip, wait_ms=20, iterations=5):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256*iterations):
        if 'forward' in data:
            for i in range(strip.numPixels()):
                if 'forward' in data:
                    strip.setPixelColor(i, wheel((int(i * 256 / strip.numPixels()) + j) & 255))
            strip.show()
            time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, wheel((i+j) % 255))
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)

def colorWipe(strip, color):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(0.005)

def scan():                  #Ultrasonic Scanning
    global dis_dir
    dis_dir = []
    turn.ultra_turn(hoz_mid)   #Ultrasonic point forward
    turn.ultra_turn(look_left_max)   #Ultrasonic point Left,prepare to scan
    dis_dir=['list']         #Make a mark so that the client would know it is a list
    time.sleep(0.5)          #Wait for the Ultrasonic to be in position
    cat_2=look_left_max                #Value of left-position
    GPIO.setwarnings(False)  #Or it may print warnings
    while cat_2>look_right_max:         #Scan,from left to right
        turn.ultra_turn(cat_2)
        cat_2 -= 3           #This value determine the speed of scanning,the greater the faster
        new_scan_data=round(ultra.checkdist(),2)   #Get a distance of a certern direction
        dis_dir.append(str(new_scan_data))              #Put that distance value into a list,and save it as String-Type for future transmission 
    turn.ultra_turn(hoz_mid)   #Ultrasonic point forward
    return dis_dir

def scan_rev():                  #Ultrasonic Scanning
    global dis_dir
    dis_dir = []
    turn.ultra_turn(hoz_mid)   #Ultrasonic point forward
    turn.ultra_turn(look_right_max)   #Ultrasonic point Left,prepare to scan
    dis_dir=['list']         #Make a mark so that the client would know it is a list
    time.sleep(0.5)          #Wait for the Ultrasonic to be in position
    cat_2=look_right_max                #Value of left-position
    GPIO.setwarnings(False)  #Or it may print warnings
    while cat_2<look_left_max:         #Scan,from left to right
        turn.ultra_turn(cat_2)
        cat_2 += 3           #This value determine the speed of scanning,the greater the faster
        new_scan_data=round(ultra.checkdist(),2)   #Get a distance of a certern direction
        dis_dir.append(str(new_scan_data))              #Put that distance value into a list,and save it as String-Type for future transmission 
    turn.ultra_turn(hoz_mid)   #Ultrasonic point forward
    return dis_dir

def ultra_turn(hoz_mid):     #Control the direction of ultrasonic
    pwm.set_pwm(1, 0, hoz_mid)

def camera_turn(vtr_mid):    #Control the direction of Camera
    pwm.set_pwm(0, 0, vtr_mid)

def turn_left_led():         #Turn on the LED on the left
    led.turn_left(4)

def turn_right_led():        #Turn on the LED on the right
    led.turn_right(4)


def setup():                 #initialization
    motor.setup()            
    turn.ahead()
    led.setup()
   

def destroy():               #Clean up
    GPIO.cleanup()
    


def move(pos):
    if pos.top:   
        motor.motor_left(status, forward,left_spd*spd_ad)
        motor.motor_right(status,backward,right_spd*spd_ad)
        time.sleep(.20)
            
    elif pos.bottom:
        motor.motor_left(status, backward, left_spd*spd_ad)
        motor.motor_right(status, forward, right_spd*spd_ad)
        time.sleep(.20)
            
    elif pos.left:
        if led_status == 0:
            led.side_color_on(left_R,left_G)
        else:
            led.side_off(left_B)
        turn.left()
        turn_status=1
    elif pos.right:
        if led_status == 0:
            led.side_color_on(right_R,right_G)
        else:
            led.side_off(right_B)
        turn.right()
        turn_status=2
    elif pos.middle:
        turn_status = 0
        turn.middle()
        stop()

def stop():
    motor.motorStop()



def run():                   #Main loop
    global hoz_mid,vtr_mid,ip_con,led_status,auto_status,opencv_mode,findline_mode,speech_mode,auto_mode,data,addr,footage_socket,ap_status,turn_status,wifi_status
    
    
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
 
