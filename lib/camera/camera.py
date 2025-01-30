
# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor
import time
import image
from math import *

# Vars
YellowIsAttack = False
thresholds = [(35,95,-35,-10,50,95)]
thresholdsTwo = [(45,75,-15,5,-45,-15)]

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(30)  # Wait for settings take effect.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()  # Create a clock object to track the FPS.

def YellowFind():
    for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150, merge=True):
        img.draw_rectangle(blob.rect(),color=(0,255,0))
        img.draw_cross(blob.cx(),blob.cy())

        YZero = round(img.height()/2)
        XZero = round(img.width()/2)

        img.draw_line(XZero,YZero-1,blob.cx(),YZero-1,thickness=3,color=(0,0,255)) #x dist
        img.draw_line(blob.cx(),YZero,blob.cx(),blob.cy(),thickness=3,color=(255,0,0)) #y dist
        LINEONE = img.draw_arrow(XZero,YZero,blob.cx(),blob.cy(),thickness=3)

        #print("X Dist: "+str(XZero-blob.cx()))
        XDIST = XZero-blob.cx()
        if(XDIST < 0):
            #XDIST*=-1
        #print("Y Dist: "+str(YZero-blob.cy()))
        YDIST = YZero-blob.cy()
        if(YDIST < 0):
            #YDIST*=-1
        HDIST = sqrt((XDIST*XDIST)+(YDIST*YDIST))
        return [XDIST,YDIST,HDIST]
        #img.draw_string(0,0,"Blue Len: "+str(XDIST),13,color=(0,0,255))
        #img.draw_string(0,0+15,"Red Len: "+str(YDIST),13,color=(255,0,0))
        #img.draw_string(0,0+30,"White Len: "+str(HDIST),13,color=(255,255,255))


def BlueFind():
    for blob in img.find_blobs(thresholdsTwo, pixels_threshold=150, area_threshold=150, merge=True):
        img.draw_rectangle(blob.rect(),color=(0,255,0))
        img.draw_cross(blob.cx(),blob.cy())

        YZero = round(img.height()/2)
        XZero = round(img.width()/2)

        img.draw_line(XZero,YZero-1,blob.cx(),YZero-1,thickness=3,color=(0,0,255)) #x dist
        img.draw_line(blob.cx(),YZero,blob.cx(),blob.cy(),thickness=3,color=(255,0,0)) #y dist
        LINEONE = img.draw_arrow(XZero,YZero,blob.cx(),blob.cy(),thickness=3)

        #print("X Dist: "+str(XZero-blob.cx()))
        XDIST = XZero-blob.cx()
        if(XDIST < 0):
            #XDIST*=-1
        #print("Y Dist: "+str(YZero-blob.cy()))
        YDIST = YZero-blob.cy()
        if(YDIST < 0):
            #YDIST*=-1
        HDIST = sqrt((XDIST*XDIST)+(YDIST*YDIST))
        return [XDIST,YDIST,HDIST]
        #img.draw_string(0,0,"Blue Len: "+str(XDIST),13,color=(0,0,255))
        #img.draw_string(0,0+15,"Red Len: "+str(YDIST),13,color=(255,0,0))
        #img.draw_string(0,0+30,"White Len: "+str(HDIST),13,color=(255,255,255))


while True:
    clock.tick()
    img = sensor.snapshot()
    #print(clock.fps())

    if(YellowIsAttack == True):
        TRI=YellowFind()
    else:
        TRI=BlueFind()


