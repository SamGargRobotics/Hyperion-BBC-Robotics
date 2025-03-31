# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
 
import sensor
import time
import image
from math import *
from pyb import UART
 
# Setting up UART
uart = UART(3, 115200, timeout_char=100) #(only UART 1 or 3 available,baud rate,delay b/w frames)
uart.init(115200, bits=8, timeout_char=10)
 
# Vars
YellowIsAttack = True
thresholds = [(35,95,-35,-10,30,95)]
thresholds2 = [(35,75,-15,5,-50,-15)]
 
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(30)  # Wait for settings take effect.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()  # Create a clock object to track the FPS.
 
def GoalFind():
    """This function finds the distance on the x and y from the middle of the camera.
    _______
    Returns
    XDIST : int
    YDIST : int
    Blob.code() : int
    List : [XDIST, YDIST, blob.code()]
    """
    Temp1 = []
    Temp2 = []
    for blob in img.find_blobs(thresholds, pixels_threshold=60, area_threshold=60, merge=True):
        img.draw_rectangle(blob.rect(),color=(0,255,0))
        img.draw_cross(blob.cx(),blob.cy())
 
        YZero = round(img.height()/2)
        XZero = round(img.width()/2)
 
        img.draw_line(XZero,YZero-1,blob.cx(),YZero-1,thickness=3,color=(0,0,255)) #x dist
        img.draw_line(blob.cx(),YZero,blob.cx(),blob.cy(),thickness=3,color=(255,0,0)) #y dist
        #img.draw_arrow(XZero,YZero,blob.cx(),blob.cy(),thickness=3)
        XDIST = XZero-blob.cx()
        YDIST = YZero-blob.cy()
        #HDIST = sqrt((XDIST*XDIST)+(YDIST*YDIST))
        Temp1 = [XDIST,YDIST,0]
    for blob in img.find_blobs(thresholds2, pixels_threshold=60, area_threshold=60, merge=True):
        img.draw_rectangle(blob.rect(),color=(0,255,0))
        img.draw_cross(blob.cx(),blob.cy())
 
        YZero = round(img.height()/2)
        XZero = round(img.width()/2)
 
        img.draw_line(XZero,YZero-1,blob.cx(),YZero-1,thickness=3,color=(0,0,255)) #x dist
        img.draw_line(blob.cx(),YZero,blob.cx(),blob.cy(),thickness=3,color=(255,0,0)) #y dist
        #img.draw_arrow(XZero,YZero,blob.cx(),blob.cy(),thickness=3)
        XDIST = XZero-blob.cx()
        YDIST = YZero-blob.cy()
        #HDIST = sqrt((XDIST*XDIST)+(YDIST*YDIST))
        Temp2 = [XDIST,YDIST,1]
    return [Temp1,Temp2]
 
while True:
    #clock.tick()
    img = sensor.snapshot()
    #print(clock.fps())
    TRI=GoalFind()
    if(TRI != None):
        if(TRI[0] != []):
            uart.writechar(200)
            uart.writechar(122)
            uart.writechar(int(TRI[0][0]))
            uart.writechar(int(TRI[0][1]))
            uart.writechar(int(TRI[0][2]))
        if(TRI[1] != []):
            uart.writechar(int(TRI[1][0]))
            uart.writechar(int(TRI[1][1]))
            uart.writechar(int(TRI[1][2]))
        else:
            uart.writechar(202)