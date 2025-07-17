# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor
import time
from pyb import UART

#              Yellow                          Blue
Both = [(70, 100, -31, 2, 30, 127),(49, 77, -21, -5, -38, -4)]

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(30)  # Wait for settings take effect.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
# sensor.set_auto_exposure(False)
sensor.set_windowing((120,120))
# sensor.set_brightness(0)
clock = time.clock()  # Create a clock object to track the FPS.
# Setting up UART
uart = UART(3, 115200, timeout_char=100) #(only UART 1 or 3 available,baud rate,delay b/w frames)
uart.init(115200, bits=8, timeout_char=10)

def GoalFind():
    """This function finds the distance on the x and y from the middle of the camera.
    _______
    Returns
    XDIST : int
    YDIST : int
    Blob.code() : int
    List : [XDIST, YDIST, blob.code()]
    """
    Temp1 = [0, 0]
    Temp2 = [0, 0]
    blobs = img.find_blobs(Both, pixels_threshold=35, area_threshold=35, merge=True, x_stride = 4, y_stride = 2)
    blobs = sorted(blobs, key=lambda blob: -blob.area())
    for blob in blobs:
        if(blob.code() == 1 and Temp1[0] == 0):
            img.draw_rectangle(blob.rect(),color=(255,255,0))
            Temp1 = [blob.cx(),blob.cy()]
        elif(blob.code() == 2 and Temp2[0] == 0):
            img.draw_rectangle(blob.rect(),color=(0,0,255))
            Temp2 = [blob.cx(),blob.cy()]
    return [Temp1,Temp2]

while True:
    img = sensor.snapshot()
    TRI = GoalFind()
    uart.writechar(200)
    uart.writechar(122)
    uart.writechar(int(TRI[0][0]))
    uart.writechar(int(TRI[0][1]))
    uart.writechar(int(TRI[1][0]))
    uart.writechar(int(TRI[1][1]))
    # print(TRI)
