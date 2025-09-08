# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import sensor
import time
from pyb import UART, LED

red = LED(1)
green = LED(2)
blue = LED(3)

#              Yellow                          Blue
Both = [(70, 97, -22, 8, 21, 127),(41, 81, -128, 127, -128, -21)]
red.on()
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
# sensor.set_auto_exposure(False, exposure_us=50000)
sensor.set_auto_exposure(False, exposure_us=20000)
sensor.set_windowing((120,120))
red.off()
clock = time.clock()  # Create a clock object to track the FPS.

# Setting up UART
uart = UART(3, 115200, timeout_char=100) #(only UART 1 or 3 available,baud rate,delay b/w frames)

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
    blobs = img.find_blobs(Both,  pixels_threshold=10, area_threshold=10, merge=True, x_stride = 2, y_stride = 2, margin=0)
    blobs = sorted(blobs, key=lambda blob: -blob.area())
    for blob in blobs:
        if(blob.code() == 1 and Temp1[0] == 0):
            Temp1 = [blob.cx(),blob.cy()]
        elif(blob.code() == 2 and Temp2[0] == 0):
            Temp2 = [blob.cx(),blob.cy()]
    return [Temp1,Temp2]

while True:
    clock.tick()
    img = sensor.snapshot()
    Temp1 = [0, 0]
    Temp2 = [0, 0]
    blobs = img.find_blobs(Both,  pixels_threshold=10, area_threshold=10, merge=True, x_stride = 2, y_stride = 2, margin=0)
    blobs = sorted(blobs, key=lambda blob: -blob.area())
    # red.off()
    # green.off()
    # blue.off()
    for blob in blobs:
        if(blob.code() == 1 and Temp1[0] == 0):
            img.draw_rectangle(blob.rect(),color=(255,255,0))
            Temp1 = [blob.cx(),blob.cy()]
            # red.on()
            # green.on()
        elif(blob.code() == 2 and Temp2[0] == 0):
            img.draw_rectangle(blob.rect(),color=(0,0,255))
            Temp2 = [blob.cx(),blob.cy()]
            # blue.on()

    uart.writechar(200)
    uart.writechar(122)
    uart.writechar(Temp1[0])
    uart.writechar(Temp1[1])
    uart.writechar(Temp2[0])
    uart.writechar(Temp2[1])


    print(Temp2)
    # img.draw_line((0, 0, Temp1[0][0], Temp1[0][1]))
    # img.draw_line((0, 0, Temp2[1][0], Temp2[1][1]))
    # print(clock.fps())
