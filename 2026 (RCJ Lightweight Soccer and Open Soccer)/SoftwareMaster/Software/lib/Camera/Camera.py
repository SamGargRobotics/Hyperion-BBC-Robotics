# BIG BLACK MEN CHASING YOU - By: 26788 - Tue Nov 25 2025

import sensor
import time
from pyb import UART, LED
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
uart = UART(3, 115200, timeout_char=100)
clock = time.clock()

T = [(49, 100, -29, 127, 49, 127),(38, 100, -30, 127, -128, -14),(38, 100, -30, 127, -128, -14)]

while True:
    clock.tick()
    img = sensor.snapshot()

    pos = {1: [0,0], 2: [0,0], 3: [0,0]}
    blobs = img.find_blobs(T,  pixels_threshold=10, area_threshold=10, merge=True, x_stride = 2, y_stride = 2, margin=0)
    blobs = sorted(blobs, key=lambda blob: -blob.area())
    for b in blobs:
        if(b.code() == 1 and pos[1] == [0,0]):
            pos[1] = [b.cx(),b.cy()]
        elif(b.code() == 2 and pos[2] == [0,0]):
            pos[2] = [b.cx(),b.cy()]
        elif(b.code() == 3 and pos[3] == [0,0]):
            pos[3] = [b.cx(),b.cy()]

    uart.writechar(200)
    uart.writechar(122)
    uart.writechar(pos[1][0])
    uart.writechar(pos[1][1])
    uart.writechar(pos[2][0])
    uart.writechar(pos[2][1])
    uart.writechar(pos[3][0])
    uart.writechar(pos[3][1])
    print(clock.fps())
