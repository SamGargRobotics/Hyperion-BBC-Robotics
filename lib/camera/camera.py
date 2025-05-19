import sensor
import time
from pyb import UART
uart = UART(3, 115200, timeout_char=100)
uart.init(115200, bits=8, timeout_char=10)
YellowIsAttack = True
thresholds = [(80,100,-40,-5,30,95)] # Yellow
thresholds2 = [(11, 34, -2, -128, -128, -12)] # Blue
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(30)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_windowing((120,120))
clock = time.clock()
def GoalFind():
    Temp1 = [0,0]
    Temp2 = [0,0]
    for blob in img.find_blobs(thresholds, pixels_threshold=35, area_threshold=35, merge=True):
        # img.draw_rectangle(blob.rect(),color=(0,255,0))
        YZero = round(img.height()/2)
        XZero = round(img.width()/2)
        Temp1 = [blob.cx(),blob.cy()]
    for blob in img.find_blobs(thresholds2, pixels_threshold=35, area_threshold=35, merge=True):
        # img.draw_rectangle(blob.rect(),color=(0,255,0))
        YZero = round(img.height()/2)
        XZero = round(img.width()/2)
        Temp2 = [blob.cx(),blob.cy()]
    return [Temp1,Temp2]
while True:
    img = sensor.snapshot()
    TRI=GoalFind()
    uart.writechar(200)
    uart.writechar(122)
    uart.writechar(int(TRI[0][0]))
    uart.writechar(int(TRI[0][1]))
    uart.writechar(int(TRI[1][0]))
    uart.writechar(int(TRI[1][1]))
    print(TRI)
