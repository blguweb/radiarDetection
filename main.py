# main.py - By: kidominox - 周一 10月 12 2020
##

import sensor, image, time ,pyb
from pid import PID
from pyb import Servo
from pyb import UART
uart = UART(3, 19200)
usb = pyb.USB_VCP()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking




#led_red = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
#led_green  = pyb.LED(2)





'''
0:开机
1：垂直上升
2：openmv识别
3：雷达得出两杆位置
4：规划路径
5：终点识别
'''

class Status:
    START = '0'
    TAKE_OFF = '1'
    OPENMV = '2'
    RADIAR = '3'
    POSITION = '4'
    TARGET = '5'

state = Status()
flag = 0
print("start")
while(True):

    img = sensor.snapshot()
    while not (uart.any()):
        pass
    status = uart.read()
    #print(type(status))
    #print(status)
    #print("main")
    #status = b'2'
    if status == b'0':#机头向下
        print("int")
        openmvutils.down_cam()
        #openmvutils.take_up_down(uart)
    elif status == b'1':
        pass
    elif status == b'2':
        #机头向前复位
        result = openmvutils.detection(uart)
        if result == 0:
            print("wrong")
        elif result == 1:
            print("success")

    elif status == b'3':
        pass
    elif status == b'4':
        pass
    elif status == b'5':
        #机头向下
        if flag ==0:
            openmvutils.down_cam()
            flag = 1
        #openmvutils.take_up_down(uart)
