import sensor, image, time, pyb
from pid import PID
from pyb import Servo
from pyb import UART

uart = UART(3, 19200)
usb = pyb.USB_VCP()



led_red = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
led_green  = pyb.LED(2)

pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
import sensor, image, time, pyb
from pid import PID
from pyb import Servo
from pyb import Pin
usb = pyb.USB_VCP()

pan_servo=Servo(1)
tilt_servo=Servo(2)

led_red = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
led_green  = pyb.LED(2)

pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
p_out = Pin('P7', Pin.OUT_PP)#设置p_out为输出引脚
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
dete_red = 0
dete_green = 0

while(dete_green != 1 or dete_red != 1):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)
    pan_servo.angle(pan_servo.angle()+2)

    print(pan_servo.angle())
    #tilt_servo.angle(tilt_servo.angle()+2)
    #print(tilt_servo.angle())
    for r in img.find_rects(threshold = 10000):
        # for p in r.corners(): img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))
        # print(r)
        area = (r.x(), r.y(), r.w(), r.h())
        statistics = img.get_statistics(roi=area)#像素颜色统计
        # print(statistics)
        if 17<statistics.l_mode()<87 and 30<statistics.a_mode()<123 and -49<statistics.b_mode()<50 and dete_red == 0:#if the circle is red
            img.draw_rectangle(area, color = (255, 0, 0))
            dete_red = 1 #识别到的红色圆形用红色的圆框出来
            print("red")
            uart.write("red")
            j = 3
            while(j):
                p_out.high()#设置p_out引脚为高
                j=j-1
            p_out.low()
            i = 5
            while(i):
                led_red.on()
                time.sleep(1000)
                led_red.off()
                i=i-1
        elif 24<statistics.l_mode()<48 and -48<statistics.a_mode()<-24 and -1<statistics.b_mode()<49 and dete_green == 0:
            img.draw_rectangle(area, color = (0, 255, 0))
            dete_green = 1
            print("green")
            uart.write("green")
            j = 3
            while(j):
                p_out.high()#设置p_out引脚为高
                j=j-1
            p_out.low()
            i = 5
            while(i):
                led_green.on()
                time.sleep(1000)
                led_green.off()
                i=i-1
    # print("FPS %f" % clock.fps())

