# openmvutils.py - By: kidominox - 周一 10月 12 2020
##
import sensor, image, time,pyb
from pyb import UART,LED,Timer
import math, struct
from pyb import Servo
from pid import PID
from pyb import Pin
clock = time.clock()
'''
led_red = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
led_green  = pyb.LED(2)
global dete_red = 0
global dete_green = 0
pan_servo=Servo(1)
tilt_servo=Servo(2)
p_out = Pin('P7', Pin.OUT_PP)#设置p_out为输出引脚
p_out.low()
pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
'''
class ctrl(object):
    work_mode = 0x01 #工作模式.默认是起飞
    check_show = 1   #开显示，在线调试时可以打开，离线使用请关闭，可提高计算速度

class Dot(object):
    x = 0
    y = 0
    pixels = 0
    num = 0
    ok = 0
    flag = 0
    angle = 0

class receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0

class Line(Dot):
    x_angle = 0
    y_angle = 0


pan_servo=Servo(1)
tilt_servo=Servo(2)
pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

def down_cam():
    pan_servo.angle(-20)
    tilt_servo.angle(-90)
    print(2)

def detection(uart):
    print("start detection")

    clock = time.clock()
    led_red = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
    led_green  = pyb.LED(2)
    dete_red = 0
    dete_green = 0

    p_out = Pin('P2', Pin.OUT_PP)#设置p_out为输出引脚
    p_out.low()


    #pan_servo=Servo(1)
    #tilt_servo=Servo(2)
    pan_servo.angle(-20)#底座
    tilt_servo.angle(10)

    while(dete_green != 1 or dete_red != 1):
        clock.tick()
        img = sensor.snapshot().lens_corr(1.8)
        pan_servo.angle(pan_servo.angle()+5)
        print(pan_servo.angle())
        #tilt_servo.angle(tilt_servo.angle()+5)
        print(tilt_servo.angle())
        time.sleep(200)

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
                    time.sleep(200)
                    led_red.off()
                    i=i-1
            elif 24 < statistics.l_mode() < 48 and -48<statistics.a_mode()<-24 and -1<statistics.b_mode()<49 and dete_green == 0:
                img.draw_rectangle(area, color = (0, 255, 0))
                dete_green = 1
                print("green")
                uart.write("green")
                j = 3
                while(j):
                    p_out.high()#设置p_out引脚为高
                    j=j-1
                p_out.low()

                k = 5
                while(k):
                    led_green.on()
                    time.sleep(200)
                    led_green.off()
                    k=k-1
    pan_servo.angle(10)
    tilt_servo.angle(-20)
    uart.write('3')
    return 1

#起飞检测数据打包
def pack_take_off_data():

    dot_x = dot.x#识别到目标的中心点横坐标
    dot_y = dot.y#识别到目标的中心点纵坐标
    point_angle = dot.angle#识别到的目标的偏移量
    det_pixel = 0#未知


    pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
        dot_x>>8,dot_x,
        dot_y>>8,dot_y,
        point_angle>>8,point_angle,
        det_pixel>>8,det_pixel,
        dot.flag,0x00])

    lens = len(pack_data)#数据包大小
    pack_data[3] = lens-5;#有效数据个数

    i = 0
    sum = 0

    #和校验
    while i<(lens-1):
        sum = sum + pack_data[i]
        i = i+1
    pack_data[lens-1] = sum;2
    if (dot.flag&0x00)==0:
        print("dot:",dot.x,",",dot.y)
        print("angle",dot.angle)
        #print("pack",pack_data)
    return pack_data

#起飞检测函数
def check_take_off(img):
    max_rect_area=0#最大矩形面积
    for r in img.find_rects(threshold = 10000):
        #获取检测到的矩形中最大面积矩形的信息
        if max_rect_area<r.magnitude():
            max_rect_area=r.magnitude()#更新最大面积
            tar_rect=r
            sum_x=0#x值之和
            sum_y=0#y值之和
            for p in r.corners():
                #累加四个顶点的坐标值
                sum_x+=p[0]
                sum_y+=p[1]
            print(sum_x)
            #中心点坐标
            dot.x=int(sum_x/4)
            dot.y=int(sum_y/4)
            #求偏移角
            x_cha = r.corners()[1][0]-r.corners()[0][0]#x差值
            y_cha = r.corners()[1][1]-r.corners()[0][1]#y差值
            for p in r.corners(): img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))
            print(x_cha,y_cha)

            if x_cha !=0:
                print(math.atan(y_cha/x_cha))
                dot.angle=int((math.atan(y_cha/x_cha)/3.14)*180)#求arctan函数
            else:
                dot.angle = 0#如果x差值为0，偏移角度设为0
            dot.ok= 1

    #判断标志位
    dot.flag = dot.ok

    #清零标志位
    dot.pixels = 0
    dot.ok = 0

    #发送数据
    uart.write(pack_take_off_data())

#串口数据解析
def Receive_Anl(data_buf,num):

    #和校验
    sum = 0
    i = 0
    while i<(num-1):
        sum = sum + data_buf[i]
        i = i + 1

    sum = sum%256 #求余
    if sum != data_buf[num-1]:
        return
    #和校验通过

    if data_buf[2]==0x01:
        print("receive 1 ok!")

    if data_buf[2]==0x02:
        print("receive 2 ok!")

    if data_buf[2]==0xF1:

        #设置模块工作模式
        ctr.work_mode = data_buf[4]

        print("Set work mode success!")


#串口通信协议接收
def Receive_Prepare(data):

    if R.state==0:

        if data == 0xAA:#帧头
            R.state = 1
            R.uart_buf.append(data)
        else:
            R.state = 0

    elif R.state==1:
        if data == 0xAF:#帧头
            R.state = 2
            R.uart_buf.append(data)
        else:
            R.state = 0

    elif R.state==2:
        if data <= 0xFF:#数据个数
            R.state = 3
            R.uart_buf.append(data)
        else:
            R.state = 0

    elif R.state==3:
        if data <= 33:
            R.state = 4
            R.uart_buf.append(data)
            R._data_len = data
            R._data_cnt = 0
        else:
            R.state = 0

    elif R.state==4:
        if R._data_len > 0:
            R. _data_len = R._data_len - 1
            R.uart_buf.append(data)
            if R._data_len == 0:
                R.state = 5
        else:
            R.state = 0

    elif R.state==5:
        R.state = 0
        R.uart_buf.append(data)
        Receive_Anl(R.uart_buf,R.uart_buf[3]+5)
        R.uart_buf=[]#清空缓冲区，准备下次接收数据
    else:
        R.state = 0


#读取串口缓存
def uart_read_buf():
    i = 0
    buf_size = uart.any()
    while i<buf_size:
        Receive_Prepare(uart.readchar())
        i = i + 1

#定时器中断
def time_irq(timer):
    print(fps)

#计算最优灰度阈值
def img_duty(img):

    stat=img.get_statistics()

    thresholds[1] = (int)(stat.mean()/2)

def take_up_down(uart):
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.set_contrast(3)#对比度
    #clock = time.clock()#初始化时钟

    ctr=ctrl()

    rad_to_angle = 57.29#弧度转度

    thresholds = [0, 90]#自定义灰度阈值
    #定义采样区
    up_roi   = [0,   0, 160, 10]#上采样区0
    down_roi = [0, 110, 160, 10]#下采样区0
    left_roi = [0,   0,  10, 120]#左采样区0
    righ_roi = [150, 0,  10, 120]#右采样区0
    dot  = Dot()#地标中心点类
    tar_rect=0
    R=receive()
    fps = 0
    #定时器频率1HZ中断
    #tim = Timer(4, freq=1) # 初始化定时器
    #tim.callback(time_irq) # 定时器中断回调函数
    while (uart.any() and uart.read() != 'end'):

        clock.tick()
        img = sensor.snapshot()

        #计算最优灰度阈值（如果希望使用自定义阈值，请把该函数注释掉）
        img_duty(img)

        #图像二值化（仅在线调试时使用，实际上机运行时请把该函数注释掉，可以提高计算速度）
        #img.binary([thresholds],invert=True)

        #起飞检测
        if (ctr.work_mode&0x01)!=0:
            check_take_off(img)


        #可视化显示
        if ctr.check_show and (dot.flag==1):
            img.draw_circle(dot.x, dot.y, 5, color = (0, 255, 0))

        #接收串口数据
        uart_read_buf()

        #计算程序运行频率
        fps=int(clock.fps())

        #LED灯闪烁
        #led1.toggle()
