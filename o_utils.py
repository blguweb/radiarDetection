import sensor, image, time, math, struct
from pyb import UART,LED,Timer

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

class Line(Dot):
    x_angle = 0
    y_angle = 0

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
            else:dot.angle = 0#如果x差值为0，偏移角度设为0
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

class receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0
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

