'''
@作者：kidominox

0:开机
1：垂直上升
2：openmv识别
3：雷达得出两杆位置
4：规划路径
5：终点识别

2、飞控串口通信数据传输格式如下：
---------------------------------------------------------
|  0   |  1   |  2   |  3   |  4   |  5   |  6   |  7   |
---------------------------------------------------------
| 0xAA | 0xAF | 0xF2 | 0x00 | SDK_point.x | SDK_point.y |
---------------------------------------------------------
|  8   |  9   |   10  |  11  |  12  |   13   |  14  |      |
---------------------------------------------------------
|SDK_point.ang| S_p_f | det_pixel   | Status | 0x00 |      |      |
---------------------------------------------------------
nano->fly(Single bit)
---------------------------------------------------------
|  0   |  1   |  2   |  3   |  4   |  5   |  6   |  7   |
---------------------------------------------------------
| 0xAA | 0xAF | 0x01 | 0x00 |basica|a_num |Unknow| 0x00 |
---------------------------------------------------------
|  8   |  9   |  10  |  11  |  12  |  13  |      |      |
---------------------------------------------------------
||    | |  |      |      |
---------------------------------------------------------

SDK_point.x：识别到目标的中心点横坐标
SDK_point.y：识别到目标的中心点纵坐标
SDK_point.ang：识别到的目标的偏移量
det_pixel：未知
Status: 0-5
S_p_f：识别目标的标志位，1表示识别到，0表示未识别到。
basica:移动方式标志位 Pitch(0x01)/Roll(0x02)/Yaw(0x03)
a_num:移动量

3、openmv串口通信数据传输格式如下：
nano->openmv
---------------------------------------------------------
|  0   |  1   |  2   |  3   |  4   |  5   |  6   |  7   |
---------------------------------------------------------
| 0xAA | 0xAF | 0x02 | 0x00 |Status| 0x00 |      |      |
---------------------------------------------------------
|  8   |  9   |  10  |  11  |  12  |  13  |      |      |
---------------------------------------------------------
||    | |  |      |      |
'''

import math
import ser
from lsp import lsp



class Status:
    START = 0
    TAKE_OFF = 1
    OPENMV = 2
    RADIAR =3
    POSITION = 4
    ULTI = 5



class Receive:
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0

class Nano:

    CW = 0#顺时针
    CCW = 1#逆时针
    

    def __init__(self,_status,u1,u2):
        self.status = _status
        self.rx = 0
        self.ry = 0
        self.gx = 0
        self.gy = 0
        self.R = Receive()
        self.openmvuart = u1
        self.droneuart = u2
        self.x = 0
        self.y = 0
        self.angle =0
    
    def process(self):
        if self.status == 0:
            result = DWritePort(ser,str(self.status))
            #print(result)
            #开机
            #改变飞控和openmv的状态0
        elif self.status == 1:#从飞控接收到状态1 进入
            result = DWritePort(ser,str(self.status))
            #print(result)
            #改变openmv状态1
        elif self.status == 2:#从飞控接收到状态2 进入
            result = DWritePort(ser,str(self.status))#通知openmv进行转机
            #print(result)
        elif self.status == 3:#从openmv接收到状态3 进入
            #self.droneuart.write(pack_drone_data(0))
            #uart.write(pack_drone_data(1))
            #处理openmv的角度
            self.rx,self.ry,self.gx,self.gy = lsp(self.x,self,y,self.angle,self.redangle,self.greendangle)
            self.status =4
            process()
            #x,y,th,mv_red_th,mv_green_th
            #改变飞控状态3
        elif self.status == 4:
            #self.droneuart.write(pack_drone_data(0))
            result = DWritePort(ser,str(self.status))
            print(result)
            plan_road()
            self.status = 5
            process()
        elif self.status == 5:#从飞控接收到状态5 进入
            result = DWritePort(ser,str(self.status))
            #通知openmv进行转机 头向下
        ##maybe is wrong
        while not u1.any():
            pass
        uart_read_buf()
    
    def plan_road(self):
        #先红色顺时针，后绿色逆时针
        red_angle = arctan(self.ry/self.rx) - 2*pi##
        ao_x = self.rx + cos(red_angle)
        ao_y = self.ry + sin(red_angle)
        ##
        
        uart.write(pack_drone_data(1))
        for i in rang(12):
            ao_x = self.rx + cos(red_angle - i * 30)
            ao_y = self.ry + sin(red_angle - i * 30) 
            send(ao_x,ao_y)
        ##
        green_angle = arctan(self.gy/self.gx) - 2*pi##
        ao_x = self.gx + cos(green_angle)
        ao_y = self.gy + sin(green_angle)
        send(ao_x,ao_y)
        for i in rang(12):
            ao_x = self.gx + cos(green_angle + i * 30)
            ao_y = self.gy + sin(green_angle + i * 30) 
            send(ao_x,ao_y)
        
        ##最后给出终点的坐标
        send()
    
    
        
    
    #实时接受数据改变status 实process
    ##存储位置角度和距离self.x =  status = 4

    #打包openmv的数据
    def pack_openmv_data(self):

        pack_data=bytearray([0xAA,0xAF,0x02,0x00,
        self.status,0x00])

        
        lens = len(pack_data)#数据包大小
        pack_data[3] = lens-5
        i = 0
        sum = 0

        #和校验
        while i<(lens-1):
            sum = sum + pack_data[i]
            i = i+1
        pack_data[lens-1] = sum
        #print("pack",pack_data)
        return pack_data
    
    

    #打包飞控的数据
    def pack_drone_data(self,flag):
        '''
        dot_x = dot.x#识别到目标的中心点横坐标
        dot_y = dot.y#识别到目标的中心点纵坐标
        ####wrong
        point_angle = dot.angle#识别到的目标的偏移量
        det_pixel = 0#未知
        if flag == 0:
            pack_data=bytearray([0xAA,0xAF,0xF2,0x00,
            dot_x>>8,dot_x,
            dot_y>>8,dot_y,
            point_angle>>8,point_angle,
            dot.flag,
            det_pixel>>8,det_pixel,
            self.status,0x00])
            
        elif flag == 1:
            pack_data=bytearray([0xAA,0xAF,0x01,0x00,
            basica,
            a_num,
            Unknow,0x00])
            ####wrong
        lens = len(pack_data)#数据包大小
        pack_data[3] = lens-5

        i = 0
        sum = 0

        #和校验
        while i<(lens-1):
            sum = sum + pack_data[i]
            i = i+1
        pack_data[lens-1] = sum
        
        if (dot.flag&0x00)==0:
            print("dot:",dot.x,",",dot.y)
            print("angle",dot.angle)
            #print("pack",pack_data)
        '''
        return pack_data

    #串口通信协议接收
    def Receive_Prepare(self,data):

        if self.R.state==0:

            if data == 0xAA:#帧头
                self.R.state = 1
                self.R.uart_buf.append(data)
            else:
                self.R.state = 0

        elif self.R.state==1:
            if data == 0xAF:#帧头
                self.R.state = 2
                self.R.uart_buf.append(data)
            else:
                self.R.state = 0

        elif self.R.state==2:
            if data <= 0xFF:#数据个数
                self.R.state = 3
                self.R.uart_buf.append(data)
            else:
                self.R.state = 0

        elif self.R.state==3:
            if data <= 33:
                self.R.state = 4
                self.R.uart_buf.append(data)
                self.R._data_len = data
                self.R._data_cnt = 0
            else:
                self.R.state = 0

        elif self.R.state==4:
            if self.R._data_len > 0:
                self.R. _data_len = self.R._data_len - 1
                self.R.uart_buf.append(data)
                if self.R._data_len == 0:
                    self.R.state = 5
            else:
                self.R.state = 0

        elif self.R.state==5:
            self.R.state = 0
            self.R.uart_buf.append(data)
            Receive_Anl(self.R.uart_buf,self.R.uart_buf[3]+5)
            self.R.uart_buf=[]#清空缓冲区，准备下次接收数据
        else:
            self.R.state = 0

    #读取串口缓存
    def uart_read_buf(self):
        
        # self.R.state = 0
        #不会同步

        if self.openmvuart.any():
            i = 0
            buf_size = self.openmvuart.any()
            while i<buf_size:
                Receive_Prepare(self.openmvuart.readchar())
                i = i + 1
        elif self.droneuart.any():
            i = 0
            buf_size = self.droneuart.any()
            while i<buf_size:
                Receive_Prepare(self.droneuart.readchar())
                i = i + 1
    
    #


    #串口数据解析
    def Receive_Anl(self,data_buf,num):
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

        if data_buf[2]==0x04:
            print("receive openmv ok!")
            if data_buf[4] == 0x00:#红色
                strangle = data_buf[5]*0xFF+data_buf[6]
                self.redangle = hex_dec(strangle)
            elif data_buf[4] == 0x01:#绿色
                strangle = data_buf[5]*0xFF+data_buf[6]
                self.greendangle = hex_dec(strangle)
            self.status = hex_dec(data_buf[7])
            process()
        ##wrong 还没写与飞控的数据

        if data_buf[2]==0x02:
            print("receive drone ok!")

        if data_buf[2]==0xF1:

            #设置模块工作模式
            ctr.work_mode = data_buf[4]

            print("Set work mode success!")


def hex_dec(str2): #十六转十                                                                                                                       
    b = eval(str2)                                                                                                                             
    print('十六进制： %s 转换成十进制为：%s:'  %(str2,b))                                                                                                   
    return b

if __name__=="__main__":
    ser,ret=DOpenPort("/dev/ttyUSB0",19200,None)
    #ser=serial.Serial("/dev/ttyUSB0",19200,timeout=0.5) #使用USB连接串行口
    if(ser.is_open):#判断串口是否成功打开
        print("serial open")
        while(1):
            str=DReadPort()
            print(str)
         #DReadPort() #读串口数据
         #DColsePort(ser)  #关闭串口
    nano =Nano(0,u1,uw)