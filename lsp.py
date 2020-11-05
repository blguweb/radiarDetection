#coding=utf-8
import os
import math


#from tqdm import tqdm

def dis2int(dis):
    
    #将距离从字符串转换成整形
    int_dis=0
    for i in range(5):
        int_dis += int(int(dis[i])*math.pow(10, 4 - i))
    return int_dis

def lsp(x,y,th,mv_red_th,mv_green_th):#x与y为五人机绝对坐标系，后二者为mv返回大致角度值（为绝对坐标系中y轴方向顺时针转动角度）
    '''
    
    '''
    times=30#雷达扫描次数
    num_data_line=600#雷达数据行数
       
    data_ls = [[round(0.6 * x,2),0] for x in range(num_data_line)]#创建存储雷达数据的数组

    error_range = 6#mv角度数据误差（度）

    mv_red_th //= 0.6
    mv_green_th //= 0.6
    mv_red_th = int(mv_red_th) #取整
    mv_green_th = int(mv_green_th) #取整
    #for i in tqdm(range(times)):#多次扫描
    for i in range(times):#多次扫描
        #执行雷达扫视代码
        os.system("./simple_grabber /dev/ttyUSB0")
	
        
        with open("data.txt", "r") as f:
            for line in f.readlines():
                angle_dis = line.strip('\n').split(" ")  #去掉列表中每一个元素的换行符,以空格作为分割符号，分割角度和距离
                #雷达数据下标
                j = int(float(angle_dis[0]) // 0.6)
                #print(j)
                dis = dis2int(angle_dis[1])
                if dis != 0:
                    if ((x + dis * math.sin(math.radians(float(angle_dis[0])))) < 4300) & ((x + dis * math.sin(math.radians(float(angle_dis[0])))) > 0):
                        if ((y + dis * math.cos(math.radians(float(angle_dis[0])))) < 3250) & ((y + dis * math.cos(math.radians(float(angle_dis[0])))) > 0):
                            data_ls[j][1]=dis            #更新数据
                #angle_dis = deal_ls_data(angle_dis,data_ls[j])  #处理角度和距离数据
                
                j += 1
    for i in range(num_data_line):
        if data_ls[i][1] != 0:
            print(i," ",data_ls[i])
    #print("len",len(data_ls))

    mpr = []
    mpg = []

    f = 0.
    a = 0.
    b = 0.
    print("mv",mv_red_th)
    for i in range(1,11):
        if data_ls[mv_red_th - 6 + i][1] != 0:
            mpr.append(data_ls[mv_red_th - 6 + i])
            print("mpr",mpr)
            f += 1

    for i in range(1,int(f)+1):
        a += mpr[i - 1][0]
        b += mpr[i - 1][1]
    print("f",f)
    if f!=0:decision_red = [a / f,b / f]
    else : decision_red = [0,0]
    print("de",decision_red)
    f = 0.
    a = 0.
    b = 0.
    print("mv",mv_green_th)
    for i in range(1,11):
        if data_ls[mv_green_th - 6 + i][1] != 0:
            mpg.append(data_ls[mv_green_th - 6 + i])
            print("mpg",mpg)
            f += 1
    for i in range(1,int(f)+1):
        a += mpg[i - 1][0]
        b += mpg[i - 1][1]
    print("f",f)
    if f!=0:decision_green = [a / f,b / f]
    else: decision_green = [0,0]

    return (x + decision_red[1] * math.sin(math.radians(decision_red[0]))),(y + decision_red[1] * math.cos(math.radians(decision_red[0]))),(x + decision_green[1] * math.sin(math.radians(decision_green[0]))),(y + decision_green[1] * math.cos(math.radians(decision_green[0])))
print("result:",lsp(0,0,45,90))


