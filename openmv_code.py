# Untitled - By: BIGBIGYDM - 周五 11月 5 2021

import sensor, image, time
from pyb import UART
from pyb import LED
import struct

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
#sensor.set_hmirror(True)
#sensor.set_vflip(True)
sensor.skip_frames(time = 2000)
#threshold = (0, 55, 3, 45, 2, 38)       #黑排针
#threshold = (18, 51, 15, 70, -32, 66)
threshold = (0, 70)#车1 50
#bias_offset = -41     #blue
#bias_offset = -31       #black
bias_offset = -34       #old
line_roi = (0, 100, 160, 10)
line_roi2 = (0, 50, 160, 10)
line_roi3 = (0, 20, 160, 10)
left_roi = (25, 0, 8, 120)
right_roi = (135, 0, 8, 120)
black_threshold = (25, 255)
bias = 0
count777 = 0
cross_status = 0  #0:无事发生   1：接近十字    2:经过十字
uart = UART(3, 115200)


# 通信协议
def send_data_packet(x1,y1,z1):
    data =struct.pack("<bbbbb",              #格式为四个字符俩个短整型(2字节)
                   0x2C,                       #帧头1
                   0x12,                       #帧头2
                   int(x1), # up sample by 4    #数据1,低位在前
                   int(y1), # up sample by 4    #数据2.
                   int(z1),
                   )
    uart.write(data)

#LED(1).on()
#LED(2).on()
#LED(3).on()


# 寻找一个blob list中最大的矩形，后面要用此判定是否停车
# find max blob in amount of blobs(blob list)
def find_biggest_blobs(blobs_list):
    area_sort = []
    for i in range(len(blobs_high)):
        area_sort.append([blobs_high[i], blobs_high[i].area()])
    blobs_out = sorted(area_sort, key=lambda x: x[1])
    stop_charge_blob = blobs_out[len(blobs_out)-1][0]
    return stop_charge_blob

#求绝对值
def abs_(x1,x2):
    xx = x1-x2
    if(xx>0):
        return xx
    else:
        return -xx

flag = 0#0正常巡线，1正常停车，2岔路口in，3岔路口out，4非正常停车,5转弯
xx1=0
xx2=0
while(True):
    img = sensor.snapshot()
    img_black=img.binary([threshold])
    img_black.mean(2)
    #img_white = img.binary([200,255])
    #img.dilate(1)
    #img.lens_corr()            #对old不要开启
    blobs_low = img.find_blobs([black_threshold], roi = line_roi, pixels_threshold = 150)
    blobs_high = img.find_blobs([black_threshold], roi = line_roi2, pixels_threshold = 150)
    blobs_left = img.find_blobs([black_threshold], roi = left_roi, pixels_threshold = 100)
    blobs_right = img.find_blobs([black_threshold], roi = right_roi, pixels_threshold = 100)
    blobs_high2 = img.find_blobs([black_threshold], roi = line_roi3, pixels_threshold = 150)


    # 基础题停车判定
    # 使用下面的框进行判定，是否能够找到停车标志
    if(blobs_low and blobs_high):
        #max_blob = find_biggest_blobs(blobs_low)
        if(blobs_low[0].w()>0.5*img.width() and len(blobs_high)==1 and blobs_low[0].w()<0.8*img.width()):#停车0.3m下
            print("stop")
            flag = 1
            xx1 = 80
            xx2 = 0
        elif(len(blobs_high2)==0 and blobs_high[0].w()>0.6*img.width()):#提升停车
            print("stopwww")
            flag = 4
            xx1 = 80
            xx2 = 0
        elif(len(blobs_high)==1 and len(blobs_low)==1):#正常巡线
            len_h_l=abs_(blobs_high[0].cx(),blobs_low[0].cx())
            if(len_h_l<5):
                print("line")
                flag = 0
                img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
                xx1 = blobs_high[0].cx()
                xx2 = 0
            else:
                print("line_wan")
                flag = 5
                img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
                xx1 = blobs_high[0].cx()
                xx2 = 0
        elif(len(blobs_high)==2 and len(blobs_low)==2):#一类出入三岔
            len_high = abs_(blobs_high[0].cx(),blobs_high[1].cx())
            len_low = abs_(blobs_low[0].cx(),blobs_low[1].cx())
            if(len_high>len_low):
                print("in")
                flag = 2
                img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
                img.draw_cross(blobs_high[1].cx(), blobs_high[1].cy(), (255,255,255), 30)
                xx1 = blobs_high[0].cx()
                xx2 = blobs_high[1].cx()
            else:
                print("out")
                flag = 3
                img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
                img.draw_cross(blobs_high[1].cx(), blobs_high[1].cy(), (255,255,255), 30)
                xx1 = blobs_high[0].cx()
                xx2 = blobs_high[1].cx()
        elif(len(blobs_high)==2 and len(blobs_high2)==2):#另一类出入三岔
            len_high = abs_(blobs_high[0].cx(),blobs_high[1].cx())
            len_high2 = abs_(blobs_high2[0].cx(),blobs_high2[1].cx())
            if(len_high<len_high2):
                print("in")
                flag = 2
                img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
                img.draw_cross(blobs_high[1].cx(), blobs_high[1].cy(), (255,255,255), 30)
                xx1 = blobs_high[0].cx()
                xx2 = blobs_high[1].cx()
            else:
                print("out")
                flag = 3
                img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
                img.draw_cross(blobs_high[1].cx(), blobs_high[1].cy(), (255,255,255), 30)
                xx1 = blobs_high[0].cx()
                xx2 = blobs_high[1].cx()
        elif(len(blobs_high)==2 and blobs_right ):#出弯
            print("out")
            flag = 3
            img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
            img.draw_cross(blobs_high[1].cx(), blobs_high[1].cy(), (255,255,255), 30)
            xx1 = blobs_high[0].cx()
            xx2 = blobs_high[1].cx()
        elif(len(blobs_high)==2 and blobs_left):#进弯
            print("in")
            flag = 2
            img.draw_cross(blobs_high[0].cx(), blobs_high[0].cy(), (255,255,255), 30)
            img.draw_cross(blobs_high[1].cx(), blobs_high[1].cy(), (255,255,255), 30)
            xx1 = blobs_high[0].cx()
            xx2 = blobs_high[1].cx()

        send_data_packet(xx1,xx2,flag)

    # 画出一个矩形，便于查看
    #send_data_packet(128,159,2)
    img.draw_rectangle(line_roi)
    img.draw_rectangle(line_roi2)
    img.draw_rectangle(line_roi3)
    img.draw_rectangle(left_roi)
    img.draw_rectangle(right_roi)

