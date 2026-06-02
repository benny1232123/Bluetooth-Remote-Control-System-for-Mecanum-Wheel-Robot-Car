import sensor, image, time , pyb
from pyb import UART
from pyb import Pin, ExtInt, Timer,LED
import math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  #设置像素模式 sensor.GRAYSCALE：为灰度，每个像素8BIT
sensor.set_framesize(sensor.VGA)  #设置图像大小
sensor.skip_frames(10)              #跳过一些帧
#sensor.set_brightness(3)            #设置相机图像亮度
#sensor.set_auto_gain(False, gain_db  = 26)    #enable 打开（True)或关闭（False)自动增益。默认打开。 value 强迫增益值。更多细节请参见摄像头数据表。
sensor.set_auto_gain(True)
#sensor.set_auto_exposure(False, exposure_us   = 141244) #enable 打开（True)或关闭（False)自动曝光。默认打开。如果为False， 则可以用 exposure_us 设置一个固定的曝光时间（以微秒为单位)。
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(False)    # must be turned off for color tracking#自动白平衡开启（True)或者关闭（False)。在使用颜色追踪时，需要关闭自动白平衡

#sensor.set_windowing((300, 300))       #图像中心为中心的240*240的感兴趣区
#sensor.set_windowing((150, 64, 366, 366))  #感兴趣区是矩形区域元组 (x, y, w, h).

sensor.skip_frames(100)   #跳过n张照片，在更改设置后，跳过一些帧，等待感光元件变稳定


threshold_red_cylinder  = ((7, 74, 14, 127, -30, 30))   #红色块的阈值
threshold_greed_cylinder  = (30, 70, -128, 2, -15, 15)   #绿色块的阈值
threshold_blue_cylinder  = (23, 52, -11, 20, -128, -5)   #蓝色块的阈值
threshold_white_blob  = (63, 100, -15, 15, -15, 15)   #白色块的阈值


red_cylinder_pixels_min = 1000      #红色块的最小像素点个数
greed_cylinder_pixels_min = 1000    #绿色块的最小像素点个数
blue_cylinder_pixels_min = 1000     #蓝色块的最小像素点个数
white_blob_pixels_min = 2000         #白色块的最小像素点个数


uart_receive = 65  #串口接受的数据
uart_send_str = 0   #串口发送的数据
uart_send_flag = 0    #串口发送标志位，置1则发送

#定义寻找最大色块的函数
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

#串口初始化
uart = UART(3, 115200)    #p4 TX ,p5 RX
uart.init(115200, bits=8, parity=None, stop=1) # init with given parameters
uart.write("UART_OK!")
#LED初始化
led_R = LED(1) # 红色 led
led_R.off()

led_G = LED(2) # 绿色 led
led_G.off()

led_B = LED(3) # 蓝色 led
led_B.off()




clock = time.clock()
while(True):
    clock.tick()
    if  uart.any(): #串口接收到数据
        uart_receive = uart.readchar()

    img = sensor.snapshot()#拍摄一张照片，img为一个image对象

    ################################################识别红色圆柱#########################################################
    if uart_receive == 65 : #‘A’
        #开启红色灯
        led_R.on()
        led_G.off()
        led_B.off()

        red_blobs = img.find_blobs([threshold_red_cylinder])  #寻找色块
        if red_blobs:                #找到色块
            red_cylinder = find_max( red_blobs )   #找到最大的色块
            red_cylinder_pixels = red_cylinder[4]  #记录色块像素大小
            red_cylinder_x = red_cylinder[5]       #记录色块的中心x值
            red_cylinder_y = red_cylinder[6]       #记录色块的中心x值
            if red_cylinder_pixels > red_cylinder_pixels_min  : #对比色块大小是否小于限制值
                uart_send_str= "A%dX%dY" % (red_cylinder_x,red_cylinder_y)   #赋值串口发送数据
                uart_send_flag = 1 #串口发送标志位置1，发送数据
                img.draw_rectangle(red_cylinder.rect(), color = (255,0,0), thickness=2)   #调试信息:框出查找区间
                img.draw_cross(red_cylinder_x, red_cylinder_y, color = (255,0,0), thickness=2)  #调试信息:在找到的位置中画一个十字

    ################################################识别绿色圆柱#########################################################
    elif uart_receive == 66 :#‘B’
        #开启绿色灯
        led_R.off()
        led_G.on()
        led_B.off()

        greed_blobs = img.find_blobs([threshold_greed_cylinder])  #寻找色块
        if greed_blobs:                #找到色块
            greed_cylinder = find_max( greed_blobs )   #找到最大的色块
            greed_cylinder_pixels = greed_cylinder[4]  #记录色块像素大小
            greed_cylinder_x = greed_cylinder[5]       #记录色块的中心x值
            greed_cylinder_y = greed_cylinder[6]       #记录色块的中心x值
            if greed_cylinder_pixels > greed_cylinder_pixels_min  : #对比色块大小是否小于限制值
                uart_send_str= "B%dX%dY" % (greed_cylinder_x,greed_cylinder_y)   #赋值串口发送数据
                uart_send_flag = 1 #串口发送标志位置1，发送数据
                img.draw_rectangle(greed_cylinder.rect(), color = (0,255,0), thickness=2)   #调试信息:框出查找区间
                img.draw_cross(greed_cylinder_x, greed_cylinder_y, color = (0,255,0), thickness=2)  #调试信息:在找到的位置中画一个十字

    ################################################识别蓝色圆柱#########################################################
    elif uart_receive == 67 :#‘C’
        #开启蓝色灯
        led_R.off()
        led_G.off()
        led_B.on()

        blue_blobs = img.find_blobs([threshold_blue_cylinder])  #寻找色块
        if blue_blobs:                #找到色块
            blue_cylinder = find_max( blue_blobs )   #找到最大的色块
            blue_cylinder_pixels = blue_cylinder[4]  #记录色块像素大小
            blue_cylinder_x = blue_cylinder[5]       #记录色块的中心x值
            blue_cylinder_y = blue_cylinder[6]       #记录色块的中心x值
            if blue_cylinder_pixels > blue_cylinder_pixels_min  : #对比色块大小是否小于限制值
                uart_send_str= "C%dX%dY" % (blue_cylinder_x,blue_cylinder_y)   #赋值串口发送数据
                uart_send_flag = 1 #串口发送标志位置1，发送数据
                img.draw_rectangle(blue_cylinder.rect(), color = (0,0,255), thickness=2)   #调试信息:框出查找区间
                img.draw_cross(blue_cylinder_x, blue_cylinder_y, color = (0,0,255), thickness=2)  #调试信息:在找到的位置中画一个十字

    ################################################识别红色圆圈内的白色圆圈#########################################################
    elif uart_receive == 68 :#‘D’
        #开启青色灯
        led_R.off()
        led_G.on()
        led_B.on()

        red_blobs = img.find_blobs([threshold_red_cylinder])  #寻找色块
        if red_blobs:                #找到色块
            red_circle = find_max( red_blobs )   #找到最大的色块
            red_circle_pixels = red_circle[4]  #记录色块像素大小
            red_circle_x = red_circle[5]       #记录色块的中心x值
            red_circle_y = red_circle[6]       #记录色块的中心x值
            if red_circle_pixels > red_cylinder_pixels_min  : #对比色块大小是否小于限制值
                img.draw_rectangle(red_circle.rect(), color = (255,0,0), thickness=2)   #调试信息:框出查找区间

                White_blobs = img.find_blobs([threshold_white_blob], roi = red_circle.rect()) #在上面寻找的最大色块框中寻找另一目标颜色色块
                if White_blobs:
                    max_white_blob = find_max( White_blobs )   #找到最大的色块
                    max_white_blob_pixels = max_white_blob[4]  #记录色块像素大小
                    max_white_blob_x = max_white_blob[5]       #记录色块的中心x值
                    max_white_blob_y = max_white_blob[6]       #记录色块的中心x值
                    if max_white_blob_pixels > white_blob_pixels_min  : #对比色块大小是否小于限制值
                        uart_send_str= "D%dX%dY" % (max_white_blob_x, max_white_blob_y)   #赋值串口发送数据
                        uart_send_flag = 1 #串口发送标志位置1，发送数据
                        img.draw_rectangle(max_white_blob.rect(),color = (255,255,255),thickness=2) # rect
                        img.draw_cross(max_white_blob_x, max_white_blob_y,color = (255,255,255),thickness=2) # cx, cy

    ################################################识别绿色圆圈内的白色圆圈#########################################################
    elif uart_receive == 69 :#‘E’
        #开启紫色灯
        led_R.on()
        led_G.off()
        led_B.on()

        greed_blobs = img.find_blobs([threshold_greed_cylinder])  #寻找色块
        if greed_blobs:                #找到色块
            greed_circle = find_max( greed_blobs )   #找到最大的色块
            greed_circle_pixels = greed_circle[4]  #记录色块像素大小
            greed_circle_x = greed_circle[5]       #记录色块的中心x值
            greed_circle_y = greed_circle[6]       #记录色块的中心x值
            if greed_circle_pixels > greed_cylinder_pixels_min  : #对比色块大小是否小于限制值
                img.draw_rectangle(greed_circle.rect(), color = (0,255,0), thickness=2)   #调试信息:框出查找区间

                White_blobs = img.find_blobs([threshold_white_blob], roi = greed_circle.rect()) #在上面寻找的最大色块框中寻找另一目标颜色色块
                if White_blobs:
                    max_white_blob = find_max( White_blobs )   #找到最大的色块
                    max_white_blob_pixels = max_white_blob[4]  #记录色块像素大小
                    max_white_blob_x = max_white_blob[5]       #记录色块的中心x值
                    max_white_blob_y = max_white_blob[6]       #记录色块的中心x值
                    if max_white_blob_pixels > white_blob_pixels_min  : #对比色块大小是否小于限制值
                        uart_send_str= "E%dX%dY" % (max_white_blob_x, max_white_blob_y)   #赋值串口发送数据
                        uart_send_flag = 1 #串口发送标志位置1，发送数据
                        img.draw_rectangle(max_white_blob.rect(),color = (255,255,255),thickness=2) # rect
                        img.draw_cross(max_white_blob_x, max_white_blob_y,color = (255,255,255),thickness=2) # cx, cy

    ################################################识别蓝色圆圈内的白色圆圈#########################################################
    elif uart_receive == 70 :#‘F’
        #开启碧绿色灯
        led_R.on()
        led_G.on()
        led_B.off()

        blue_blobs = img.find_blobs([threshold_blue_cylinder])  #寻找色块
        if blue_blobs:                #找到色块
            blue_circle = find_max( blue_blobs )   #找到最大的色块
            blue_circle_pixels = blue_circle[4]  #记录色块像素大小
            blue_circle_x = blue_circle[5]       #记录色块的中心x值
            blue_circle_y = blue_circle[6]       #记录色块的中心x值
            if blue_circle_pixels > blue_cylinder_pixels_min  : #对比色块大小是否小于限制值
                img.draw_rectangle(blue_circle.rect(), color = (0,0,255), thickness=2)   #调试信息:框出查找区间

                White_blobs = img.find_blobs([threshold_white_blob], roi = blue_circle.rect()) #在上面寻找的最大色块框中寻找另一目标颜色色块
                if White_blobs:
                    max_white_blob = find_max( White_blobs )   #找到最大的色块
                    max_white_blob_pixels = max_white_blob[4]  #记录色块像素大小
                    max_white_blob_x = max_white_blob[5]       #记录色块的中心x值
                    max_white_blob_y = max_white_blob[6]       #记录色块的中心x值
                    if max_white_blob_pixels > white_blob_pixels_min  : #对比色块大小是否小于限制值
                        uart_send_str= "F%dX%dY" % (max_white_blob_x, max_white_blob_y)   #赋值串口发送数据
                        uart_send_flag = 1 #串口发送标志位置1，发送数据
                        img.draw_rectangle(max_white_blob.rect(),color = (255,255,255),thickness=2) # rect
                        img.draw_cross(max_white_blob_x, max_white_blob_y,color = (255,255,255),thickness=2) # cx, cy



    ################################################未作任何处理亮白灯#########################################################
    else :
        #开启白色灯
        led_R.on()
        led_G.on()
        led_B.on()

    if uart_send_flag == 1: #串口发送标志位置1，发送数据
        uart.write(uart_send_str)                    #将偏差以字符串形式发送
        uart_send_flag = 0 #串口发送标志位置0，完成数据发送
        print("串口发送: ",uart_send_str)
    elif uart_receive >= 65 and uart_receive <= 70: #表示色块寻找时候没有找到色块的情况
        uart_send_str= 'Q'   #赋值串口发送数据
        uart.write(uart_send_str)                    #发送Q表示没有找到色块
        print("串口发送: ",uart_send_str)
    print("每秒帧率: ",clock.fps(),"曝光时间: ",sensor.get_exposure_us(), "增益: ", sensor.get_gain_db(),"uart_receive: ",uart_receive )
