from ctypes import sizeof #gọi các hàm từ các thư viện khác không thuộc về python
from tkinter import * # thư viện GUI tiêu chuẩn cho Python
import tkinter
from tkinter import font
from typing import Mapping, Sized #thư viện mảng kich thuoc
import numpy as np #thư viện toán học

from serial.serialutil import Timeout #thu vien serial, timeout: limit the max time for calling a function
import KinematicRobot #thu vien dong hoc
import serial # cong serial
import time # thoi gian

import threading #luồng thực thi riêng biệt


def action(): #ham chuyen doi 2 giao dien

    top.destroy() # tat man hinh dau
    GiaoDien.deiconify() #hien thi man hinh giao dien

GiaoDien = Tk() #tao man hinh giao dien
top=Toplevel()  # màn hình hiện thị đầu tiên
top.title("GIAO DIỆN ROBOT 3 DOF") #tên
top.geometry('1230x605') #kích  thước 
#MC = PhotoImage(file="D:\DOWNLOAD\giaodien1.png") #hinh C:\\Users\\84382\\Pictures\\
#Label(top,image=MC).place(x=0,y=0)  # vi tri
#MC=PhotoImage(file="giaodien1.png") # hình
#Label(top,image=MC).grid() # vị trí
Button(top,text="GO CONTROL INTERFACE",font='Times 15 bold', bg='#6666ff',fg='white',command=lambda:action()).place(x=400,y=500) # nút nhấn fg mau vien bg mau nen

GiaoDien.withdraw() # de chi hien thi 1 giao dien TOP ban dau
ser = serial.Serial('COM3',9600)

x_axis = 15
y_axis = 30
y_lable_IK = 250
value1 = 0
value2 = 0
value3 = 0
value4 = 0
x=0
L1 = 107
L2 = 162
L3 = 130
L4 = 0

background_color = '#C8BBBE' # mã TLP RGB xem ở https://htlit.maytinhhtl.com/lam-web/bang-ma-mau-css-html-code-thet-ke-design.html

#GiaoDien.geometry('570x700')
GiaoDien.geometry('1300x700')
GiaoDien.title("CONTROL INTERFACE")
GiaoDien.configure(bg=background_color)

#------------------------------- VIẾT HÀM CON ------------------------------------
def slider_theta1_value(x):
    value1 = slider_theta1.get()
    txb_slider_theta1.delete(0,END) # Xóa dữ liệu trước đó
    txb_slider_theta1.insert(0,value1) # hiển thị dữ liệu mới
    return value1

def slider_theta2_value(x):
    txb_slider_theta2.delete(0,END)
    value2 = slider_theta2.get()
    txb_slider_theta2.insert(0,value2)
    return value2

def slider_theta3_value(x):
    txb_slider_theta3.delete(0,END)
    value3 = slider_theta3.get()
    txb_slider_theta3.insert(0,value3)  
    return value3

def slider_theta4_value(x):
    txb_slider_theta4.delete(0,END)
    value4 = slider_theta4.get()
    txb_slider_theta4.insert(0,value4)  
    return value4

def FK(x):
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    value1 = slider_theta1_value(x)
    value2 = slider_theta2_value(x)
    value3 = slider_theta3_value(x)
    value4 = slider_theta4_value(x)
    Px = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[0]
    Py = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[1]
    Pz = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới vao PX PY PZ
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    #mang = str(value1)+'A'+str(value2)+'B'+str(value3)+'C'
    #ser.write(mang.encode())
    #time.sleep(0.01)

def IK():
    txb_theta1_IK.delete(0,END)
    txb_theta2_IK.delete(0,END)
    txb_theta3_IK.delete(0,END)
    #txb_theta4_IK.delete(0,END)
    Px = float(txb_Px_IK.get())
    Py = float(txb_Py_IK.get())
    Pz = float(txb_Pz_IK.get())
    theta = float(txb_Theta.get())
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK.insert(0,theta1)
    txb_theta2_IK.insert(0,theta2)
    txb_theta3_IK.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    #txb_Px_IK.delete(0,END)
    #txb_Py_IK.delete(0,END)
    #txb_Pz_IK.delete(0,END)
    #txb_Theta.delete(0,END)

######### hang test
def RO():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 110.208
    Py = -216.906
    Pz = 394.0
    theta = 0
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         RỔ'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    

    Px = 148.857
    Py = -262.965
    Pz = 148.358
    theta = -75
 
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         RỔ'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    

def vitri_reset():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         HOME'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 399.0  
    Py = -55.0
    Pz = 232.0
    theta = 0
 
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '      HOME'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.     
    
def IK_VT1():
     
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 

    Px = 331.152
    Py = 52.805
    Pz = 80.007
    theta = -59
 
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


#########
    
def IK_VT2():   
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 332.238
    Py = 11.468
    Pz = 78.292
    theta = -60.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 
 
def IK_VT3():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 331.808
    Py = -20.429
    Pz = 78.292
    theta = -60.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

    

def IK_VT4():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 324.861
    Py = -60.679
    Pz = 77.192
    theta = -61.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 



def IK_VT5():    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 313.749
    Py = -93.937
    Pz =75.608
    theta = -63
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D1'
    txb_vitri_BC.insert(0,vitri)
    
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 



    
def IK_VT6():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px =299.963
    Py = -131.473
    Pz = 75.608
    theta = -63
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


        

def IK_VT7():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 287.407
    Py = -163.137
    Pz = 77.192
    theta = -61.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)

    txb_vitri_BC.delete(0,END)
    vitri = '         B1'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 



def IK_VT8():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A1'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 279.181
    Py = -190.87
    Pz = 81.798
    theta = -57.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A1'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT9():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 316.463
    Py = 56.653
    Pz = 86.553
    theta = -67.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


def IK_VT10():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 314.699
    Py = 18.99
    Pz = 92.149
    theta = -71
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
def IK_VT11():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 305.855
    Py = -23.156
    Pz = 89.496
    theta = -75
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


def IK_VT12():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 301.268
    Py = -57.631
    Pz = 89.496
    theta = -75
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.       

def IK_VT13():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 295.17
    Py = -97.024
    Pz = 87.938
    theta = -73
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.     

def IK_VT14():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 279.981
    Py = -134.721
    Pz = 87.938
    theta = -73
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.     

def IK_VT15():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 271.615
    Py = -166.105
    Pz = 85.292
    theta = -69
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 
   
def IK_VT16():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A2'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 261.521
    Py = -198.145
    Pz = 81.977
    theta = -63.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A2'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT17():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 294.187
    Py = 56.77
    Pz = 93.309
    theta = -78.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


def IK_VT18():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 288.541
    Py = 15.258
    Pz = 98.595
    theta = -83.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT19():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 285.822
    Py = -22.791
    Pz = 98.358
    theta = -84.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.   

def IK_VT20():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 273.875
    Py = -62.191
    Pz = 90.878
    theta = -87
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.  

def IK_VT21():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 264.788
    Py = -100.075
    Pz = 91.016
    theta = -86
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT22():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 247.087
    Py = -138.114
    Pz = 91.016
    theta = -86
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.     


def IK_VT23():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 234.554
    Py = -170.06
    Pz = 91.668
    theta = -83
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


def IK_VT24():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A3'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 217.908
    Py = -204.036
    Pz = 93.088
    theta = -79
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A3'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.    

def IK_VT25():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 272.607
    Py = 63.792
    Pz = 95.057
    theta = -87.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT26():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 268.443
    Py = 14.989
    Pz = 95.057
    theta = -92.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT27():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 260.73
    Py = -27.899
    Pz = 95.532
    theta = -95.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.     

def IK_VT28():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 249.239
    Py = -65.934
    Pz = 96.046
    theta = -97.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.   

def IK_VT29():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 234.938
    Py = -106.166
    Pz = 96.146
    theta = -97.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.   

def IK_VT30():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 216.665        
    Py = -139.726
    Pz = 96.046
    theta = -97.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 


def IK_VT31():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px =207.912
    Py = -170.238
    Pz = 92.234
    theta = -92.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT32():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A4'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 181.61
    Py = -205.472
    Pz = 94.934
    theta = -90
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A4'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT33():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 245.302
    Py = 65.69
    Pz = 88.074
    theta = -99
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT34():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 242.383
    Py = 18.754
    Pz = 90.335
    theta = -104
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

    
def IK_VT35():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 234.381
    Py = -24.618
    Pz = 92.49
    theta = -107.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT36():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 222.742
    Py = -66.749
    Pz = 93.556
    theta = -109
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT37():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 206.848
    Py = -106.223
    Pz = 93.556
    theta = -109
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT38():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 190.0
    Py = -140.4
    Pz = 100.95
    theta = -107.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT39():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 170.666
    Py = -173.939
    Pz = 98.795
    theta = -104
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT40():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A5'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 145.384
    Py = -204.914
    Pz = 97.111
    theta = -100.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A5'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT41():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 224.795
    Py = 75.613
    Pz = 78.905
    theta = -106
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT42():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 222.043
    Py = 22.287
    Pz = 80.985
    theta = -112.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT43():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 214.656
    Py = -25.373
    Pz = 84.246
    theta = -116
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT44():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 197.432
    Py = -72.483
    Pz = 87.388
    theta = -119
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT45():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 179.995
    Py = -112.531
    Pz = 86.306
    theta = -118
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT46():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 147.272
    Py = -148.536
    Pz = 77.192
    theta = -118.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT47():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 130.274
    Py = -181.11
    Pz = 86.992
    theta = -113
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT48():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A6'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 107.823
    Py = -207.87
    Pz = 88.62
    theta = -108
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A6'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.


def IK_VT49():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 196.814
    Py = 79.442
    Pz = 86.306
    theta = -118
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT50():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 199.847
    Py = 28.772
    Pz = 92.684
    theta = -123.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT51():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 185.392
    Py = -27.904
    Pz = 84.325
    theta = -129.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT52():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 166.416
    Py = -78.929
    Pz = 87.272
    theta = -131.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT53():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 140.317
    Py = -119.311
    Pz = 87.272
    theta = -131.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT54():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 118.01
    Py = -147.843
    Pz = 82.897
    theta = -128.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT55():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 87.252
    Py = -181.625
    Pz = 96.094
    theta = -124
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT56():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 313.475
    Py = -55.0
    Pz = 254.627
    theta = -45
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A7'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    Px = 62.091
    Py = -203.156
    Pz = 93.809
    theta = -118.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A7'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT64():


    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 21.76
    Py = -210.161
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 17.515
    Py = -199.1
    Pz = 87.783
    theta = -124
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         A8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.    

def IK_VT63():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 47.21
    Py =-205.942
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 34.362
    Py = -181.779
    Pz = 86.524
    theta = -131
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         B8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.    


def IK_VT62():

    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 78.845
    Py = -196.021
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    
    Px = 45.856
    Py = -152.243
    Pz =65.712
    theta = -139
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         C8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.

def IK_VT61():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 131.272
    Py = -165.556
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 76.615
    Py =-125.112
    Pz = 62.915
    theta = -143.5
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         D8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def IK_VT60():

    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 186.399
    Py = -99.48
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    
    Px = 124.452
    Py = -85.179
    Pz = 63.732
    theta = -142
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         E8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.  


def IK_VT59():

    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 210.607
    Py = -16.903
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    
    Px = 156.709
    Py = -26.892
    Pz = 65.712
    theta = -139
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         F8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.


def IK_VT58():
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 204.169
    Py = 54.369
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    
    Px = 178.139
    Py = 39.34
    Pz = 78.48
    theta = -131
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         G8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.         

def IK_VT57():
    
    txb_theta1_IK_BC.delete(0,END)
    txb_theta2_IK_BC.delete(0,END)
    txb_theta3_IK_BC.delete(0,END)
    txb_Px_IK_BC.delete(0,END)
    txb_Py_IK_BC.delete(0,END)
    txb_Pz_IK_BC.delete(0,END)
    txb_Theta_BC.delete(0,END)
    #txb_vitri_BC.delete(0,END)
    #txb_theta4_IK.delete(0,END)

    Px = 185.836
    Py = 100.529
    Pz = 119.417
    theta = -120
     
    #txb_Px_IK_BC.insert(0,Px)
    #txb_Py_IK_BC.insert(0,Py)
    #txb_Pz_IK_BC.insert(0,Pz)
    #txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H8'
    txb_vitri_BC.insert(0,vitri)

    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #txb_theta1_IK_BC.insert(0,theta1)
    #txb_theta2_IK_BC.insert(0,theta2)
    #txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(2)  #để thêm độ trễ trong quá trình thực thi chương trình. 
    
    Px = 175.119
    Py = 90.359
    Pz = 76.861
    theta = -124
    txb_Px_IK_BC.insert(0,Px)
    txb_Py_IK_BC.insert(0,Py)
    txb_Pz_IK_BC.insert(0,Pz)
    txb_Theta_BC.insert(0,theta)
    txb_vitri_BC.delete(0,END)
    vitri = '         H8'
    txb_vitri_BC.insert(0,vitri)
    theta1 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[0]
    theta2 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[1]
    theta3 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    #theta4 = KinematicRobot.Inverse_Kinematic(Px,Py,Pz,L1,L2,L3,theta)[2]
    txb_theta1_IK_BC.insert(0,theta1)
    txb_theta2_IK_BC.insert(0,theta2)
    txb_theta3_IK_BC.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình.        
























def Reset_Slider():
    txb_slider_theta1.delete(0,END)
    txb_slider_theta2.delete(0,END)
    txb_slider_theta3.delete(0,END)
   # txb_slider_theta4.delete(0,END)  
    txb_slider_theta1.insert(0,value1)
    txb_slider_theta2.insert(0,value2)
    txb_slider_theta3.insert(0,value3)      
    slider_theta1.set('0') #đặt lại vị trí thanh slider tương ứng với giá trị lấy từ textbox
    slider_theta2.set('0')
    slider_theta3.set('0')
    

def Reset_lable_Slider():
    Reset_Slider()
    Px = KinematicRobot.Forward_Kinematic(0,0,0,L1,L2,L3)[0]
    Py = KinematicRobot.Forward_Kinematic(0,0,0,L1,L2,L3)[1]
    Pz = KinematicRobot.Forward_Kinematic(0,0,0,L1,L2,L3)[2] 

    txb_Px_FK.delete(0,END) # hiển thị dữ liệu mới
    txb_Py_FK.delete(0,END) # hiển thị dữ liệu mới
    txb_Pz_FK.delete(0,END) # hiển thị dữ liệu mới 
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới    

def ReSet_btn():
    new_thread = threading.Thread(target=Reset_lable_Slider) # Thread là các hàm hay thủ tục chạy độc lập đối với chương trình chính
    new_thread.start()
    mang = f'F0A0B0C'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01) 
    


def theta1_set_btn():
    slider_theta1.set(txb_slider_theta1.get())
    value1 = slider_theta1_value(x) # lấy giá trị hiện tại của thanh slider 
    value2 = slider_theta2_value(x)
    value3 = slider_theta3_value(x)
    #value4 = slider_theta4_value(x)
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    Px = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[0]
    Py = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[1]
    Pz = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    mang = f'F{value1}A{value2}B{value3}C'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)


def theta2_set_btn():
    slider_theta2.set(txb_slider_theta2.get())
    value1 = slider_theta1_value(x) # lấy giá trị hiện tại của thanh slider 
    value2 = slider_theta2_value(x)
    value3 = slider_theta3_value(x)
    #value4 = slider_theta4_value(x)
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    Px = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[0]
    Py = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[1]
    Pz = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    mang = f'F{value1}A{value2}B{value3}C'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)

def theta3_set_btn():
    slider_theta3.set(txb_slider_theta3.get())
    value1 = slider_theta1_value(x) # lấy giá trị hiện tại của thanh slider 
    value2 = slider_theta2_value(x)
    value3 = slider_theta3_value(x)
    #value4 = slider_theta4_value(x)
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    Px = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[0]
    Py = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[1]
    Pz = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    mang = f'F{value1}A{value2}B{value3}C'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)

def theta4_set_btn():
    slider_theta4.set(txb_slider_theta4.get())
    value1 = slider_theta1_value(x) # lấy giá trị hiện tại của thanh slider 
    value2 = slider_theta2_value(x)
    value3 = slider_theta3_value(x)
    value4 = slider_theta4_value(x)
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    Px = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[0]
    Py = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[1]
    Pz = KinematicRobot.Forward_Kinematic(value1,value2,value3,L1,L2,L3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    mang = f'F{value1}A{value2}B{value3}C'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)
    

def Start_btn():
    mang = 'S'
    ser.write(mang.encode())
    time.sleep(0.01)

def Stop_btn():
    mang = 'T'
    ser.write(mang.encode())
    time.sleep(0.01)

def hut_btn():
    mang = 'H'
    ser.write(mang.encode())
    time.sleep(0.01)
def tha_btn():
    mang = 'L'
    ser.write(mang.encode())
    time.sleep(0.01)

def reset_vt():
    mang = 'R'
    ser.write(mang.encode())
    time.sleep(0.01)

#--------------------------------------------------------------------------

#lbl_tieude = Label(GiaoDien,text="   CONTROL INTERFACE",font=("Arial",17,font.BOLD),bg=background_color)
lbl_banco = Label(GiaoDien,text="ĐIỀU KHIỂN VỊ TRÍ VẬT TRÊN BÀN CỜ",fg="blue",font=("Arial",14,font.BOLD),bg=background_color)
lbl_FK = Label(GiaoDien,text="ĐỘNG HỌC THUẬN",fg="blue",font=("Arial",14,font.BOLD),bg=background_color)
lbl_IK = Label(GiaoDien,text="ĐỘNG HỌC NGHỊCH",fg="blue",font=("Arial",14,font.BOLD),bg=background_color)
lbl_theta1_FK = Label(GiaoDien,text="Theta1",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta2_FK = Label(GiaoDien,text="Theta2",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta3_FK = Label(GiaoDien,text="Theta3",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta4_FK = Label(GiaoDien,text="Theta4",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta1_IK = Label(GiaoDien,text="Theta1",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta2_IK = Label(GiaoDien,text="Theta2",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta3_IK = Label(GiaoDien,text="Theta3",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta4_IK = Label(GiaoDien,text="Theta4",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Px_IK = Label(GiaoDien,text="Px",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Py_IK = Label(GiaoDien,text="Py",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Pz_IK = Label(GiaoDien,text="Pz",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Px_FK = Label(GiaoDien,text="Px",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Py_FK = Label(GiaoDien,text="Py",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Pz_FK = Label(GiaoDien,text="Pz",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Theta = Label(GiaoDien,text="Theta",fg="black",font=("Arial",12,font.BOLD),bg=background_color)

#BÀN CỜ
lbl_theta1_IK_BC = Label(GiaoDien,text="Theta1",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta2_IK_BC = Label(GiaoDien,text="Theta2",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta3_IK_BC = Label(GiaoDien,text="Theta3",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta4_IK_BC = Label(GiaoDien,text="Theta4",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Px_IK_BC = Label(GiaoDien,text="Px",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Py_IK_BC = Label(GiaoDien,text="Py",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Pz_IK_BC = Label(GiaoDien,text="Pz",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Theta_BC = Label(GiaoDien,text="Theta",fg="black",font=("Arial",12,font.BOLD),bg=background_color)

lbl_1 = Label(GiaoDien,text="1",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_2 = Label(GiaoDien,text="2",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_3 = Label(GiaoDien,text="3",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_4 = Label(GiaoDien,text="4",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_5 = Label(GiaoDien,text="5",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_6 = Label(GiaoDien,text="6",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_7 = Label(GiaoDien,text="7",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_8 = Label(GiaoDien,text="8",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_A = Label(GiaoDien,text="A",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_B = Label(GiaoDien,text="B",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_C = Label(GiaoDien,text="C",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_D = Label(GiaoDien,text="D",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_E = Label(GiaoDien,text="E",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_F = Label(GiaoDien,text="F",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_G = Label(GiaoDien,text="G",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_H = Label(GiaoDien,text="H",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_VITRIVAT = Label(GiaoDien,text="VỊ TRÍ VẬT",fg="black",font=("Arial",12,font.BOLD),bg=background_color)




txb_slider_theta1 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD)) #tao o hien thi goc ben canh slider
txb_slider_theta1.insert(0,value1)

txb_slider_theta2 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta2.insert(0,value2)

txb_slider_theta3 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta3.insert(0,value3)

txb_slider_theta4 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta4.insert(0,value4)

txb_Px_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) # tao o hien thi Px Invert
txb_Py_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_Pz_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_Px_FK = Entry(GiaoDien,width=7,font=("Arial",12,font.BOLD))
txb_Py_FK = Entry(GiaoDien,width=7,font=("Arial",12,font.BOLD))
txb_Pz_FK = Entry(GiaoDien,width=7,font=("Arial",12,font.BOLD))
txb_Theta = Entry(GiaoDien,width=10,font=("Aria-l",12,font.BOLD))
txb_theta1_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) #tao o hien thi goc invert
txb_theta2_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta4_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))

#BÀN CỜ
txb_Px_IK_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) # tao o hien thi Px Invert
txb_Py_IK_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_Pz_IK_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_Theta_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta1_IK_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) #tao o hien thi goc invert
txb_theta2_IK_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_IK_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_vitri_BC = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))



slider_theta1 = Scale(GiaoDien,from_=-90, to_= 90,orient=HORIZONTAL,width=15,resolution=0.5,length=350,command=FK) # gioi han thanh slider
slider_theta1.set(value1)
slider_theta2 = Scale(GiaoDien,from_=-90, to_= 150,orient=HORIZONTAL,width=15,resolution=0.5,length=350,command=FK)
slider_theta2.set(value2)
slider_theta3 = Scale(GiaoDien,from_=-150, to_= 90,orient=HORIZONTAL,width=15,resolution=0.5,length=350,command=FK)
slider_theta3.set(value3)
slider_theta4 = Scale(GiaoDien,from_=-150, to_= 150,orient=HORIZONTAL,width=15,resolution=0.5,length=350,command=FK)
slider_theta4.set(value4)


btn_Start = Button(GiaoDien,text="Set Home",font=("Arial",12,font.BOLD),width=10,height=2,bg='#00FF33',command=Start_btn)
btn_Stop = Button(GiaoDien,text="Stop",font=("Arial",12,font.BOLD),width=10,height=2,bg='#FF4500',command=Stop_btn)
btn_Solve = Button(GiaoDien,text="Solve",font=("Arial",12,font.BOLD),width=10,height=2,bg='#CDC9A5',command=IK)
btn_ReSet = Button(GiaoDien,text="Reset",font=("Arial",12,font.BOLD),width=10,height=2,bg='#CD0000',command=ReSet_btn)
btn_Set_Theta1 = Button(GiaoDien,text="Set Theta1",font=("Arial",10,font.BOLD),width=8,height=2,bg='#BEBEBE',command=theta1_set_btn)
btn_Set_Theta2 = Button(GiaoDien,text="Set Theta2",font=("Arial",10,font.BOLD),width=8,height=2,bg='#BEBEBE',command=theta2_set_btn)
btn_Set_Theta3 = Button(GiaoDien,text="Set Theta3",font=("Arial",10,font.BOLD),width=8,height=2,bg='#BEBEBE',command=theta3_set_btn)
btn_Set_Theta4 = Button(GiaoDien,text="Set Theta4",font=("Arial",10,font.BOLD),width=8,height=2,bg='#BEBEBE',command=theta4_set_btn)

btn_hut = Button(GiaoDien,text="GẮP",font=("Arial",12,font.BOLD),width=10,height=2,bg='#00FFFF',command=hut_btn)
btn_tha = Button(GiaoDien,text="THẢ",font=("Arial",12,font.BOLD),width=10,height=2,bg='#FF1493',command=tha_btn)

btn_vitri1 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT1)
btn_vitri2 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT2)
btn_vitri3 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT3)
btn_vitri4 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT4)
btn_vitri5 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT5)
btn_vitri6 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT6)
btn_vitri7 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT7)
btn_vitri8 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT8)
btn_vitri9 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT9)
btn_vitri10 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT10)
btn_vitri11 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT11)
btn_vitri12 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT12)
btn_vitri13 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT13)
btn_vitri14 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT14)
btn_vitri15 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT15)
btn_vitri16 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT16)
btn_vitri17 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT17)
btn_vitri18 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT18)
btn_vitri19 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT19)
btn_vitri20 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT20)
btn_vitri21=  Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT21)
btn_vitri22= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT22)
btn_vitri23= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT23)
btn_vitri24= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT24)
btn_vitri25= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT25)
btn_vitri26= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT26)
btn_vitri27= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT27)
btn_vitri28= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT28)
btn_vitri29= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT29)
btn_vitri30= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT30)
btn_vitri31= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT31)
btn_vitri32= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT32)
btn_vitri33= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT33)
btn_vitri34= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT34)
btn_vitri35= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT35)
btn_vitri36= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT36)
btn_vitri37= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT37)
btn_vitri38= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT38)
btn_vitri39 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT39)
btn_vitri40 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT40)
btn_vitri41 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT41)
btn_vitri42 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT42)
btn_vitri43 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT43)
btn_vitri44 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT44)
btn_vitri45 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT45)
btn_vitri46 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT46)
btn_vitri47 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT47)
btn_vitri48 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT48)
btn_vitri49 = Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT49)
btn_vitri50= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT50)
btn_vitri51= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT51)
btn_vitri52= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT52)
btn_vitri53= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT53)
btn_vitri54= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT54)
btn_vitri55= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT55)
btn_vitri56= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT56)
btn_vitri57= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT57)
btn_vitri58= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT58)
btn_vitri59= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT59)
btn_vitri60= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT60)
btn_vitri61= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT61)
btn_vitri62= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT62)
btn_vitri63= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#000000',command=IK_VT63)
btn_vitri64= Button(GiaoDien,text=" ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#DCDCDC',command=IK_VT64)
btn_vitri65= Button(GiaoDien,text="RỔ",font=("Arial",12,font.BOLD),width=5,height=2,bg='#5CB3FF',command=RO)
btn_vitri_reset=Button(GiaoDien,text="Reset",font=("Arial",12,font.BOLD),width=10,height=2,bg='#5CB3FF',command=vitri_reset)

#btn_vitri2 = Button(GiaoDien,text="RESET_IK",font=("Arial",12,font.BOLD),width=10,height=2,bg='#00FFFF',command=IK_VT2)
######

################
#lbl_tieude.place(x=150,y=0)
lbl_banco.place(x=x_axis+ 650,y=y_axis)
lbl_FK.place(x=x_axis,y=y_axis)
lbl_IK.place(x=x_axis,y=y_axis+y_lable_IK+50)
lbl_theta1_FK.place(x=x_axis,y=y_axis+40)
lbl_theta2_FK.place(x=15,y=y_axis+85)
lbl_theta3_FK.place(x=15,y=y_axis+130)
lbl_theta4_FK.place(x=15,y=y_axis+175)
lbl_theta1_IK.place(x=200,y=y_axis+y_lable_IK+100)
lbl_theta2_IK.place(x=200,y=y_axis+y_lable_IK+150)
lbl_theta3_IK.place(x=200,y=y_axis+y_lable_IK+200)
lbl_theta4_IK.place(x=200,y=y_axis+y_lable_IK+250)
lbl_Px_FK.place(x=x_axis+40,y=y_axis+230)
lbl_Py_FK.place(x=x_axis+140,y=y_axis+230)
lbl_Pz_FK.place(x=x_axis+240,y=y_axis+230)
lbl_Px_IK.place(x=x_axis+20,y=y_axis+y_lable_IK+100)
lbl_Py_IK.place(x=x_axis+20,y=y_axis+y_lable_IK+150)
lbl_Pz_IK.place(x=x_axis+20,y=y_axis+y_lable_IK+200)
lbl_Theta.place(x=x_axis,y=y_axis+y_lable_IK+250)

#BÀN CỜ
lbl_theta1_IK_BC.place(x=740,y=y_axis+y_lable_IK+280)
lbl_theta2_IK_BC.place(x=840,y=y_axis+y_lable_IK+280)
lbl_theta3_IK_BC.place(x=940,y=y_axis+y_lable_IK+280)
lbl_theta4_IK_BC.place(x=200,y=y_axis+y_lable_IK+250)
lbl_Px_IK_BC.place(x=800,y=y_axis+y_lable_IK+230)
lbl_Py_IK_BC.place(x=900,y=y_axis+y_lable_IK+230)
lbl_Pz_IK_BC.place(x=1000,y=y_axis+y_lable_IK+230)
lbl_Theta_BC.place(x=1040,y=y_axis+y_lable_IK+280)

lbl_1.place(x=640,y=70)
lbl_2.place(x=640,y=122)
lbl_3.place(x=640,y=174)
lbl_4.place(x=640,y=226)
lbl_5.place(x=640,y=278)
lbl_6.place(x=640,y=330)
lbl_7.place(x=640,y=382)
lbl_8.place(x=640,y=434)
lbl_A.place(x=1106,y=472)
lbl_B.place(x=1046,y=472)
lbl_C.place(x=986,y=472)
lbl_D.place(x=926,y=472)
lbl_E.place(x=866,y=472)
lbl_F.place(x=806,y=472)
lbl_G.place(x=746,y=472)
lbl_H.place(x=686,y=472)

lbl_VITRIVAT.place(x=1170,y=85)




        
slider_theta1.place(x=x_axis+60,y=y_axis+30)
slider_theta2.place(x=x_axis+60,y=y_axis+75)
slider_theta3.place(x=x_axis+60,y=y_axis+120)
slider_theta4.place(x=x_axis+60,y=y_axis+165)

txb_slider_theta1.place(x=x_axis+420,y=y_axis+40)
txb_slider_theta2.place(x=x_axis+420,y=y_axis+80)
txb_slider_theta3.place(x=x_axis+420,y=y_axis+130)
txb_slider_theta4.place(x=x_axis+420,y=y_axis+170)
txb_Px_FK.place(x=x_axis+20,y=y_axis+250)
txb_Py_FK.place(x=x_axis+120,y=y_axis+250)
txb_Pz_FK.place(x=x_axis+220,y=y_axis+250)
txb_Px_IK.place(x=x_axis+50,y=y_axis+y_lable_IK+100)
txb_Py_IK.place(x=x_axis+50,y=y_axis+y_lable_IK+150)
txb_Pz_IK.place(x=x_axis+50,y=y_axis+y_lable_IK+200)
txb_Theta.place(x=x_axis+50,y=y_axis+y_lable_IK+250)
txb_theta1_IK.place(x=270,y=y_axis+y_lable_IK+100)
txb_theta2_IK.place(x=270,y=y_axis+y_lable_IK+150)
txb_theta3_IK.place(x=270,y=y_axis+y_lable_IK+200)
txb_theta4_IK.place(x=270,y=y_axis+y_lable_IK+250)

#BÀN CỜ
txb_Px_IK_BC.place(x=x_axis+750,y=y_axis+y_lable_IK+250)
txb_Py_IK_BC.place(x=x_axis+850,y=y_axis+y_lable_IK+250)
txb_Pz_IK_BC.place(x=x_axis+950,y=y_axis+y_lable_IK+250)
txb_Theta_BC.place(x=1020,y=y_axis+y_lable_IK+300)
txb_theta1_IK_BC.place(x=720,y=y_axis+y_lable_IK+300)
txb_theta2_IK_BC.place(x=820,y=y_axis+y_lable_IK+300)
txb_theta3_IK_BC.place(x=920,y=y_axis+y_lable_IK+300)
#txb_theta4_IK_BC.place(x=270,y=y_axis+y_lable_IK+250)
txb_vitri_BC.place(x=x_axis+1150,y=y_axis+78)

btn_Start.place(x=150,y=y_axis+y_lable_IK+350)
btn_Stop.place(x=275,y=y_axis+y_lable_IK+350)
btn_Solve.place(x=x_axis+370,y=y_axis+y_lable_IK+160)
btn_ReSet.place(x=x_axis+370,y=y_axis+230)
btn_Set_Theta1.place(x=x_axis+480,y=y_axis+26)
btn_Set_Theta2.place(x=x_axis+480,y=y_axis+74)
btn_Set_Theta3.place(x=x_axis+480,y=y_axis+120)
btn_Set_Theta4.place(x=x_axis+480,y=y_axis+166)

btn_hut.place(x=800,y=y_axis+y_lable_IK+350)
btn_tha.place(x=925,y=y_axis+y_lable_IK+350)

btn_vitri1.place(x=x_axis+650,y=y_axis+26)
btn_vitri2.place(x=x_axis+710,y=y_axis+26)
btn_vitri3.place(x=x_axis+770,y=y_axis+26)
btn_vitri4.place(x=x_axis+830,y=y_axis+26)
btn_vitri5.place(x=x_axis+890,y=y_axis+26)
btn_vitri6.place(x=x_axis+950,y=y_axis+26)
btn_vitri7.place(x=x_axis+1010,y=y_axis+26)
btn_vitri8.place(x=x_axis+1070,y=y_axis+26)
btn_vitri9.place(x=x_axis+650,y=y_axis+78)
btn_vitri10.place(x=x_axis+710,y=y_axis+78)
btn_vitri11.place(x=x_axis+770,y=y_axis+78)
btn_vitri12.place(x=x_axis+830,y=y_axis+78)
btn_vitri13.place(x=x_axis+890,y=y_axis+78)
btn_vitri14.place(x=x_axis+950,y=y_axis+78)
btn_vitri15.place(x=x_axis+1010,y=y_axis+78)
btn_vitri16.place(x=x_axis+1070,y=y_axis+78)
btn_vitri17.place(x=x_axis+650,y=y_axis+130)
btn_vitri18.place(x=x_axis+710,y=y_axis+130)
btn_vitri19.place(x=x_axis+770,y=y_axis+130)
btn_vitri20.place(x=x_axis+830,y=y_axis+130)
btn_vitri21.place(x=x_axis+890,y=y_axis+130)
btn_vitri22.place(x=x_axis+950,y=y_axis+130)
btn_vitri23.place(x=x_axis+1010,y=y_axis+130)
btn_vitri24.place(x=x_axis+1070,y=y_axis+130)
btn_vitri25.place(x=x_axis+650,y=y_axis+182)
btn_vitri26.place(x=x_axis+710,y=y_axis+182)
btn_vitri27.place(x=x_axis+770,y=y_axis+182)
btn_vitri28.place(x=x_axis+830,y=y_axis+182)
btn_vitri29.place(x=x_axis+890,y=y_axis+182)
btn_vitri30.place(x=x_axis+950,y=y_axis+182)
btn_vitri31.place(x=x_axis+1010,y=y_axis+182)
btn_vitri32.place(x=x_axis+1070,y=y_axis+182)
btn_vitri33.place(x=x_axis+650,y=y_axis+234)
btn_vitri34.place(x=x_axis+710,y=y_axis+234)
btn_vitri35.place(x=x_axis+770,y=y_axis+234)
btn_vitri36.place(x=x_axis+830,y=y_axis+234)
btn_vitri37.place(x=x_axis+890,y=y_axis+234)
btn_vitri38.place(x=x_axis+950,y=y_axis+234)
btn_vitri39.place(x=x_axis+1010,y=y_axis+234)
btn_vitri40.place(x=x_axis+1070,y=y_axis+234)
btn_vitri41.place(x=x_axis+650,y=y_axis+286)
btn_vitri42.place(x=x_axis+710,y=y_axis+286)
btn_vitri43.place(x=x_axis+770,y=y_axis+286)
btn_vitri44.place(x=x_axis+830,y=y_axis+286)
btn_vitri45.place(x=x_axis+890,y=y_axis+286)
btn_vitri46.place(x=x_axis+950,y=y_axis+286)
btn_vitri47.place(x=x_axis+1010,y=y_axis+286)
btn_vitri48.place(x=x_axis+1070,y=y_axis+286)
btn_vitri49.place(x=x_axis+650,y=y_axis+338)
btn_vitri50.place(x=x_axis+710,y=y_axis+338)
btn_vitri51.place(x=x_axis+770,y=y_axis+338)
btn_vitri52.place(x=x_axis+830,y=y_axis+338)
btn_vitri53.place(x=x_axis+890,y=y_axis+338)
btn_vitri54.place(x=x_axis+950,y=y_axis+338)
btn_vitri55.place(x=x_axis+1010,y=y_axis+338)
btn_vitri56.place(x=x_axis+1070,y=y_axis+338)
btn_vitri57.place(x=x_axis+650,y=y_axis+390)
btn_vitri58.place(x=x_axis+710,y=y_axis+390)
btn_vitri59.place(x=x_axis+770,y=y_axis+390)
btn_vitri60.place(x=x_axis+830,y=y_axis+390)
btn_vitri61.place(x=x_axis+890,y=y_axis+390)
btn_vitri62.place(x=x_axis+950,y=y_axis+390)
btn_vitri63.place(x=x_axis+1010,y=y_axis+390)
btn_vitri64.place(x=x_axis+1070,y=y_axis+390)
btn_vitri65.place(x=x_axis+1130,y=y_axis+208)
btn_vitri_reset.place(x=x_axis+1145,y=y_axis+120)





#####
####
#------------------------------------ TRẠNG THÁI BAN ĐẦU --------------------------------------
Px = KinematicRobot.Forward_Kinematic(0,0,0,L1,L2,L3)[0]
Py = KinematicRobot.Forward_Kinematic(0,0,0,L1,L2,L3)[1]
Pz = KinematicRobot.Forward_Kinematic(0,0,0,L1,L2,L3)[2]
txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới


GiaoDien.mainloop() # vong lap
