'''
Author       : Hanqing Qi
Date         : 2023-07-20 13:34:09
LastEditors  : Hanqing Qi
LastEditTime : 2023-08-01 17:18:55
FilePath     : /undefined/Users/hanqingqi/Desktop/sensfusion_10DOF/ESP-NOW-Control/Serial_Sender.py
Description  : Sender to send data to ESP32 through hardware serial port
'''
import serial
import time
import numpy as np
import sys
import pygame
import time
import socket
import struct
import math

PORT = 'COM5'

feedbackPD = { "roll" : 0,
  "pitch" : 0,
  "yaw" : 0,
  "x" : 0,
  "y" : 0,
  "z" : 1,
  "rotation" : 0,

  "Croll" : 0,
  "Cpitch" : 0, 
  "Cyaw" : 1,
  "Cx" : 1,
  "Cy" : 1,
  "Cz" : 1,
  "Cabsz" : 0,

  "kproll" : 0,
  "kdroll" : 0 ,
  "kppitch" : 0,
  "kdpitch" : 0,
  "kpyaw" : 3.0,
  "kdyaw" : -120,

  "kpx" : 0,
  "kdx" : 0,
  "kpy" : 0,
  "kdy" : 0,
  "kpz" : .2,#.5
  "kdz" : 0,#-3
  "kiz" : 0,

  "integral_dt" : 0,#.0001,
  "z_int_low" : 0,
  "z_int_high" : 200,

  "lx" : .15,
  "pitchSign" : 1,
  "pitchOffset" : -3.2816
}
weights = { "eulerGamma" : 0,
  "rollRateGamma" : 0.7,
  "yawRateGamma" : 0.975,
  "pitchRateGamma" : 0.7,
  "zGamma" : 0.5,
  "vzGamma" : 0.975
}

class Control_Input:
    def __init__(self, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.p5 = p5
        self.p6 = p6
        self.p7 = p7
        self.p8 = p8
        self.p9 = p9
        self.p10 = p10
        self.p11 = p11
        self.p12 = p12
        self.p13 = p13

    def __str__(self) -> str:
        return (
            '<'
            + str(self.p1)
            + '|'
            + str(self.p2)
            + '|'
            + str(self.p3)
            + '|'
            + str(self.p4)
            + '|'
            + str(self.p5)
            + '|'
            + str(self.p6)
            + '|'
            + str(self.p7)
            + '|'
            + str(self.p8)
            + '|'
            + str(self.p9)
            + '|'
            + str(self.p10)
            + '|'
            + str(self.p11)
            + '|'
            + str(self.p12)
            + '|'
            + str(self.p13)
            + '>'
        )


def espnow_init():
    ser = serial.Serial(PORT, 115200)
    return ser


def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick


def esp_now_send(ser, input):
    try:
        # NOTE - The daley here need to match the delay in the ESP32 receiver code
        message = str(input)
        ser.write(message.encode())
        try:
            incoming = ser.readline().decode(errors='ignore').strip()
            print("Received Data: " + incoming)
        except UnicodeDecodeError:
            print("Received malformed data!")
    except KeyboardInterrupt:
        print("Exiting Program")
        ser.close()


def init():
    joystick = joystick_init()
    return joystick

def sendAllFlags():
    esp_now_input = Control_Input(
        10, 0,
        feedbackPD["roll"], 
        feedbackPD["pitch"], 
        feedbackPD["yaw"],  
        feedbackPD["x"], 
        feedbackPD["y"], 
        feedbackPD["z"], 
        feedbackPD["rotation"], 
        0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        11, 0,
        feedbackPD["Croll"], 
        feedbackPD["Cpitch"], 
        feedbackPD["Cyaw"],  
        feedbackPD["Cx"], 
        feedbackPD["Cy"], 
        feedbackPD["Cz"], 
        feedbackPD["Cabsz"],
        0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        12, 0,
        feedbackPD["kproll"], 
        feedbackPD["kdroll"], 
        feedbackPD["kppitch"],  
        feedbackPD["kdpitch"], 
        feedbackPD["kpyaw"], 
        feedbackPD["kdyaw"], 
        0, 0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        13, 0,
        feedbackPD["kpx"], 
        feedbackPD["kdx"], 
        feedbackPD["kpy"],  
        feedbackPD["kdy"], 
        feedbackPD["kpz"], 
        feedbackPD["kdz"],  
        feedbackPD["lx"], 
        feedbackPD["pitchSign"],  
        feedbackPD["pitchOffset"], 
        0,0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        14, 0,
        weights["eulerGamma"], 
        weights["rollRateGamma"], 
        weights["pitchRateGamma"],  
        weights["yawRateGamma"], 
        weights["zGamma"], 
        weights["vzGamma"], 
        0, 0, 0, 0, 0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005
    
    esp_now_input = Control_Input(
        15, 0,
        feedbackPD["kiz"], 
        feedbackPD["integral_dt"],  
        feedbackPD["z_int_low"], 
        feedbackPD["z_int_high"],  
        0,0,0,0,0,0,0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005

if __name__ == "__main__":
    sock = espnow_init()


    sendAllFlags()

    joystick = init()
    l = 0.2  # meters
    absz = 0
    b_old = 0
    b_state = 0
    x_old = 0
    x_state = 1
    l_old = 0

    r_old = 0
    snap = 0

    tauz = 0
    fx = 0
    state = 0

    time_start = time.time()
    try:
        while True:
            # Get the joystick readings
            pygame.event.pump()
            b = joystick.get_button(1)
            x = joystick.get_button(2)
            left = joystick.get_hat(0)[0] == -1
            right = joystick.get_hat(0)[0] == 1
            
            if b == 1 and b_old == 0:
                b_state = not b_state
            b_old = b

            if x == 1 and x_old == 0:
                x_state = not x_state
            x_old = x

            if abs(joystick.get_axis(3)) > 0.1:
                fx = 1 * joystick.get_axis(3)  # left handler: up-down, inverted
            else:
                fx = 0
            if abs(joystick.get_axis(2)) > 0.1:
                fy = 1 * joystick.get_axis(2)  # right handler: left-right
            else:
                fy = 0
            if abs(joystick.get_axis(0)) > 0.1:
                tauz = 0.5 * joystick.get_axis(0)
            else:
                tauz = 0
            fz = 0  # -2*joystick.get_axis(1)  # right handler: up-down, inverted

            # if x_state:
            #     if left == 1 and l_old == 0:
            #         print("left")
            #         snap += 1
            #         tauz += 3.1415 / 4
            #     elif right == 1 and r_old == 0:
            #         print("right")
            #         snap += 1
            #         tauz += -3.1415 / 4
            # else:
            #     snap = 0

            l_old = left
            r_old = right
            
            tauy = 0
            taux = 0
            # absz = .5
            if abs(joystick.get_axis(1)) > 0.15:
                absz += -(time.time() - time_start) * joystick.get_axis(1)*.3
            if b_state == 0:
                absz = 1
                x_state = 0

            time_start = time.time()

            # print(
            #     round(fx, 2),
            #     round(fz, 2),
            #     round(taux, 2),
            #     round(tauz, 2),
            #     round(absz, 2),
            #     b_state,
            #     x_state,
            #     snap
            # )


            esp_now_input = Control_Input(
                21,int(b_state), fx, fy, absz, taux, tauy, tauz, fz, 0, 0, 0, 0
            )
            esp_now_send(sock, esp_now_input)
                

            # state = not state
            time.sleep(0.02)  # 0.005
            # while(time.time() - time_start < 0.01):
            # time.sleep(0.001) #0.005
    except KeyboardInterrupt:
        print("The end")
        sys.exit()
