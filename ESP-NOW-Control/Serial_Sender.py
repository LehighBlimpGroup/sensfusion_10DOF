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


if __name__ == "__main__":
    sock = espnow_init()
    joystick = init()
    l = 0.2  # meters
    absz = 0
    b_old = 0
    b_state = 1
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
                fx = -.8 * joystick.get_axis(3)  # left handler: up-down, inverted
            else:
                fx = 0
            if abs(joystick.get_axis(0)) > 0.1:
                taux = -0.03 * joystick.get_axis(0)
            else:
                taux = 0
            fz = 0  # -2*joystick.get_axis(1)  # right handler: up-down, inverted

            if x_state:
                if left == 1 and l_old == 0:
                    print("left")
                    snap += 1
                    tauz += 3.1415 / 4
                elif right == 1 and r_old == 0:
                    print("right")
                    snap += 1
                    tauz += -3.1415 / 4
            else:
                snap = 0
                if abs(joystick.get_axis(2)) > 0.1:
                    tauz = -5 * joystick.get_axis(2)  # right handler: left-right
                else:
                    tauz = 0
            l_old = left
            r_old = right
            fy = 0
            tauy = 0
            # absz = .5
            if abs(joystick.get_axis(1)) > 0.15:
                absz += -(time.time() - time_start) * joystick.get_axis(1)
            if b_state == 1:
                absz = 3
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
                0, fx, fy, fz, taux, tauy, tauz, absz, int(not b_state), int(x_state), 0, 0, 0
            )
            esp_now_send(sock, esp_now_input)
                

            # state = not state
            time.sleep(0.02)  # 0.005
            # while(time.time() - time_start < 0.01):
            # time.sleep(0.001) #0.005
    except KeyboardInterrupt:
        print("The end")
        sys.exit()
