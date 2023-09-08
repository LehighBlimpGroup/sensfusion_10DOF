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

sys.path.append("ESP-NOW-Control\optitrack_natnet")
from NatNetClient import NatNetClient
from util import quaternion_to_euler
from simple_pid import PID
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import math


# correction of the angle 

def pi_clip(angle):
    if angle > 0:
        while angle > math.pi:
            angle = angle - 2*math.pi
    else:
        while angle < -math.pi:
            angle = angle + 2*math.pi
    return angle


positions = {}
rotations = {}


# definition to receive the data from the optitrack 
def receive_rigid_body_frame(id, position, rotation_quaternion):
    positions[id] = position
    # # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)

def find_point_on_circle(nx, ny, t, s):#n radius, t time, s frequency 
    # Calculate frequency
    f = 1 / s

    # Calculate the angle for time t
    angle = 2 * math.pi * f * t

    # Calculate x and y coordinates
    x = nx * math.cos(angle)
    y = ny * math.sin(angle)

    return x, y

PORT = 'COM5'

feedbackPD = { "roll" : 0,
  "pitch" : 0,
  "yaw" : 0,
  "x" : 0,
  "y" : 0,
  "z" : 0,
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
  "kdz" : .2,#-3
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
            #print("Received Data: " + incoming)
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
        0,0,
        0,0,0,0,0
    )
    esp_now_send(sock, esp_now_input)
    time.sleep(0.05)  # 0.005

if __name__ == "__main__":
    sock = espnow_init()


    sendAllFlags()
    
    time.sleep(0.05)  # 0.005

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
    time_all=  time.time()

    #####################################

    clientAddress = "192.168.0.14"
    optitrackServerAddress = "192.168.0.4"
    #tow_robot_id = 372
    lead_robot_id = 384

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)

     ###############################################

            
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()



    #tow_z_pid = PID(3, 0, 1, setpoint = 1, sample_time=0.01)
    lead_z_pid = PID(.6, 0.01, .4, setpoint = 1.5)
    yaw_pid = PID(0.4, 0.001, 0.2, setpoint = 0.)
    yaw_pid.error_map = pi_clip
    while (lead_robot_id not in positions):
        print("get optitrack online!")
        time.sleep(.5)
    # xy_d = np.array([positions[lead_robot_id][0], 
    #                 positions[lead_robot_id][1]])1.8234654664993286, -0.22969473898410797, 0.45290014147758484
    xy_d = np.array([1.823,0.4529])
    xy_center = np.array([0,0])

    xyp = .2#.6
    xyd = .2#.4
    xyi = 0.0005
    ex_norm_pid = PID(xyp, xyi, xyd, setpoint = 0.)
    ey_norm_pid = PID(xyp, xyi, xyd, setpoint = 0.)

    yaw_slow = rotations[lead_robot_id][2]
    
    last_x = 0
    last_y = 0
    xy_old = np.array([positions[lead_robot_id][0], 
                                positions[lead_robot_id][1]])
    ################################################
    
    try:
        while True:
            # if pygame.joystick.get_count() == 0:
            #     while pygame.joystick.get_count() == 0:
            #         print("joy_lost")
            #         time.sleep(.02)
            #     joystick = init()
                
            # Get the joystick readings
            pygame.event.pump()
            b = joystick.get_button(1)
            x = joystick.get_button(2)
            left = joystick.get_hat(0)[0] == -1
            right = joystick.get_hat(0)[0] == 1
            fy = 0
            if b == 1 and b_old == 0:
                b_state = not b_state
                if b_state:
                    while (lead_robot_id not in positions):
                        print("no tracking")
                        pass
                    xy_d = np.array([positions[lead_robot_id][0], 
                                    positions[lead_robot_id][1]])
                
            b_old = b

            if x == 1 and x_old == 0:
                x_state = not x_state
            x_old = x

            if abs(joystick.get_axis(3)) > 0.1:
                fx = -1 * joystick.get_axis(3)  # left handler: up-down, inverted
            else:
                fx = 0

            # if abs(joystick.get_axis(2)) > 0.1:
            #     tauz = -.2 * joystick.get_axis(2)  # right handler: left-right
            # else:
            #     tauz = 0
            if abs(joystick.get_axis(2)) > 0.1:
                fy = -1 * joystick.get_axis(2)  # right handler: left-right
            else:
                fy = 0

            fz = 0  # -2*joystick.get_axis(1)  # right handler: up-down, inverted



            l_old = left
            r_old = right
            
            tauy = 0
            taux = 0
            # absz = .5

            if abs(joystick.get_axis(1)) > 0.15:
                absz += -(time.time() - time_start) * joystick.get_axis(1)*.3
            
            
            if b_state == 0:
                absz = .4
                x_state = 0

            time_start = time.time()


            if lead_robot_id in positions: # read the body frame indicated
                xy = np.array([positions[lead_robot_id][0], 
                                positions[lead_robot_id][1]])
                time_elapsed = time.time() - time_all
                #print(xy)
                # if (b_state):
                #     print(time_elapsed)
                cx, cy = find_point_on_circle(2,1.3, time_elapsed, 90)
                #e_norm = np.linalg.norm(e_xy)   # euclidean distance
                

                #yaw_d = np.arctan2(e_xy[1], e_xy[0])  # heading angle 
                yaw_pid.setpoint = 0#np.arctan2(cx,cy)

                
                lead_tauz = yaw_pid(rotations[lead_robot_id][2]*np.pi/180)
                lead_fz = lead_z_pid(positions[lead_robot_id][2]) + .2
                #if (abs(positions[lead_robot_id][2] - 1.5) < .2):
                xy_d = xy_center + np.array([cx, cy])                    
                e_xy = xy_d - xy  # error x and y
                print(e_xy)

                # else:
                #     xy_d = xy_old#xy_center + np.array([cx, cy])
                #     e_xy = xy_d - xy  # error x and y
                #     xy_old = xy
                    
                lead_fx = - ex_norm_pid(e_xy[0])   # forces of x in the body frame 
                lead_fy = - ey_norm_pid(e_xy[1])   # forces of x in the body frame 
                force_vec = np.array([lead_fx, lead_fy])
                yaw = rotations[lead_robot_id][2]*np.pi/180
                rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                            [np.sin(yaw), np.cos(yaw)]])
                if np.linalg.norm(force_vec) > lead_fz*.7:
                    force_vec = force_vec/np.linalg.norm(force_vec) * lead_fz*.7
                f = rot.T.dot(force_vec)
                fx = f[0]
                fy = f[1]    
                tauz = lead_tauz
                absz = lead_fz



            esp_now_input = Control_Input(
                21,int(b_state), fx, fy, absz, taux, tauy, tauz, fz, 0, 0, 0, 0
            )
            esp_now_send(sock, esp_now_input)
                

            # state = not state
            time.sleep(0.01)  # 0.005
            # while(time.time() - time_start < 0.01):
            # time.sleep(0.001) #0.005
    except KeyboardInterrupt:
        print("The end")
        sys.exit()
