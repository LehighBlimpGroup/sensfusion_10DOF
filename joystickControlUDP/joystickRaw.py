import pygame
import time
import socket
import struct
import math

# udp params
UDP_IP = "192.168.0.53" #192.168.0.05
UDP_PORT = 1333
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

def udp_init():
    sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM
    ) # UDP
    return sock

def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    joy = 0
    pygame.joystick.Joystick(joy).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(joy)
    return joystick

def init():
    sock = udp_init()
    joystick = joystick_init()
    return sock, joystick

def udp_send(sock, ip, port, message):
    sock.sendto(message, (UDP_IP, UDP_PORT))


if __name__ == "__main__":
    sock, joystick = init()
    joyx = 3
    joytz = 2
    l = 0.2 # meters
    absz = 0
    b_old = 0
    b_state = 1
    tauz = 0
    fx = 0
    while True:
        time_start = time.time()
        # Get the joystick readings
        pygame.event.pump()
        b = joystick.get_button(1)
        if b == 1 and b_old == 0:
            b_state = not b_state
        b_old = b
        if abs(joystick.get_axis(joyx) )> .1:
            fx = -1*joystick.get_axis(joyx) # left handler: up-down, inverted
        else:
            fx = 0
        taux = 0#joystick.get_axis(3) # left handler: left-right
        fz = 0#-2*joystick.get_axis(1)  # right handler: up-down, inverted
        if abs(joystick.get_axis(joytz)) > .2:
            tauz = -1*joystick.get_axis(joytz) # right handler: left-right
        else:
            tauz = 0
        fy = 0
        tauy = 0
        #absz = .5
        if abs(joystick.get_axis(1)) > .15:
            absz += -0.005*joystick.get_axis(1)
        if  b_state == 1:
            absz = .2
        

        
        # print(fx, taux, fz, tauz)

        # f1x = (fx - tauz/l)/2
        # f2x = (fx + tauz/l)/2
        # f1z = (fz + taux/l)/2
        # f2z = (fz - taux/l)/2

        # f1 = math.sqrt(f1x**2 + f1z**2) *255/3
        # f2 = math.sqrt(f2x**2 + f2z**2) *255/3
        # t1 = math.atan2(f1z, f1x)*180/math.pi
        # t2 = math.atan2(f2z, f2x)*180/math.pi
        print(round(fx,2), round(fz,2), round(taux,2), round(tauz,2), round(absz,2), b_state)
        # print()
        
        message = struct.pack('<ffffffff', fx, fy , fz, taux, tauy, tauz, absz, not b_state) 
        udp_send(sock, UDP_IP, UDP_PORT, message)
        #print(message)
        time.sleep(0.01) #0.005
        while(time.time() - time_start < 0.01):
            time.sleep(0.005) #0.005
