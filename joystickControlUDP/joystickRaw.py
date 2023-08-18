import sys
import pygame
import time
import socket
import struct
import math

# udp params
UDP_IP = "192.168.0.55" #192.168.0.05
UDP_PORT = 1345
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

def udp_init():
    sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM
    ) # UDP
    # allows sock to obtain data without stopping to wait for it
    sock.setblocking(1)

    # sock binds to the interfaces and uses port 1333
    sock.bind(('', UDP_PORT))
    return sock

def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick

def init():
    sock = udp_init()
    joystick = joystick_init()
    return sock, joystick

def udp_send(sock, ip, port, message):
    sock.sendto(message, (UDP_IP, UDP_PORT))

##################################################################
#Code implemented from: https://www.youtube.com/watch?v=PQ27j8WAP8Q

def check_data():
    try:
        # Data received
        data, addr = sock.recvfrom(1024)
        #print("received message: %s from %s" % (data,addr))

        # return the decoded bytes as a string
        return data.decode()
    # If no data is received just return None
    except socket.error:
        return None

def main():
    # Main loop
    #while True:
    # Check for UDP data
    line = check_data()
    # If there is data split it and print it to console
    if line:
        split_line = line.split('|')
        print(split_line)

#####################################################################
        
if __name__ == "__main__":

    sock, joystick = init()
    l = 0.2 # meters
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
            left = joystick.get_button(13)
            right = joystick.get_button(14)
            if b == 1 and b_old == 0:
                b_state = not b_state
            b_old = b

            if x == 1 and x_old == 0:
                x_state = not x_state
            x_old = x

            if abs(joystick.get_axis(3) )> .1:
                fx = -1*joystick.get_axis(3) # left handler: up-down, inverted
            else:
                fx = 0
            if abs(joystick.get_axis(0)) > .1:
                taux = -0.1*joystick.get_axis(0) 
            else:
                taux = 0
            fz = 0#-2*joystick.get_axis(1)  # right handler: up-down, inverted
            
            if x_state:
                if left == 1 and l_old == 0:
                    snap += 1
                    tauz = 3.1415/4
                elif right == 1 and r_old == 0:
                    snap += 1
                    tauz = -3.1415/4
            else:
                snap = 0
                if abs(joystick.get_axis(2)) > .1:
                    tauz = -2*joystick.get_axis(2) # right handler: left-right
                else:
                    tauz = 0
            l_old = left
            r_old = right
            fy = 0
            tauy = 0
            #absz = .5
            if abs(joystick.get_axis(1)) > .15:
                absz += -(time.time() - time_start)*joystick.get_axis(1)
            if  b_state == 1:
                absz = .2
                x_state = 0
            
            time_start = time.time()

            
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
            if state < 40:
                message = struct.pack('<fffffffffffff', 0, fx, fy , fz, taux, tauy, tauz, absz, not b_state,snap,0,0,0) 
                udp_send(sock, UDP_IP, UDP_PORT, message)
                #print(message)
                state += 1
            else:
                #main()#prints out information from drone
                state = 0
            
            #state = not state
            time.sleep(0.02) #0.005
            #while(time.time() - time_start < 0.01):
                #time.sleep(0.001) #0.005
    except KeyboardInterrupt:
        print("The end")
        sys.exit()
