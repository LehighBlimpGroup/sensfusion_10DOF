import sys
import pygame
import time
import socket
import struct
import math

# udp params
UDP_IP = "192.168.0.2" #192.168.0.05
UDP_PORT = 1333
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



def dataMain():
    # Main loop
    #while True:
    # Check for UDP data
    line = check_data()
    # If there is data split it and print it to console
    if line:
        numvals += 1
        split_line = line.split('|')
        if split_line[0] == 1:
            otherstate = 1
            print(split_line)
            values.append(split_line[1:])
        elif split_line[0] == 2:
            otherstate = 2

    

#####################################################################
        
if __name__ == "__main__":

    sock, joystick = init()
    l = 0.2 # meters
    absz = 0
    b_old = 0
    b_state = 1
    tauz = 0
    fx = 0
    state = 0

    values = []
    numvals = 0
    otherstate = 0
    
    time_start = time.time()
    try:

        while numvals < 300:
            
            # print()
            
            message = struct.pack('<fffffffffffff', 1,0,0,0,0,0,0,0,0,0,0,0,0) 
            udp_send(sock, UDP_IP, UDP_PORT, message)
            #print(message)
            state += 1
        
        
            line = check_data()
            # If there is data split it and print it to console
            if line:
                numvals += 1
                split_line = line.split('|')
                if split_line[0] == 1:
                    otherstate = 1
                    print(split_line)
                    values.append(split_line[1:])
                elif split_line[0] == 2:
                    otherstate = 2#prints out information from drone
            state = 0
            
            #state = not state
            time.sleep(0.005) #0.005
            #while(time.time() - time_start < 0.01):
                #time.sleep(0.001) #0.005

        while otherstate != 2:
            message = struct.pack('<fffffffffffff', 2,1,1,1,1,1,1,1,1,1,1,1,1) 
            udp_send(sock, UDP_IP, UDP_PORT, message)
            #print(message)
            state += 1
        
        
            line = check_data()
            # If there is data split it and print it to console
            if line:
                numvals += 1
                split_line = line.split('|')
                if split_line[0] == 1:
                    otherstate = 1
                    print(split_line)
                    values.append(split_line[1:])
                elif split_line[0] == 2:
                    otherstate = 2#prints out information from drone
            state = 0
            
            #state = not state
            time.sleep(0.005) #0.005

        

    except KeyboardInterrupt:
        print("The end")
        sys.exit()
