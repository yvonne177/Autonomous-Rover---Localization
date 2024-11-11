'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial
import numpy as np
# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.''' 
    start_time = time.time()
    response_raw = ''
    time.sleep(0.21)
    response_raw = SER.read(1024)
    response_new=response_raw.decode("ascii")
   # response_new= ((response_new.split('\',0).split(''')[1]))

    print(f'Raw response was:{response_raw}')
    
    # If response received, return it
    if response_raw:
        return [response_new, datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
##########################TURN OFF###########################
def depacketize(data_raw: str): 
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        print(FRAMESTART + data + FRAMEEND)
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid

#Define wall blocking 
min_dist = 10
def wall_blocking(u0, u1, u3, u5):
    '''
    Determine how many walls are surrounding the rover at a given instance"
    '''
    sensors = [u0, u1, u3, u5]
    walls = []
    for value in sensors:
        if value > min_dist:
            walls.append(0)
        if value <= min_dist:
            walls.append(1)
    blocking = walls.count(1)      
    if blocking ==3:
        return 3
    if blocking ==1:
        return 1
    if blocking ==0:
        return 0
    if blocking ==2:
        indices = [i for i in range(len(walls)) if walls[i]==1]
        if sum(indices)==3:
            return 5
        else:
            return 2
    else:
        return False

def SHIFT(direction, matrix):
    row_ind=-1
    inserted_min = (matrix.min())/2
    ZERO_ROW = np.full((1, 8), inserted_min)
    if direction =='UP':
        matrix[0] = matrix[1]
        matrix[1] = matrix[2]
        matrix[2] = matrix[3]
        matrix[3] = ZERO_ROW
        return matrix

    if direction =='DOWN':
        matrix[3] = matrix[2]
        matrix[2] = matrix[1]
        matrix[1] = matrix[0]
        matrix[0] = ZERO_ROW
        return matrix
    
    if direction == 'LEFT':
        for row in matrix:
            row_ind = row_ind+1
            removed_row=np.delete(row, 0)
            new_row=np.append(removed_row, inserted_min)
            matrix[row_ind] = new_row
        return matrix

    if direction == 'RIGHT':
        for row in matrix:
            row_ind = row_ind+1
            removed_row=np.delete(row, 7)
            new_row=np.insert(removed_row,0, inserted_min)
            matrix[row_ind] = new_row    
        return matrix
    else:
        return False
def CONVERGE(matrix):
    flat = matrix.flatten()
    flat.sort()
    max = float(flat[31])
    sec_max = float(flat[30])
    if max > 3*sec_max:
        return True
    else:
        return False
def TURN_HEADING(previous_ind, turn_amount):
    heading_ind = previous_ind + turn_amount
    if heading_ind > 3:
        heading_ind = heading_ind - 4
    if heading_ind < 0: 
        heading_ind = heading_ind + 4
    return heading_ind

def FIND_ROW(location):
    if location in range(0, 8):
        return 0
    if location in range(8, 16):
        return 1
    if location in range(16, 24):
        return 2
    if location in range(24, 32):
        return 3
    else:
        return False
    
def TURN_TO(current_heading, desired_heading):
    if current_heading != desired_heading:
        current_ind = COMPASS.index(current_heading)
        desired_ind = COMPASS.index(desired_heading)
        turn = current_ind-desired_ind
        if turn ==-3 or turn==1:
            return transmit(packetize('r:-80'))
        if turn ==3 or turn==-1:
            return transmit(packetize('r:80'))
        if turn ==2 or turn ==-2:
            return transmit(packetize('r:150'))
        if turn==0:
            return time.sleep(0.01)
            
############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM7'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = False



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0




############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = True # If true, run this. If false, skip it

#For obstacle avoidance
ct = 0
fix = False
min_dist_corner = 5 #min distance for the corner sensor
min_dist_front = 4.5
min_dist_fix = 6.5
parallel_sensor = 0.5
too_close = 1.5
LOOP_PAUSE_TIME = 0.1
PARALLEL_PAUSE = 0.01
count_turn =0
MOVE_COUNT=0

# Define Maze
X = 9
world = np.array(
        [[2, 1, 5, 2, X, 3, X, 3],
         [1, 2, X, 2, 5, 0, 5, 1],
         [5, X, 3, X, X, 5, X, 5],
         [2, 5, 1, 5, 5, 2, X, 3]])

LOCALIZED = False
high_prob=0.8
low_prob=0.1
NORM = 0.1
PROB = np.full((4,8), low_prob)
LOCATION = np.full((4,8), 1)  #Assume UP heading
ori_ind=-1
first_cycle = True

COMPASS=['UP', 'RIGHT', 'DOWN','LEFT']


B_PAUSE = 0.3
START_heading = int(input('What is the starting heading?' ))
B = float(input('What is B? Options are 5, 7, 18, 31:'))
COMPASS_IND =  START_heading #Assume UP starting orientation
orientation = COMPASS[COMPASS_IND]
command=input("Please enter 1 to start:")
#Begin by aligning parallel to walls

if command=='1':
    # Check all ultrasonic sensor 'us'
    packet_tx=packetize('us')
    if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            print(us_cleaned)
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    
    #Compare u1 and u3, check which wall is closest
    #If right wall closer,
    if u1_response > u3_response:
        print("Not Parallel!")
        uneven = True
        
            
        while uneven == True:
            #Checks which side it needs to rotate towards
            if u3_response > u4_response:
                transmit(packetize('r:-10'))
                time.sleep(PARALLEL_PAUSE)
            elif u3_response < u4_response:
                transmit(packetize('r:10'))
                time.sleep(PARALLEL_PAUSE)
            dump=receive()
            
            # Start moving once minimum distance cleared
            packet_tx=packetize('us')
            if packet_tx:
                    transmit(packet_tx)
                    time.sleep(B_PAUSE)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic ALL reading: {responses}")
                    us_response=responses[0:len(responses)-2]
                    us_cleaned=us_response.split(',')
                    print(us_cleaned)
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            
            if abs(u3_response-u4_response) <= parallel_sensor:
                uneven = False
                #stops rover once its parallel
                fixed = True
        

    clear_serial()

    #If left wall closer,
    if u1_response < u3_response:
        print("Not Parallel!")
        uneven = True
        while uneven == True:
            
            #Rotates to align with wall
            if u1_response > u2_response:
                transmit(packetize('r:10'))
                time.sleep(PARALLEL_PAUSE)
            elif u1_response < u2_response:
                transmit(packetize('r:-10'))
                time.sleep(PARALLEL_PAUSE)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
                print(us_cleaned)
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
                # Start moving once minimum distance cleared
            if abs(u1_response-u2_response) <= parallel_sensor:
                uneven = False
                fixed = True
    if u0_float < too_close:
            transmit(packetize('d:-0.75'))
            time.sleep(0.1)
            dump=receive
            
    if u1_response<2:
            #transmit(packetize('d:-1'))
            #time.sleep(LOOP_PAUSE_TIME)
            transmit(packetize('r:-5'))
            time.sleep(LOOP_PAUSE_TIME)
            dump=receive()   
    
    if u3_response<2:
            #transmit(packetize('d:-1'))
            #time.sleep(LOOP_PAUSE_TIME)
            transmit(packetize('r:5'))
            time.sleep(LOOP_PAUSE_TIME)
            dump=receive()
    clear_serial()   

        

time.sleep(LOOP_PAUSE_TIME)
dump=receive()
clear_serial

while not LOCALIZED:
    packet_tx=packetize('us')
    if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    
    print(f"TRACE",u0_float,u1_response,u2_response,u3_response,u5_response,u6_response,u7_response)   
    time.sleep(LOOP_PAUSE_TIME)
    clear_serial()
############### LOCALISATION ##########################################    
    if MOVE_COUNT in (0,2):
        #Get current possible locations based on walls
        current_walls = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        print(f"Walls are:", current_walls)
        
        #Get probability matrix based on current location
        for row in world:
            ori_ind = ori_ind+1
            for element in range(len(row)):
                if row[element]==current_walls:
                    PROB[ori_ind][element] = high_prob
        ori_ind = -1
        NEW_PROB = PROB
        #print(NEW_PROB)
        PROB = np.full((4,8), low_prob)
    
        if not first_cycle:
            LOCATION = SHIFT(COMPASS[COMPASS_IND], LOCATION)
            
        LOCATION = np.multiply(LOCATION, NEW_PROB)
        NORM = np.max(LOCATION)
        LOCATION = np.divide(LOCATION, NORM)
        print(LOCATION)
        
        #End loop once localized
        if CONVERGE(LOCATION):
            TRUE_LOCATION = np.argmax(LOCATION)
            TRUE_HEADING = COMPASS[COMPASS_IND]
            print(f"Localised! You are at:", TRUE_LOCATION, TRUE_HEADING)
            LOCALIZED = True
            break
        
        MOVE_COUNT=0
 ##################################################################################################       
    #Move to new location
    #If rover comes up to wall
    if u0_float < min_dist_front: #if its right by the wall
        print("Wall!")
        if u0_float < too_close:
            transmit(packetize('d:-0.75'))
            time.sleep(0.05)
            dump=receive()
            
        #If right reading is not close to wall and larger than the left reading, turns right
        lanes =  [u1_response, u3_response] 
        if current_walls == 3:
            lanes = [u1_response, u3_response, u5_response]
        clear_lane = max(lanes)
        
        if clear_lane == u1_response:
            transmit(packetize('r:80'))
            time.sleep(1)
            dump=receive()
            COMPASS_IND=TURN_HEADING(COMPASS_IND, 1)
            print(COMPASS[COMPASS_IND])
            
        if clear_lane == u3_response:
            transmit(packetize('r:-80'))
            time.sleep(1)
            dump=receive()
            COMPASS_IND=TURN_HEADING(COMPASS_IND, -1)
            print(COMPASS[COMPASS_IND])
        
        if clear_lane == u5_response:
            transmit(packetize('r:-190'))
            time.sleep(1)
            dump=receive()
            COMPASS_IND=TURN_HEADING(COMPASS_IND, -2)
            print(COMPASS[COMPASS_IND])
    
    # Check Left Corner
    if u6_response <= min_dist_corner and u0_float > min_dist_fix:
        fix = True
        print("Collision!")
        
        #fixes the rotational until fixed
        while fix == True: 
            transmit(packetize('r:-20'))
            print("sent r:-20")
            time.sleep(LOOP_PAUSE_TIME)
            dump = receive()
            #gets u6 value again to check
            packet_tx = packetize('u6')
            if packet_tx:
            
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic 6 reading: {responses}")
                u6_response = float(responses)
            
            # Start moving once minimum distance cleared
            if u6_response > min_dist_corner:
                fix = False   
    
    #Check Right Corner Sensor   
    if u7_response <= min_dist_corner and u0_float > min_dist_fix:
        fix = True
        print("Collision!")
        
        #fixes the rotational until fixed
        while fix == True:
    
            packet_tx=(packetize('r:20'))
            print("sent r:20")
            if packet_tx:
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #  [responses, time_rx] = receive() #why do we need this??
            time.sleep(LOOP_PAUSE_TIME)        
            #gets u7 value again to check
            packet_tx = packetize('u7')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(LOOP_PAUSE_TIME)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 7 reading: {responses}")
                u7_response = float(responses)
                dump=receive()
            
            # Start moving once minimum distance cleared
            if u7_response > min_dist_corner:
                fix = False
    
    first_cycle = False
    print('Forwards!')
    transmit(packetize('d:2.35'))
    MOVE_COUNT = MOVE_COUNT+1
    #Delay to allow rover to travel 1 block
    time.sleep(1)
    dump=receive()


######################################## DRIVE TO LZ ###############################################
# time.sleep(0.5)
# dump=receive()

#Reset compas index
COMPASS_IND = [0, 1, 2, 3]

#Initiate pause time for turns
TURN_PAUSE = 1

#Check if already in Loading Zone
if TRUE_LOCATION in (0, 1, 8, 9):
    LZ = True
    
    #LED blink
    data='1'
    SER.write(data.encode('ascii'))
    dump=receive()
    print('At Loading zone!')
    
else: 
    LZ = False
    
current_row = FIND_ROW(TRUE_LOCATION)
print(f"Current Row is:", current_row)
    
#Move to Loading zone
while not LZ:
    #If in row 0
    if current_row == 0 and TRUE_LOCATION in (2, 3):
        TURN_TO(TRUE_HEADING, 'LEFT')
        TRUE_HEADING = 'LEFT'
        TRUE_INDEX = 3
        time.sleep(TURN_PAUSE)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            time.sleep(B_PAUSE)
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        print(current_block)
        
        # time.sleep(B_PAUSE)
        # dump=receive()  
        
        while current_block != 1 and u0_float > min_dist_front:
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #   [responses, time_rx] = receive()
                    time.sleep(LOOP_PAUSE_TIME)
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            
            time.sleep(B_PAUSE)
            dump=receive() 
            
            
        TRUE_HEADING = 'LEFT'
        TRUE_INDEX = 3
        TRUE_LOCATION = 1
        LZ = True
        
        #LED blink
        data='1'
        SER.write(data.encode('ascii'))
        dump=receive()
        print("At Loading Zone!")
        break

    if current_row == 0 and TRUE_LOCATION not in (2,3):
        TURN_TO(TRUE_HEADING, 'DOWN')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'DOWN'
        TRUE_INDEX = 2
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        #Go to row 1
        while current_block not in (0,1):
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
            
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive()
            
            if current_block == 0 or current_block==1:
                print('Inch into lane')
                transmit(packetize('d:1'))
                #Delay to allow rover to travel 1 block
                time.sleep(1)
                dump=receive()
            
        current_row = 1
        if current_block == 0:
            TRUE_LOCATION = 13
            TRUE_HEADING = 'DOWN'
            TRUE_INDEX = 2
        if current_block ==1:
            TRUE_LOCATION = 15
            TRUE_HEADING = 'DOWN'
            TRUE_INDEX = 2
            
    #If in row 1   
    if current_row ==1:
        TURN_TO(TRUE_HEADING, 'LEFT')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'LEFT'
        TRUE_INDEX = 3
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
            dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        
        #Drive down the lane
        while u0_float > min_dist_front:
            
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
            
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
        
        #Backs up if too close to wall to turn
        if u0_float < too_close:
            print('Too close!')
            transmit(packetize('d:-0.75'))
            #time.sleep(1)
            dump=receive()
            
        #Turn into row 1
        TURN_TO(TRUE_HEADING, 'UP')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        packet_tx=packetize('us')
        if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        #Drive until in row 0
        while u0_float > min_dist_front:
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
            
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive()
        
        #Backup if too close to turn
        if u0_float < too_close:
            print('Too close!')
            transmit(packetize('d:-0.75'))
            time.sleep(1)
            dump=receive()  
              
        TURN_TO(TRUE_HEADING, 'LEFT')
        time.sleep(TURN_PAUSE)
        dump=receive()
        current_row = 0
        TRUE_LOCATION = 3
        TRUE_HEADING = 'LEFT'
        TRUE_INDEX = 3
        
        # while current_block != 1:
            
        #     # Check Left Corner
        #     if u6_response <= min_dist_corner and u0_float > min_dist_front:
        #         fix = True
        #         print("Collision!")
                
        #         #fixes the rotational until fixed
        #         while fix == True: 
        #             transmit(packetize('r:-20'))
        #             print("sent r:-20")
        #             if packet_tx:
        #                 transmit(packet_tx)
        #                 time.sleep(LOOP_PAUSE_TIME)
        #                 dump=receive()
        #                 #   [responses, time_rx] = receive()
        #             time.sleep(LOOP_PAUSE_TIME)
        #             #gets u6 value again to check
        #             packet_tx = packetize('u6')
        #             if packet_tx:
                    
        #                 transmit(packet_tx)
        #                 time.sleep(LOOP_PAUSE_TIME)
        #                 [responses, time_rx] = receive()
                        
        #                 print(f"Ultrasonic 6 reading: {responses}")
        #                 u6_response = float(responses)
                    
        #             # Start moving once minimum distance cleared
        #             if u6_response > min_dist_corner:
        #                 fix = False   
            
        #     #Check Right Corner Sensor   
        #     if u7_response <= min_dist_corner and u0_float > min_dist_front:
        #         fix = True
        #         print("Collision!")
                
        #         #fixes the rotational until fixed
        #         while fix == True:
            
        #             packet_tx=(packetize('r:20'))
        #             print("sent r:20")
        #             if packet_tx:
        #                 transmit(packet_tx)
        #                 time.sleep(LOOP_PAUSE_TIME)
        #                 dump=receive()
        #                 #  [responses, time_rx] = receive() #why do we need this??
        #             time.sleep(LOOP_PAUSE_TIME)        
        #             #gets u7 value again to check
        #             packet_tx = packetize('u7')
        #             if packet_tx:
        #                 transmit(packet_tx)
        #                 time.sleep(LOOP_PAUSE_TIME)
        #                 [responses, time_rx] = receive()
        #                 print(f"Ultrasonic 7 reading: {responses}")
        #                 u7_response = float(responses)
        #                 dump=receive()
                    
        #             # Start moving once minimum distance cleared
        #             if u7_response > min_dist_corner:
        #                 fix = False
        #     print('Forwards!')
        #     transmit(packetize('d:2.35'))
        #     #Delay to allow rover to travel 1 block
        #     time.sleep(1)
        #     dump=receive()
            
        #     packet_tx=packetize('us')
        #     if packet_tx:
        #         transmit(packet_tx)
        #         time.sleep(B_PAUSE)
        #         [responses, time_rx] = receive()
                
        #         print(f"Ultrasonic ALL reading: {responses}")
        #         us_response=responses[0:len(responses)-2]
        #         us_cleaned=us_response.split(',')
        #     dump=receive()  
        #     u0_float=float(us_cleaned[0])
        #     u1_response=float(us_cleaned[1])
        #     u2_response=float(us_cleaned[2])
        #     u3_response=float(us_cleaned[3])
        #     u4_response=float(us_cleaned[4])
        #     u5_response=float(us_cleaned[5])
        #     u6_response=float(us_cleaned[6])
        #     u7_response=float(us_cleaned[7])
        #     current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        #     time.sleep(B_PAUSE)
        #     dump=receive()
            
        # TRUE_HEADING = 'LEFT'
        # TRUE_INDEX = 3
        # TRUE_LOCATION = 1
        # LZ = True
        # print("At Loading Zone!")
        # break
    
    #If in row 2, drive  to row 1
    if current_row == 2 and TRUE_LOCATION != 16:
        TURN_TO(TRUE_HEADING, 'UP')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        packet_tx=packetize('us')
        if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        #Drive until in row 1
        while current_block != 1 and current_block != 0:
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
            
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            #print(current_row)
            time.sleep(B_PAUSE)
            dump=receive()
            
            if current_block == 0 or current_block==1:
                print('Inch into lane')
                transmit(packetize('d:1'))
                #Delay to allow rover to travel 1 block
                time.sleep(1)
                dump=receive()
                
        current_row = 1
        if current_block == 0:
            TRUE_LOCATION = 13
        if current_block == 1:
            TRUE_LOCATION = 15
            
        #print(f'Current row is (from 23):', current_row)
        
    if current_row == 2 and TRUE_LOCATION == 16:
        TURN_TO(TRUE_HEADING, 'UP')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        packet_tx=packetize('us')
        if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        
        while current_block != 1 and u0_float > min_dist_front:  
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive()
        TRUE_LOCATION = 8
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        current_row = 1
        LZ = True
        
        #LED blink
        data='1'
        SER.write(data.encode('ascii'))
        dump=receive()
        print("At Loading Zone!")
        break
        
        
    if current_row ==3 and TRUE_LOCATION != 31:
        TURN_TO(TRUE_HEADING, 'LEFT')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'LEFT'
        TRUE_INDEX = 3
        packet_tx=packetize('us')
        if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        
        time.sleep(B_PAUSE)
        dump=receive() 
        #Drive down row 3
        while u0_float > min_dist_front:  
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
        
        #Backup if too close to wall to turn
        if u0_float < too_close:
            print('Too close!')
            transmit(packetize('d:-0.75'))
            time.sleep(1)
            dump=receive()
            
        #turn towards LZ    
        TRUE_LOCATION = 24
        TURN_TO(TRUE_HEADING, 'UP')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        #drive to LZ
        while current_block !=1 and u0_float > min_dist_front:
            
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
        
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
            
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive()
            
        TRUE_LOCATION = 8
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        current_row = 1
        LZ = True
        
        #LED blink
        data='1'
        SER.write(data.encode('ascii'))
        dump=receive()
        print("At Loading Zone!")
        break
    
    if current_row ==3 and TRUE_LOCATION == 31:
        TURN_TO(TRUE_HEADING, 'UP')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'UP'
        TRUE_INDEX = 0
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        #drive to LZ
        while current_block !=1:
            
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
            
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive()
            
            if current_block==1:
                print('Inch into lane')
                transmit(packetize('d:1'))
                #Delay to allow rover to travel 1 block
                time.sleep(1)
                dump=receive()
                
        TRUE_LOCATION = 15
        TRUE_HEADING = 'UP'
        current_row = 1

################################# GO TO B #############################################

B_PAUSE = 0.3

time.sleep(1)
dump=receive()

data='0'
SER.write(data.encode('ascii'))
dump=receive()

#Get starting sensor readings
packet_tx=packetize('us')
if packet_tx:
    transmit(packet_tx)
    time.sleep(B_PAUSE)
    [responses, time_rx] = receive()
    
    print(f"Ultrasonic ALL reading: {responses}")
    us_response=responses[0:len(responses)-2]
    us_cleaned=us_response.split(',')
dump=receive()  
u0_float=float(us_cleaned[0])
u1_response=float(us_cleaned[1])
u2_response=float(us_cleaned[2])
u3_response=float(us_cleaned[3])
u4_response=float(us_cleaned[4])
u5_response=float(us_cleaned[5])
u6_response=float(us_cleaned[6])
u7_response=float(us_cleaned[7])
current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
time.sleep(B_PAUSE)
dump=receive() 

if B in (5, 7, 31):
    
    #Drive to row 0
    if TRUE_HEADING == 'UP':
        #Drive to row 0
        while u0_float > min_dist_front:
            if u0_float < min_dist_front:
                break
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
    
    #Turn to face end of row 0
    TURN_TO(TRUE_HEADING, 'RIGHT')
    TRUE_HEADING = 'RIGHT'
    TRUE_INDEX = 1
    time.sleep(TURN_PAUSE)
    dump=receive()
    packet_tx=packetize('us')
    if packet_tx:
        transmit(packet_tx)
        time.sleep(B_PAUSE)
        [responses, time_rx] = receive()
        
        print(f"Ultrasonic ALL reading: {responses}")
        us_response=responses[0:len(responses)-2]
        us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    #Drive down row 0
    while u0_float > min_dist_front:
        # Check Left Corner
        if u6_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True: 
                transmit(packetize('r:-20'))
                print("sent r:-20")
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #gets u6 value again to check
                packet_tx = packetize('u6')
                if packet_tx:
                
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic 6 reading: {responses}")
                    u6_response = float(responses)
                
                # Start moving once minimum distance cleared
                if u6_response > min_dist_corner:
                    fix = False   
        
        #Check Right Corner Sensor   
        if u7_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True:
                packet_tx=(packetize('r:20'))
                print("sent r:20")
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #  [responses, time_rx] = receive() #why do we need this??
                time.sleep(LOOP_PAUSE_TIME)        
                #gets u7 value again to check
                packet_tx = packetize('u7')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 7 reading: {responses}")
                    u7_response = float(responses)
                    dump=receive()
                
                # Start moving once minimum distance cleared
                if u7_response > min_dist_corner:
                    fix = False
        print('Forwards!')
        transmit(packetize('d:2.35'))
        #Delay to allow rover to travel 1 block
        time.sleep(1)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        dump=receive()
        time.sleep(B_PAUSE)

    #Backup if too close to wall to turn
    if u0_float < too_close:
        print('Too close!')
        transmit(packetize('d:-0.75'))
        time.sleep(1)
        dump=receive()
        
    #Turn and drive to row 1
    TURN_TO(TRUE_HEADING, 'DOWN')
    time.sleep(TURN_PAUSE)
    dump=receive()
    TRUE_LOCATION = 3
    TRUE_HEADING = 'DOWN'
    TRUE_INDEX = 1
    packet_tx=packetize('us')
    if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    #Drive until in row 1
    while u0_float > min_dist_front:
        # Check Left Corner
        if u6_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True: 
                transmit(packetize('r:-20'))
                print("sent r:-20")
                time.sleep(LOOP_PAUSE_TIME)
                dump = receive()
                #gets u6 value again to check
                packet_tx = packetize('u6')
                if packet_tx:
                
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic 6 reading: {responses}")
                    u6_response = float(responses)
                
                # Start moving once minimum distance cleared
                if u6_response > min_dist_corner:
                    fix = False   
        
        #Check Right Corner Sensor   
        if u7_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True:
                packet_tx=(packetize('r:20'))
                print("sent r:20")
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #  [responses, time_rx] = receive() #why do we need this??
                time.sleep(LOOP_PAUSE_TIME)        
                #gets u7 value again to check
                packet_tx = packetize('u7')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 7 reading: {responses}")
                    u7_response = float(responses)
                    dump=receive()
                
                # Start moving once minimum distance cleared
                if u7_response > min_dist_corner:
                    fix = False
        print('Forwards!')
        transmit(packetize('d:2.35'))
        #Delay to allow rover to travel 1 block
        time.sleep(1)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        dump=receive()
        time.sleep(B_PAUSE)
    
    #Backup if too close to wall to turn
    if u0_float < too_close:
        print('Too close!')
        transmit(packetize('d:-0.75'))
        time.sleep(1)
        dump=receive()
        
    print('Turning to row 1!')
    #Turn to face end of row 1
    TURN_TO(TRUE_HEADING, 'RIGHT')
    time.sleep(TURN_PAUSE)
    dump=receive()
    TRUE_LOCATION = 11
    TRUE_HEADING = 'RIGHT'
    TRUE_INDEX = 1
    packet_tx=packetize('us')
    if packet_tx:
        transmit(packet_tx)
        time.sleep(B_PAUSE)
        [responses, time_rx] = receive()
        
        print(f"Ultrasonic ALL reading: {responses}")
        us_response=responses[0:len(responses)-2]
        us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    if B == 5:
        #Drive to location with 0 walls
        while current_block != 0:
            
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    dump = receive()
                    time.sleep(LOOP_PAUSE_TIME)
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
            
            if current_block == 0:
                print('Inch into lane')
                transmit(packetize('d:1'))
                #Delay to allow rover to travel 1 block
                time.sleep(1)
                dump=receive()
        
        #Turn to face B
        TURN_TO(TRUE_HEADING, 'UP')
        time.sleep(TURN_PAUSE)
        dump=receive()
        TRUE_HEADING = 'UP'
        TRUE_LOCATION = 13
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        #Drive until in B
        while u0_float > min_dist_front:
            
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
        data='1'
        SER.write(data.encode('ascii'))
        dump=receive()
        print('Arrived at drop-off!')
        
    if B == 7 or B== 31:
        #Drive to location with 1 wall
        while u0_float > min_dist_front:
            # Check Left Corner
            if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True: 
                    transmit(packetize('r:-20'))
                    print("sent r:-20")
                    time.sleep(LOOP_PAUSE_TIME)
                    dump = receive()
                    #gets u6 value again to check
                    packet_tx = packetize('u6')
                    if packet_tx:
                    
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        
                        print(f"Ultrasonic 6 reading: {responses}")
                        u6_response = float(responses)
                    
                    # Start moving once minimum distance cleared
                    if u6_response > min_dist_corner:
                        fix = False   
            
            #Check Right Corner Sensor   
            if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                fix = True
                print("Collision!")
                
                #fixes the rotational until fixed
                while fix == True:
                    packet_tx=(packetize('r:20'))
                    print("sent r:20")
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #  [responses, time_rx] = receive() #why do we need this??
                    time.sleep(LOOP_PAUSE_TIME)        
                    #gets u7 value again to check
                    packet_tx = packetize('u7')
                    if packet_tx:
                        transmit(packet_tx)
                        time.sleep(LOOP_PAUSE_TIME)
                        [responses, time_rx] = receive()
                        print(f"Ultrasonic 7 reading: {responses}")
                        u7_response = float(responses)
                        dump=receive()
                    
                    # Start moving once minimum distance cleared
                    if u7_response > min_dist_corner:
                        fix = False
            print('Forwards!')
            transmit(packetize('d:2.35'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
        
        if B == 7:
            #Turn to face B
            TURN_TO(TRUE_HEADING, 'UP')
            time.sleep(TURN_PAUSE)
            dump=receive()
            TRUE_HEADING = 'UP'
            TRUE_LOCATION = 15
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        
        #Drive until in B    
            while u0_float > min_dist_front:
                # Check Left Corner
                if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                    fix = True
                    print("Collision!")
                    
                    #fixes the rotational until fixed
                    while fix == True: 
                        transmit(packetize('r:-20'))
                        print("sent r:-20")
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #gets u6 value again to check
                        packet_tx = packetize('u6')
                        if packet_tx:
                        
                            transmit(packet_tx)
                            time.sleep(LOOP_PAUSE_TIME)
                            [responses, time_rx] = receive()
                            
                            print(f"Ultrasonic 6 reading: {responses}")
                            u6_response = float(responses)
                        
                        # Start moving once minimum distance cleared
                        if u6_response > min_dist_corner:
                            fix = False   
                
                #Check Right Corner Sensor   
                if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                    fix = True
                    print("Collision!")
                    
                    #fixes the rotational until fixed
                    while fix == True:
                        packet_tx=(packetize('r:20'))
                        print("sent r:20")
                        if packet_tx:
                            transmit(packet_tx)
                            time.sleep(LOOP_PAUSE_TIME)
                            dump=receive()
                            #  [responses, time_rx] = receive() #why do we need this??
                        time.sleep(LOOP_PAUSE_TIME)        
                        #gets u7 value again to check
                        packet_tx = packetize('u7')
                        if packet_tx:
                            transmit(packet_tx)
                            time.sleep(LOOP_PAUSE_TIME)
                            [responses, time_rx] = receive()
                            print(f"Ultrasonic 7 reading: {responses}")
                            u7_response = float(responses)
                            dump=receive()
                        
                        # Start moving once minimum distance cleared
                        if u7_response > min_dist_corner:
                            fix = False
                print('Forwards!')
                transmit(packetize('d:2.35'))
                #Delay to allow rover to travel 1 block
                time.sleep(1)
                dump=receive()
                
                packet_tx=packetize('us')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(B_PAUSE)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic ALL reading: {responses}")
                    us_response=responses[0:len(responses)-2]
                    us_cleaned=us_response.split(',')
                dump=receive()  
                u0_float=float(us_cleaned[0])
                u1_response=float(us_cleaned[1])
                u2_response=float(us_cleaned[2])
                u3_response=float(us_cleaned[3])
                u4_response=float(us_cleaned[4])
                u5_response=float(us_cleaned[5])
                u6_response=float(us_cleaned[6])
                u7_response=float(us_cleaned[7])
                current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
                time.sleep(B_PAUSE)
                dump=receive()
            data='1'
            SER.write(data.encode('ascii'))
            dump=receive()
            print('Arrived at drop-off!')
        
        if B ==31:
            TURN_TO(TRUE_HEADING, 'DOWN')
            time.sleep(TURN_PAUSE)
            dump=receive()
            TRUE_HEADING = 'DOWN'
            TRUE_LOCATION = 15
            
            packet_tx=packetize('us')
            if packet_tx:
                transmit(packet_tx)
                time.sleep(B_PAUSE)
                [responses, time_rx] = receive()
                
                print(f"Ultrasonic ALL reading: {responses}")
                us_response=responses[0:len(responses)-2]
                us_cleaned=us_response.split(',')
            dump=receive()  
            u0_float=float(us_cleaned[0])
            u1_response=float(us_cleaned[1])
            u2_response=float(us_cleaned[2])
            u3_response=float(us_cleaned[3])
            u4_response=float(us_cleaned[4])
            u5_response=float(us_cleaned[5])
            u6_response=float(us_cleaned[6])
            u7_response=float(us_cleaned[7])
            current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
            time.sleep(B_PAUSE)
            dump=receive() 
            
            while u0_float > min_dist_front:
                
                # Check Left Corner
                if u6_response <= min_dist_corner and u0_float > min_dist_fix:
                    fix = True
                    print("Collision!")
                    
                    #fixes the rotational until fixed
                    while fix == True: 
                        transmit(packetize('r:-20'))
                        print("sent r:-20")
                        time.sleep(LOOP_PAUSE_TIME)
                        dump=receive()
                        #gets u6 value again to check
                        packet_tx = packetize('u6')
                        if packet_tx:
                        
                            transmit(packet_tx)
                            time.sleep(LOOP_PAUSE_TIME)
                            [responses, time_rx] = receive()
                            
                            print(f"Ultrasonic 6 reading: {responses}")
                            u6_response = float(responses)
                        
                        # Start moving once minimum distance cleared
                        if u6_response > min_dist_corner:
                            fix = False   
                
                #Check Right Corner Sensor   
                if u7_response <= min_dist_corner and u0_float > min_dist_fix:
                    fix = True
                    print("Collision!")
                    
                    #fixes the rotational until fixed
                    while fix == True:
                        packet_tx=(packetize('r:20'))
                        print("sent r:20")
                        if packet_tx:
                            transmit(packet_tx)
                            time.sleep(LOOP_PAUSE_TIME)
                            dump=receive()
                            #  [responses, time_rx] = receive() #why do we need this??
                        time.sleep(LOOP_PAUSE_TIME)        
                        #gets u7 value again to check
                        packet_tx = packetize('u7')
                        if packet_tx:
                            transmit(packet_tx)
                            time.sleep(LOOP_PAUSE_TIME)
                            [responses, time_rx] = receive()
                            print(f"Ultrasonic 7 reading: {responses}")
                            u7_response = float(responses)
                            dump=receive()
                        
                        # Start moving once minimum distance cleared
                        if u7_response > min_dist_corner:
                            fix = False
                print('Forwards!')
                transmit(packetize('d:2.35'))
                #Delay to allow rover to travel 1 block
                time.sleep(1)
                dump=receive()
                
                packet_tx=packetize('us')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(B_PAUSE)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic ALL reading: {responses}")
                    us_response=responses[0:len(responses)-2]
                    us_cleaned=us_response.split(',')
                dump=receive()  
                u0_float=float(us_cleaned[0])
                u1_response=float(us_cleaned[1])
                u2_response=float(us_cleaned[2])
                u3_response=float(us_cleaned[3])
                u4_response=float(us_cleaned[4])
                u5_response=float(us_cleaned[5])
                u6_response=float(us_cleaned[6])
                u7_response=float(us_cleaned[7])
                current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
                time.sleep(B_PAUSE)
                dump=receive()
            data='1'
            SER.write(data.encode('ascii'))
            dump=receive()
            print('Arrived at drop-off!')

if B == 18:
    #Turn to left wall
    TURN_TO(TRUE_HEADING, 'LEFT')
    TRUE_HEADING = 'LEFT'
    TRUE_INDEX = 3
    time.sleep(TURN_PAUSE)
    dump=receive()
    
    packet_tx=packetize('us')
    if packet_tx:
        transmit(packet_tx)
        time.sleep(B_PAUSE)
        [responses, time_rx] = receive()
        print(f"Ultrasonic ALL reading: {responses}")
        us_response=responses[0:len(responses)-2]
        us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    #Drive down row 0
    while u0_float > min_dist_front:
        # Check Left Corner
        if u6_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True: 
                transmit(packetize('r:-20'))
                print("sent r:-20")
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #gets u6 value again to check
                packet_tx = packetize('u6')
                if packet_tx:
                
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic 6 reading: {responses}")
                    u6_response = float(responses)
                
                # Start moving once minimum distance cleared
                if u6_response > min_dist_corner:
                    fix = False   
        
        #Check Right Corner Sensor   
        if u7_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True:
                packet_tx=(packetize('r:20'))
                print("sent r:20")
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #  [responses, time_rx] = receive() #why do we need this??
                time.sleep(LOOP_PAUSE_TIME)        
                #gets u7 value again to check
                packet_tx = packetize('u7')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 7 reading: {responses}")
                    u7_response = float(responses)
                    dump=receive()
                
                # Start moving once minimum distance cleared
                if u7_response > min_dist_corner:
                    fix = False
        print('Forwards!')
        transmit(packetize('d:2.35'))
        #Delay to allow rover to travel 1 block
        time.sleep(1)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
    #Backup if too close to wall to turn
    if u0_float < too_close:
        print('Too close!')
        transmit(packetize('d:-0.75'))
        time.sleep(1)
        dump=receive()
    
    #Turn to face end of 1st column
    TURN_TO(TRUE_HEADING, 'DOWN')
    TRUE_HEADING = 'DOWN'
    TRUE_INDEX = 2
    time.sleep(TURN_PAUSE)
    dump=receive()
    packet_tx=packetize('us')
    if packet_tx:
        transmit(packet_tx)
        time.sleep(B_PAUSE)
        [responses, time_rx] = receive()
        
        print(f"Ultrasonic ALL reading: {responses}")
        us_response=responses[0:len(responses)-2]
        us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    #Drive down leftmost column
    while u0_float > min_dist_front:
        
        # Check Left Corner
        if u6_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True: 
                transmit(packetize('r:-20'))
                print("sent r:-20")
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive
                #gets u6 value again to check
                packet_tx = packetize('u6')
                if packet_tx:
                
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic 6 reading: {responses}")
                    u6_response = float(responses)
                
                # Start moving once minimum distance cleared
                if u6_response > min_dist_corner:
                    fix = False   
        
        #Check Right Corner Sensor   
        if u7_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True:
                packet_tx=(packetize('r:20'))
                print("sent r:20")
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #  [responses, time_rx] = receive() #why do we need this??
                time.sleep(LOOP_PAUSE_TIME)        
                #gets u7 value again to check
                packet_tx = packetize('u7')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 7 reading: {responses}")
                    u7_response = float(responses)
                    dump=receive()
                
                # Start moving once minimum distance cleared
                if u7_response > min_dist_corner:
                    fix = False
        print('Forwards!')
        transmit(packetize('d:2.35'))
        #Delay to allow rover to travel 1 block
        time.sleep(1)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
    
    
    #Backup if too close to wall to turn
    if u0_float < too_close:
        print('Too close!')
        transmit(packetize('d:-0.75'))
        time.sleep(1)
        dump=receive()
        
    #Turn towards end of row 3
    TURN_TO(TRUE_HEADING, 'RIGHT')
    time.sleep(TURN_PAUSE)
    dump=receive()
    TRUE_HEADING = 'RIGHT'
    TRUE_LOCATION = 24
    TRUE_INDEX = 1
    
    packet_tx=packetize('us')
    if packet_tx:
        transmit(packet_tx)
        time.sleep(B_PAUSE)
        [responses, time_rx] = receive()
        
        print(f"Ultrasonic ALL reading: {responses}")
        us_response=responses[0:len(responses)-2]
        us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    while current_block != 1:

        # Check Left Corner
        if u6_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True: 
                transmit(packetize('r:-20'))
                print("sent r:-20")
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #gets u6 value again to check
                packet_tx = packetize('u6')
                if packet_tx:
                
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic 6 reading: {responses}")
                    u6_response = float(responses)
                
                # Start moving once minimum distance cleared
                if u6_response > min_dist_corner:
                    fix = False   
        
        #Check Right Corner Sensor   
        if u7_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True:
                packet_tx=(packetize('r:20'))
                print("sent r:20")
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #  [responses, time_rx] = receive() #why do we need this??
                time.sleep(LOOP_PAUSE_TIME)        
                #gets u7 value again to check
                packet_tx = packetize('u7')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 7 reading: {responses}")
                    u7_response = float(responses)
                    dump=receive()
                
                # Start moving once minimum distance cleared
                if u7_response > min_dist_corner:
                    fix = False
        print('Forwards!')
        transmit(packetize('d:2.35'))
        #Delay to allow rover to travel 1 block
        time.sleep(1)
        dump=receive()
        
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive() 
        
        if current_block == 1:
            print('Inch into lane')
            transmit(packetize('d:1'))
            #Delay to allow rover to travel 1 block
            time.sleep(1)
            dump=receive()
            
    
    #Turn towards B
    TURN_TO(TRUE_HEADING, 'UP')
    time.sleep(TURN_PAUSE)
    dump=receive()
    TRUE_HEADING = 'UP'
    TRUE_LOCATION = 26
    TRUE_INDEX = 0
    
    packet_tx=packetize('us')
    if packet_tx:
        transmit(packet_tx)
        time.sleep(B_PAUSE)
        [responses, time_rx] = receive()
        
        print(f"Ultrasonic ALL reading: {responses}")
        us_response=responses[0:len(responses)-2]
        us_cleaned=us_response.split(',')
    dump=receive()  
    u0_float=float(us_cleaned[0])
    u1_response=float(us_cleaned[1])
    u2_response=float(us_cleaned[2])
    u3_response=float(us_cleaned[3])
    u4_response=float(us_cleaned[4])
    u5_response=float(us_cleaned[5])
    u6_response=float(us_cleaned[6])
    u7_response=float(us_cleaned[7])
    current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
    time.sleep(B_PAUSE)
    dump=receive() 
    
    while u0_float > min_dist_front:
        
        # Check Left Corner
        if u6_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True: 
                transmit(packetize('r:-20'))
                print("sent r:-20")
                time.sleep(LOOP_PAUSE_TIME)
                dump=receive()
                #gets u6 value again to check
                packet_tx = packetize('u6')
                if packet_tx:
                
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    
                    print(f"Ultrasonic 6 reading: {responses}")
                    u6_response = float(responses)
                
                # Start moving once minimum distance cleared
                if u6_response > min_dist_corner:
                    fix = False   
        
        #Check Right Corner Sensor   
        if u7_response <= min_dist_corner and u0_float > min_dist_fix:
            fix = True
            print("Collision!")
            
            #fixes the rotational until fixed
            while fix == True:
                packet_tx=(packetize('r:20'))
                print("sent r:20")
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    dump=receive()
                    #  [responses, time_rx] = receive() #why do we need this??
                time.sleep(LOOP_PAUSE_TIME)        
                #gets u7 value again to check
                packet_tx = packetize('u7')
                if packet_tx:
                    transmit(packet_tx)
                    time.sleep(LOOP_PAUSE_TIME)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 7 reading: {responses}")
                    u7_response = float(responses)
                    dump=receive()
                
                # Start moving once minimum distance cleared
                if u7_response > min_dist_corner:
                    fix = False
        print('Forwards!')
        transmit(packetize('d:2.35'))
        #Delay to allow rover to travel 1 block
        time.sleep(1)
        dump=receive()
        packet_tx=packetize('us')
        if packet_tx:
            transmit(packet_tx)
            time.sleep(B_PAUSE)
            [responses, time_rx] = receive()
            
            print(f"Ultrasonic ALL reading: {responses}")
            us_response=responses[0:len(responses)-2]
            us_cleaned=us_response.split(',')
        dump=receive()  
        u0_float=float(us_cleaned[0])
        u1_response=float(us_cleaned[1])
        u2_response=float(us_cleaned[2])
        u3_response=float(us_cleaned[3])
        u4_response=float(us_cleaned[4])
        u5_response=float(us_cleaned[5])
        u6_response=float(us_cleaned[6])
        u7_response=float(us_cleaned[7])
        current_block = wall_blocking(u0_float, u1_response, u3_response, u5_response)
        time.sleep(B_PAUSE)
        dump=receive()
    data='1'
    SER.write(data.encode('ascii'))
    dump=receive()
    print('Arrived at drop-off!')