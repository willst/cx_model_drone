import os, sys
import numpy as np
import time
import central_complex
import cx_rate
import cx_basic
LAT = 0
LON = 1
 
def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    flow = cx.get_flow(heading, velocity, filtered_steps)
    tn1 = cx.tn1_output(velocity)
    tn2 = cx.tn2_output(velocity)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2[LAT] - aLocation1[LAT]
    dlong = aLocation2[LON]- aLocation1[LON]
    return np.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_abgle_degrees(aLocation1, aLocation2):
    """
    Returns the angle in degree between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles.
    """
    dlat = (aLocation2[LAT] - aLocation1[LAT]) * 1.113195e5
    dlong = (aLocation2[LON]- aLocation1[LON]) * 1.113195e5
    return -np.arctan2(dlong, dlat)/np.pi*180
'''
# load log file
error_log_path = sys.argv[1]
with open(error_log_path) as f:
    data = f.read()
data = data.split('\n')
while data[0].split(':')[2] != 'sl':
    print data[0]
    del data[0]

navigation_info = []
model_info = []
for i in range(len(data)/2):
    navigation_info.append(data[2*i])
    model_info.append(data[2*i+1])

# get different data from the log file 
heading_list = []
speed_left = []
speed_left_real = []
speed_right = []
speed_right_real = []
velocity_x_list = []
velocity_y_list = []

alt_list = []
lat_list = []
lon_list = []

time_list = []

distance_list_optical = []
distance_list_gps = []
angle_list_optical = []
angle_list_gps = []
distance_real = []
angle_real = []

alt = float(navigation_info[0].split('alt=')[-1])
lat = float(navigation_info[0].split('lat=')[-1].split(',')[0])
lon = float(navigation_info[0].split('lon=')[-1].split(',')[0])
home_pos = [lat, lon, alt]
for i in range(len(navigation_info)):
    
    alt = float(navigation_info[i].split('alt=')[-1])
    alt_list.append(alt)
    lat = float(navigation_info[i].split('lat=')[-1].split(',')[0])
    lon = float(navigation_info[i].split('lon=')[-1].split(',')[0])
    lat_list.append(lat)
    lon_list.append(lon)

    sl=navigation_info[i].split(' ')[0].split(':')[-1]
    sr=navigation_info[i].split(' ')[1].split(':')[-1]
    speed_left.append(float(sl)) #*alt*alt/25)
    speed_right.append(float(sr))  #*alt*alt/25)

    heading = float(navigation_info[i].split(' ')[2].split(':')[-1])
    heading_list.append(heading)
    
    velocity = navigation_info[i].split('[')[1].split(']')[0].split(', ')
    velocity = [float(j) for j in velocity]
    velocity_x_list.append((velocity[0]))
    velocity_y_list.append((velocity[1]))

    left_real = (velocity[0]*np.cos(heading/180.0*np.pi-np.pi/4) + 
                velocity[1]*np.cos(heading/180.0*np.pi-np.pi/4-np.pi/2))
    speed_left_real.append(left_real/1.0)
    right_real = (velocity[0]*np.cos(heading/180.0*np.pi+np.pi/4) + 
                velocity[1]*np.cos(heading/180.0*np.pi+np.pi/4-np.pi/2))
    speed_right_real.append(right_real/1.0)   
    
    
    elapsed_time = model_info[i].split('elapsed_time:')[-1]
    time_list.append(float(elapsed_time))

    distance = model_info[i].split('Distance_optical:')[1].split(' ')[0]
    distance_list_optical.append(float(distance))    
    distance = model_info[i].split('Distance_gps:')[-1].split(' ')[0]
    distance_list_gps.append(float(distance))

    new_pos = [lat, lon, alt]
    dis_real = get_distance_metres(home_pos, new_pos)
    distance_real.append(dis_real)
    ang_real = get_abgle_degrees(home_pos, new_pos)
    angle_real.append(ang_real)

    angle = model_info[i].split('Angle_optical:')[1].split(' ')[0]
    angle_list_optical.append(float(angle))
    angle = model_info[i].split('Angle_gps:')[-1].split(' ')[0]
    angle_list_gps.append(float(angle))

'''
# initialize CX model
#cx = cx_basic.CXBasic()
cx = cx_rate.CXRate(noise=0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

T = 300
velocity = np.ones([T,2], dtype=float) 
headings = np.zeros(T, dtype=float) + 90
headings[150:300] = 0
heading_list = headings
for t in range(T): #range(len(speed_left)): #
        #speed = np.array([speed_left_real[t], speed_right_real[t]])
        speed = velocity[t];

        __, __, tb1, __, __, memory, cpu4, __, motor = update_cells(
            heading=heading_list[t]/180.0*np.pi, velocity=speed, tb1=tb1, memory=memory, cx=cx)

angle, distance = cx.decode_cpu4(cpu4)
angle_degree = angle/np.pi * 180
print "Epoch %d, Angle:%.2f  Distance:%.2f Motor:%.2f" % (t, angle_degree, distance, motor)
#time.sleep(0.1)

_cx = cx_rate.CXRate(noise=0)
_tb1 = np.zeros(central_complex.N_TB1)
_memory = 0.5 * np.ones(central_complex.N_CPU4)
velocity = np.ones([T,2], dtype=float) 
headings = np.zeros(T, dtype=float) - 90
headings[150:300] = 0
heading_list = headings

for t in range(T): #range(len(speed_left)): #
        #speed = np.array([speed_left_real[t], speed_right_real[t]])
        speed = velocity[t];

        __, __, _tb1, __, __, _memory, _cpu4, __, motor = update_cells(
            heading=heading_list[t]/180.0*np.pi, velocity=speed, tb1=_tb1, memory=_memory, cx=_cx)

angle, distance = cx.decode_cpu4(_cpu4)
angle_degree = angle/np.pi * 180
print "Epoch %d, Angle:%.2f  Distance:%.2f Motor:%.2f" % (t, angle_degree, distance, motor)

cx_sc = cx_rate.CXRate(noise=0)
tb1_sc = tb1 + _tb1
memory_sc = memory + _memory
__, __, tb1_sc, __, __, memory_sc, cpu4_sc, __, motor = update_cells(
            heading=0, velocity=np.array([0,0]), tb1=tb1_sc, memory=memory_sc, cx=cx_sc)

angle, distance = cx_sc.decode_cpu4(cpu4_sc)
angle_degree = angle/np.pi * 180
print "Epoch %d, Angle:%.2f  Distance:%.2f Motor:%.2f" % (t, angle_degree, distance, motor)



