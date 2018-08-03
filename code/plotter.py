import numpy as np
import os,sys
import glob
import math
import matplotlib.pyplot as plt
LAT = 0
LON = 1 

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2[LAT] - aLocation1[LAT]
    dlong = aLocation2[LON]- aLocation1[LON]
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_abgle_degrees(aLocation1, aLocation2):
    """
    Returns the angle in degree between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles.
    """
    dlat = (aLocation2[LAT] - aLocation1[LAT]) * 1.113195e5
    dlong = (aLocation2[LON]- aLocation1[LON]) * 1.113195e5
    return -np.arctan2(dlong, dlat)/np.pi*180

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
    speed_left.append(float(sl))#*alt*alt/25)
    speed_right.append(float(sr))#*alt*alt/25)

    heading = float(navigation_info[i].split(' ')[2].split(':')[-1])
    heading_list.append(heading)
    
    velocity = navigation_info[i].split('[')[1].split(']')[0].split(', ')
    velocity = [float(j) for j in velocity]
    velocity_x_list.append((velocity[0]))
    velocity_y_list.append((velocity[1]))

    left_real = (velocity[0]*np.cos(heading/180.0*np.pi-np.pi/4) + 
                velocity[1]*np.cos(heading/180.0*np.pi-np.pi/4-np.pi/2))
    speed_left_real.append(left_real/4.0)
    right_real = (velocity[0]*np.cos(heading/180.0*np.pi+np.pi/4) + 
                velocity[1]*np.cos(heading/180.0*np.pi+np.pi/4-np.pi/2))
    speed_right_real.append(right_real/4.0)   
    
    
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

print("Data number:", len(heading_list))

# draw plots for analysis
x_axis = np.linspace(0, len(navigation_info), num=len(navigation_info), endpoint=False)

# speed reterival
fig1, (ax1, ax2, ax3, ax10) = plt.subplots(4, sharey=False)
ax1.set(title='Speed and altitude', ylabel='left')
ax2.set(ylabel='right')
ax3.set(ylabel='Altitude')
ax10.set(xlabel='time (s)', ylabel='elapsed_time')
ax1.plot(x_axis, speed_left, 'r-', label='optical_based')
ax1.plot(x_axis, speed_left_real, 'b-', label='gps_based')
ax2.plot(x_axis, speed_right, 'r-')
ax2.plot(x_axis, speed_right_real, 'b-')
ax3.plot(x_axis, alt_list, 'g-')
ax10.plot(x_axis, time_list, 'g-')
fig1.legend(ncol=3)
fig1.subplots_adjust(hspace=0.3)
# left-right speed compare
fig2, (ax4, ax5) = plt.subplots(2, sharey=False)
ax4.set(title='Speed compare', ylabel='optical')
ax5.set(xlabel='time (s)', ylabel='real')

ax4.plot(x_axis, speed_left, 'r-')
ax4.plot(x_axis, speed_right, 'b-')
ax5.plot(x_axis, speed_left_real, 'r-')
ax5.plot(x_axis, speed_right_real, 'b-')

# show distance and angle
fig3, (ax6, ax7, ax8, ax9) = plt.subplots(4, sharey=False)
ax6.set(title='Navigation data', ylabel='heading')
ax7.set(ylabel='home_vector angle')
ax8.set(ylabel='home_vector distance')
ax9.set(xlabel='time (s)', ylabel='gps_based distance')

ax6.plot(x_axis, heading_list)
ax7.plot(x_axis, angle_list_optical, 'r-', label='optical_CX')
ax7.plot(x_axis, angle_list_gps, 'b-', label='gps_CX')
ax7.plot(x_axis, angle_real, 'g-', label='real')
ax8.plot(x_axis, distance_list_optical, 'r-')
ax8.plot(x_axis, distance_list_gps, 'b-')
ax9.plot(x_axis, distance_real, 'g-')
fig3.legend(ncol=3)
plt.show()

