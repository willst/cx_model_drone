import numpy as np
import os,sys
import glob
import math
import matplotlib.pyplot as plt
LAT = 0
LON = 1 
data_crop = [300, -1]

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    latMid = (aLocation1[LAT]+aLocation2[LAT])/2.0  # or just use Lat1 for slightly less accurate estimate


    m_per_deg_lat = 111132.954 - 559.822 * np.cos(np.deg2rad(2.0*latMid)) + 1.175 * np.cos(np.deg2rad(4.0*latMid))
    m_per_deg_lon = (3.14159265359/180.0 ) * 6367449 * np.cos (np.deg2rad(latMid))

    deltaLat = np.abs(aLocation1[LAT] - aLocation2[LAT])
    deltaLon = np.abs(aLocation1[LON] - aLocation2[LON])

    dist_m = np.sqrt(np.power( deltaLat * m_per_deg_lat,2) + np.power(deltaLon * m_per_deg_lon , 2))
    return dist_m

def get_abgle_degrees(aLocation1, aLocation2):
    """
    Returns the angle in degree between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles.
    """
    latMid = (aLocation1[LAT]+aLocation2[LAT])/2.0  # or just use Lat1 for slightly less accurate estimate


    m_per_deg_lat = 111132.954 - 559.822 * np.cos(np.deg2rad(2.0*latMid)) + 1.175 * np.cos(np.deg2rad(4.0*latMid))
    m_per_deg_lon = (3.14159265359/180.0 ) * 6367449 * np.cos (np.deg2rad(latMid))

    deltaLat = np.abs(aLocation1[LAT] - aLocation2[LAT])
    deltaLon = np.abs(aLocation1[LON] - aLocation2[LON])

    dlat = np.sqrt(np.power(deltaLat * m_per_deg_lat,2))
    dlong = np.sqrt(np.power(deltaLon * m_per_deg_lon, 2))
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
    
    # position information
    alt = float(navigation_info[i].split('alt=')[-1])
    alt_list.append(alt)
    lat = float(navigation_info[i].split('lat=')[-1].split(',')[0])
    lon = float(navigation_info[i].split('lon=')[-1].split(',')[0])
    lat_list.append(lat)
    lon_list.append(lon)

    heading = float(navigation_info[i].split(' ')[2].split(':')[-1])
    heading_list.append(heading) 

    new_pos = [lat, lon, alt]
    dis_real = get_distance_metres(home_pos, new_pos)
    distance_real.append(dis_real)
    ang_real = get_abgle_degrees(home_pos, new_pos)
    angle_real.append(ang_real)

    # speed
    sl=float(navigation_info[i].split(' ')[0].split(':')[-1])
    sr=float(navigation_info[i].split(' ')[1].split(':')[-1])
    speed_left.append((sl)*alt*alt/25)
    speed_right.append((sr)*alt*alt/25)
    
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
    
    # CX model data
    distance = model_info[i].split('Distance_optical:')[1].split(' ')[0]
    distance_list_optical.append(float(distance))    
    distance = model_info[i].split('Distance_gps:')[-1].split(' ')[0]
    distance_list_gps.append(float(distance))

    angle = model_info[i].split('Angle_optical:')[1].split(' ')[0]
    angle_list_optical.append(float(angle))
    angle = model_info[i].split('Angle_gps:')[-1].split(' ')[0]
    angle_list_gps.append(float(angle))
    

    elapsed_time = model_info[i].split('elapsed_time:')[-1]
    time_list.append(float(elapsed_time))

# find minimal
min_distance_optical = np.amin(distance_list_optical[data_crop[0]:data_crop[1]])
index_o = np.argmin(distance_list_optical[data_crop[0]:data_crop[1]])
min_distance_optical_real = distance_real[data_crop[0]+index_o]

min_distance_gps = np.amin(distance_list_gps[data_crop[0]:data_crop[1]])
index_g = np.argmin(distance_list_gps[data_crop[0]:data_crop[1]])
min_distance_gps_real = distance_real[data_crop[0]+index_g]

min_distance_real = np.amin(distance_real[data_crop[0]:data_crop[1]])
index_r = np.argmin(distance_real[data_crop[0]:data_crop[1]])

print("Data number:", len(heading_list))
print "Final distance: ", distance_real[-1]
print "\nMinimal optical CX model distance and its index: ", min_distance_optical, data_crop[0]+index_o, ", \nat that time, real distance is:", min_distance_optical_real
print "\nMinimal gps CX model distance and its index: ", min_distance_gps, data_crop[0]+index_g, ", \n at that time, real distance is:", min_distance_gps_real
print "\nMinimal real distance during homing:", min_distance_real, data_crop[0]+index_r

# draw plots for analysis
x_axis = np.linspace(0, len(navigation_info), num=len(navigation_info), endpoint=False)

# speed reterival
fig1, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(ylabel='left')
ax2.set(ylabel='right')
ax2.set(xlabel='frames')
ax1.plot(x_axis, speed_left, 'r-', label='optical_based')
ax1.plot(x_axis, speed_left_real, 'b-', label='gps_based')
ax2.plot(x_axis, speed_right, 'r-')
ax2.plot(x_axis, speed_right_real, 'b-')
fig1.legend(ncol=2)
fig1.subplots_adjust(hspace=0.3)

# left-right speed compare
fig2, (ax3, ax4) = plt.subplots(2, sharey=True)
ax3.set(title='Speed compare', ylabel='optical based spped')
ax4.set(ylabel='GPS based speed')
ax4.set(xlabel='frames')
ax3.plot(x_axis, speed_left, 'r-' , label='left')
ax3.plot(x_axis, speed_right, 'b-', label='right')
ax4.plot(x_axis, speed_left_real, 'r-')
ax4.plot(x_axis, speed_right_real, 'b-')
fig2.legend(ncol=2)
fig2.subplots_adjust(hspace=0.3)


# show distance and angle
fig3, (ax5, ax6, ax7) = plt.subplots(3, sharey=False)
ax5.set(title='Navigation data', ylabel='heading')

ax6.set(ylabel='home_vector distance')
ax7.set(xlabel='frames', ylabel='gps_based distance')

ax5.plot(x_axis, heading_list)
ax6.plot(x_axis, distance_list_optical, 'r-', label='optical_based')
ax6.plot(x_axis, distance_list_gps, 'b-', label='gps_based')
ax7.plot(x_axis, distance_real, 'g-', label='real')
fig3.legend(ncol=3)


# other information
fig3, (ax8, ax9, ax10) = plt.subplots(3, sharey=False)
ax8.set(ylabel='home_vector angle')
ax8.set(ylabel='home_vector distance')
ax10.set(xlabel='frames', ylabel='elapsed_time')

ax8.plot(x_axis, alt_list, 'g-')
ax9.plot(x_axis, angle_list_optical, 'r-', label='optical_CX')
ax9.plot(x_axis, angle_list_gps, 'b-', label='gps_CX')
ax9.plot(x_axis, angle_real, 'g-', label='real')
ax10.plot(x_axis, time_list, 'g-')
fig3.legend(ncol=3)
plt.show()
