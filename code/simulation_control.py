import numpy as np
import sys, os, time
import logging, datetime
import dronekit
import argparse
import cv2
from CX_model import cx_rate, central_complex
from dronekit import VehicleMode
from CX_model.optical_flow import Optical_flow, FRAME_DIM
from CX_model.central_complex import update_cells
from CX_model.drone_ardupilot import arm, arm_and_takeoff, condition_yaw, send_ned_velocity

connection_string = "127.0.0.1:14550"

# initialize CX models
cx_gps = cx_rate.CXRate(noise = 0)
tb1_gps = np.zeros(central_complex.N_TB1)
memory_gps = 0.5 * np.ones(central_complex.N_CPU4)
cpu4_gps = np.zeros(16)

# connect to PX4 and arm
try:
    drone = dronekit.connect(connection_string)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')
state = arm_and_takeoff(drone, 5)

# set to mission mode.
drone.mode = VehicleMode("AUTO")
while drone.mode.name != "AUTO":
    print "Waiting for the mission mode."
    time.sleep(2)
# wait until reach first waypoint, 0->takeoff, 1->first waypoint
nextwaypoint = drone.commands.next
while nextwaypoint <= 1:
    print "Moving to waypoint", drone.commands.next+1
    nextwaypoint = drone.commands.next
    time.sleep(1)


start_time = time.time()
print "Start to update CX model, switch mode to end"
frame_num = 0
while drone.mode.name == "AUTO":
    # update CX neurons
    frame_num += 1
    velocity = drone.velocity
    drone_heading = drone.heading/180.0*np.pi

    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity = np.array([left_real, right_real]) / 20.0 # normarlization
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)


    
    

    # moniter the mission
    if frame_num % 10==0:
        # logging
        angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps) 
        print('heading:{} Angle:{} Distance:{} motor:{}'.format(drone.heading, 
              (angle_gps/np.pi)*180.0, distance_gps, motor_gps))

        display_seq = drone.commands.next
        print "Moving to waypoint: ", display_seq
        nextwaypoint = drone.commands.next
               

    time.sleep(0.1)

print "Mission ended or stoppped. The final results of CX model based on optcial flow is:"
print((angle_gps/np.pi) * 180, distance_gps)

time.sleep(1)
drone.mode = VehicleMode("GUIDED")
time.sleep(1)

navigation_heading = drone.heading
count = 0
while drone.mode.name == "GUIDED":

    velocity = drone.velocity
    drone_heading = drone.heading/180.0*np.pi

    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity = np.array([left_real, right_real]) / 20.0 # normarlization
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)
    
    frame_num += 1
    

    if (frame_num) % 5==0:
        heading = motor_gps*10000.0
        heading = np.min([np.max([-5,heading]), 5])
        print heading
        #navigation_heading += heading
        if np.abs(heading) > 2.0:
            print "rotating"
            condition_yaw(drone, heading, relative=True)
    if frame_num % 6 == 0:
       send_ned_velocity(drone, 3*np.cos(drone_heading), 3*np.sin(drone_heading), 0, 1)

    # logging
    if frame_num % 10==0:
        angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps) 
        print('heading:{} Angle:{} Distance:{} motor:{}'.format(drone_heading, 
              (angle_gps/np.pi)*180.0, distance_gps, motor_gps))

    time.sleep(0.1)

#drone.mode = VehicleMode("LAND")


drone.close()
cv2.destroyAllWindows()
