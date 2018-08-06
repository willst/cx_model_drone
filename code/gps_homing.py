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
from CX_model.drone_ardupilot import arm, arm_and_takeoff, condition_yaw, \
      send_ned_velocity, adds_3wayP_mission, adds_10wayP_mission

#connection_string = "127.0.0.1:14550"
connection_string = '/dev/ttyAMA0'
HEIGHT = 4

# command line arguments handlder
parser = argparse.ArgumentParser(description='CX model navigation.')
parser.add_argument('--scale', default = 1.0, type=float,
                    help='scale of the route')
parser.add_argument('--windy', default = 1, type=bool,
                    help='Route depends on the weather, no windy make it more random')
args = parser.parse_args()
scale = args.scale
windy = args.windy

# connect to PX4 and upload mission
try:
    drone = dronekit.connect(connection_string, baud = 921600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')

cmds = drone.commands
cmds.download()
cmds.wait_ready()
drone.home_location = drone.location.global_frame
time.sleep(0.1)
home=drone.home_location
adds_3wayP_mission(drone, home, drone.heading, HEIGHT, scale=scale, windy=windy)
#adds_10wayP_mission(drone, home, drone.heading, HEIGHT, scale=scale, windy=windy)

# initialize logger
time_string = str(datetime.datetime.now()).replace(':', '-').replace(' ', '_').split('.')[0]
fname = 'log_sim/' + time_string + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)


# initialize CX models
cx_gps = cx_rate.CXRate(noise = 0)
tb1_gps = np.zeros(central_complex.N_TB1)
memory_gps = 0.5 * np.ones(central_complex.N_CPU4)
cpu4_gps = np.zeros(16)

# takeoff and set to mission mode.
state = arm_and_takeoff(drone, HEIGHT)
drone.mode = VehicleMode("AUTO")
while drone.mode.name != "AUTO":
    print "Pre-intialisation, waiting for the AUTO mode."
    time.sleep(2)
# wait until reach first waypoint, 1->home, 2->takeoff
nextwaypoint = drone.commands.next
while nextwaypoint <= 1:
    print "Initialisation, Moving to waypoint", drone.commands.next+1
    nextwaypoint = drone.commands.next
    time.sleep(1)

# -------------------------start mission--------------------------------
# -----------------end when mode switched ------------------------------
#-----------------------------------------------------------------------
# moving out, update CX model
sl = 0
sr = 0
angle_optical = 0
distance_optical = 0
frame_num = 0
start_time = time.time()
print "Start to update CX model, switch mode to end"
while drone.mode.name == "AUTO":

    frame_num += 1
    # update CX neurons
    drone_heading = drone.heading/180.0*np.pi
    velocity = drone.velocity
    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity_gps = np.array([left_real, right_real]) / 4.0   # normarlize velocity [-1,1]
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity_gps, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)

    elapsed_time = time.time() - start_time
    start_time = time.time()
    # logging
    logging.info('sl:{} sr:{} heading:{} velocity:{} position:{}'.format(
                sl,sr,drone.heading,drone.velocity, drone.location.global_relative_frame))
    angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps)
    logging.info('Angle_optical:{} Distance_optical:{} Angle_gps:{} Distance_gps:{} \
                 elapsed_time:{}'.format((angle_optical/np.pi)*180.0, distance_optical, \
                 (angle_gps/np.pi)*180.0, distance_gps, elapsed_time))

    # moniter the mission
    if nextwaypoint < len(drone.commands):
        if frame_num%20==0:
            display_seq = drone.commands.next
            print('heading:{} Angle:{} Distance:{} motor:{}'.format(drone.heading, 
                  (angle_gps/np.pi)*180.0, distance_gps, motor_gps))
            print "Moving to waypoint %s" % display_seq
            nextwaypoint = drone.commands.next
    else:
        break

    if elapsed_time > 0.1:
        print('Elapsed time:%.5f!!!!!!!!!!!'%elapsed_time)

    time.sleep(0.05)

print "\n\nMission ended or stoppped. The final results of CX model based on optcial flow is:"
print(' Angle_optical:{}\n Distance_optical:{}\n Angle_gps:{}\n Distance_gps:{}\n elapsed_time:{}' \
      .format((angle_optical/np.pi)*180.0, distance_optical, \
      (angle_gps/np.pi)*180.0, distance_gps, elapsed_time))
# land for measure distance
drone.mode = VehicleMode("LAND")
print "Landing, wait for GUided mode"
time.sleep(10)
# wait until GUIDED mode is set
while drone.mode.name != "GUIDED":
    print "Waiting for the GUIDED mode."
    time.sleep(2)
state = arm_and_takeoff(drone, HEIGHT)

# -------------------------------------homing-----------------------------------------------
# ------------------stop when the same period of time reached-------------------------------
#-------------------------------------------------------------------------------------------
# rotate to return direction first
send_ned_velocity(drone, 0, 0, 0, 1)
while drone.mode.name == "GUIDED":
    drone_heading = drone.heading/180.0*np.pi
    velocity = np.array([0, 0]) / 4.0 # normarlization
    __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)
    heading = motor_gps*100.0 / scale
    heading = np.min([np.max([-10,heading]), 10])
    print heading
    if np.abs(heading) > 1.0:
        print "rotating"
        condition_yaw(drone, heading, relative=True)
    else:
        break;
    time.sleep(0.5)

start_time = time.time()
while drone.mode.name == "GUIDED":

    velocity = drone.velocity
    drone_heading = drone.heading/180.0*np.pi

    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity = np.array([left_real, right_real]) / 4.0 # normarlization
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)
    
    frame_num -= 1
    

    if (frame_num) % 10==0:
        heading = motor_gps*100.0 / scale
        if heading>2:
            heading = heading*8.0 + 1
        heading = np.min([np.max([-15,heading]), 15])
        #navigation_heading += heading
        if np.abs(heading) > 0.5:
            print "rotating, ", heading
            condition_yaw(drone, heading, relative=True)
    if (frame_num+1) % 10 == 0:
       send_ned_velocity(drone, 1.5*np.cos(drone_heading), 1.5*np.sin(drone_heading), 0, 1)

    elapsed_time = time.time() - start_time
    start_time = time.time()
    # logging
    logging.info('sl:{} sr:{} heading:{} velocity:{} position:{}'.format(
                sl,sr,drone.heading,drone.velocity, drone.location.global_relative_frame))
    angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps)
    logging.info('Angle_optical:{} Distance_optical:{} Angle_gps:{} Distance_gps:{} \
                 elapsed_time:{}'.format((angle_optical/np.pi)*180.0, distance_optical, \
                 (angle_gps/np.pi)*180.0, distance_gps, elapsed_time))

    # show data for debugging
    if frame_num % 20==0:
        angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps) 
        print('heading:{} Angle:{} Distance:{} motor:{}'.format(drone_heading, 
              (angle_gps/np.pi)*180.0, distance_gps, motor_gps))

    if elapsed_time > 0.1:
        print('Elapsed time:%.5f !!!!!!!!!!!!!'%elapsed_time)

    time.sleep(0.05)

print "\n\nMission ended or stoppped. The final results of CX model based on optcial flow is:"
print(' Angle_optical:{}\n Distance_optical:{}\n Angle_gps:{}\n Distance_gps:{}\n elapsed_time:{}' \
      .format((angle_optical/np.pi)*180.0, distance_optical, \
      (angle_gps/np.pi)*180.0, distance_gps, elapsed_time))
drone.close()
cv2.destroyAllWindows()
