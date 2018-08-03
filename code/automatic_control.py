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
from CX_model.drone_ardupilot import arm, arm_and_takeoff
from picamera.array import PiRGBArray
from picamera import PiCamera
from CX_model.video_threading import picameraThread

resolution = FRAME_DIM['medium']
print "Resolution: ", resolution

# command line arguments halder
parser = argparse.ArgumentParser(description='CX model navigation.')
parser.add_argument('--recording', default = 'no', 
                    help='Recoding option, true or false(default: false)')

args = parser.parse_args()
RECORDING = args.recording

# initialize logger
time_string = str(datetime.datetime.now()).replace(':', '-').replace(' ', '_').split('.')[0]
fname = 'log/' + time_string + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)
logging.info("Resolotion:{},{}".format(resolution[0], resolution[1]))

# initialize CX models
cx_optical = cx_rate.CXRate(noise = 0)
tb1_optical = np.zeros(central_complex.N_TB1)
memory_optical = 0.5 * np.ones(central_complex.N_CPU4)

cx_gps = cx_rate.CXRate(noise = 0)
tb1_gps = np.zeros(central_complex.N_TB1)
memory_gps = 0.5 * np.ones(central_complex.N_CPU4)
cpu4_gps = np.zeros(16)

# initialize camera and optical flow
frame_num = 0
picam = picameraThread(1, "picamera_video", resolution, 30)
picam.start()
fw,fh = resolution
# allow the camera to warmup
time.sleep(0.1)
print("Frame size: {}*{}".format(fw, fh))

# intialise optical flow object
optflow = Optical_flow(resolution)
temp = picam.get_frame()
prvs = optflow.undistort(temp)
(fh, fw) = prvs.shape
print("Undistorted frame size: {0}*{1}".format(fw,fh))
left_filter, right_filter = optflow.get_filter(fh, fw)

# Define the codec and create VideoWriter object
if RECORDING == 'true':
    fname = 'video/' + time_string + '.avi'
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(fname,fourcc, 20.0, (fw,fh), False)

# connect to PX4, arm and takeoff
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 921600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')
state = arm_and_takeoff(drone, 2.5)

# set to mission mode.
drone.mode = VehicleMode("AUTO")
while drone.mode.name != "AUTO":
    print "Waiting for the mission mode."
    time.sleep(2)
# wait until reach first waypoint, 1->home, 2->takeoff
nextwaypoint = drone.commands.next
while nextwaypoint < 3:
    print "Initialisation, Moving to waypoint", drone.commands.next+1
    nextwaypoint = drone.commands.next
    time.sleep(1)


start_time = time.time()
print "Start to update CX model, switch mode to end"
while drone.mode.name == "AUTO":

    # Image processing, compute optical flow
    frame_num += 1
    frame_gray = picam.get_frame()
    next = optflow.undistort(frame_gray)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    # speed
    elapsed_time = time.time() - start_time
    start_time = time.time()
    sl, sr = optflow.get_speed(flow, left_filter, right_filter, elapsed_time)

    # update CX neurons
    drone_heading = drone.heading/180.0*np.pi
    velocity = np.array([sl, sr])
    __, __, tb1_optical, __, __, memory_optical, cpu4_optical, __, motor_optical = \
            update_cells(heading=drone_heading, velocity=velocity, tb1=tb1_optical, \
                         memory=memory_optical, cx=cx_optical)

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

    # write the frame for later recheck
    if RECORDING == 'true':
        out.write(next)

    # logging
    logging.info('sl:{} sr:{} heading:{} velocity:{} position:{}'.format(
                sl,sr,drone.heading,drone.velocity, drone.location.global_relative_frame))
    angle_optical, distance_optical = cx_optical.decode_cpu4(cpu4_optical)
    angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps)
    logging.info('Angle_optical:{} Distance_optical:{} Angle_gps:{} Distance_gps:{} \
                 elapsed_time:{}'.format((angle_optical/np.pi)*180.0, distance_optical, \
                 (angle_gps/np.pi)*180.0, distance_gps, elapsed_time))

    # moniter the mission
    if frame_num%10==0:
        display_seq = drone.commands.next+1
        #print "Moving to waypoint: ", display_seq
        nextwaypoint = drone.commands.next

    prvs = next
    if elapsed_time > 0.1:
        print('Elapsed time:%.5f'%elapsed_time)

print "Mission ended or stoppped. The final results of CX model based on optcial flow is:"
print((angle_optical/np.pi) * 180, distance_optical)
drone.close()
if RECORDING == 'true':
    out.release()
picam.stop()
cv2.destroyAllWindows()
