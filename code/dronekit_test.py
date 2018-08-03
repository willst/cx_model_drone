import numpy as np
import dronekit 
import socket 
import exceptions 
import time 
import cv2 
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command 
from CX_model.drone_basic import arm, arm_and_takeoff, download_mission, adds_square_mission, PX4setMode, adds_Lshape_mission 
from pymavlink import mavutil

connection_string = "127.0.0.1:14550"
#connection_string = '/dev/ttyAMA0'

# Try to connect to PX4
try:
    vehicle = dronekit.connect(connection_string)
# Bad TCP connection
except socket.error:
    print 'No server exists!'
# Bad TTY connection
except exceptions.OSError as e:
    print 'No serial exists!'
# API Error
except dronekit.APIException:
    print 'Timeout!'
# Other error
except:
    print 'Some other error!'

# Get all vehicle attributes (state)
'''print "\nGet all vehicle attribute values:"
print "   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target
print "   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned
print "   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int
'''
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle.ekf_ok
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Heading: %s, Heading in radiance: %s" % (vehicle.heading, vehicle.heading/180.0*np.pi)
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable

char = raw_input("Check the status, press anykey to continue, \'q\' to quit")
if char == 'q':
    raise Exception('Mission cancelled!')
else:
    print 'Mission start.'

if vehicle:
    # old mission
    cmds = download_mission(vehicle.commands)
    print ('Waypoint numbers: ', cmds.count)
    print('Next ID:', cmds.next)
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
        print(cmd.x, cmd.y, cmd.z)
        print "Command ID: ", cmd.command
    
    time.sleep(10)
    # modify mission
    cmd = missionlist[1]
    startlocation=LocationGlobalRelative(cmd.x, cmd.y,cmd.z)
    adds_Lshape_mission(vehicle, startlocation, 20, 5)
    time.sleep(3)

    # new mission
#    cmds = download_mission(vehicle.commands)
    print ('Waypoint numbers: ', cmds.count)
    for cmd in cmds:
        print(cmd.x, cmd.y, cmd.z)
    vehicle.close()
