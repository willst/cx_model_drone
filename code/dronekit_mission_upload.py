import dronekit
import socket
import exceptions
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from CX_model.drone_ardupilot import arm, arm_and_takeoff, download_mission, get_angles_degree, \
     get_location_metres, arm_and_takeoff, condition_yaw, send_ned_velocity, save_mission, readmission, upload_mission
from pymavlink import mavutil

connection_string = "127.0.0.1:14550"
#connection_string = '/dev/ttyAMA0'

# Try to connect to PX4
try:
    vehicle = dronekit.connect(connection_string, baud=921600, wait_ready=True)
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
print "\nGet all vehicle attribute values:"

print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " Battery: %s" % vehicle.battery
print " Heading: %s" % vehicle.heading
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s\n\n" % vehicle.armed    # settable


vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

while vehicle.mode.name != "GUIDED":
    print "Failed to enter GUIDED mode"
    time.sleep(2)

if vehicle:
    upload_mission(vehicle, "sample_mission.txt")
    time.sleep(2)
    home=vehicle.home_location
    # Load commands
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    cmd = cmds[3]
    first_waypoint = LocationGlobalRelative(cmd.x, cmd.y, cmd.z)
    arm_and_takeoff(vehicle, 4)
    send_ned_velocity(vehicle, 0, 0, 0, 1)
    condition_yaw(vehicle, get_angles_degree(home,first_waypoint), 1)
    print get_angles_degree(home,first_waypoint)
    time.sleep(5)

    vehicle.mode = VehicleMode("AUTO")
    # monitor mission execution
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands):
        if vehicle.commands.next > nextwaypoint:
            display_seq = vehicle.commands.next
            print "Moving to waypoint %s" % display_seq
            nextwaypoint = vehicle.commands.next
        time.sleep(1)



    print 'Return to launch'
    vehicle.mode = VehicleMode("RTL")
    time.sleep(2)

    save_mission(vehicle, 'sample_mission.txt')
    


    #Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()

