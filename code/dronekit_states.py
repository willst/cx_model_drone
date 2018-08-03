import dronekit
import socket
import exceptions
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from CX_model.drone_basic import arm, arm_and_takeoff, download_mission, adds_square_mission, PX4setMode
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
print " Armed: %s" % vehicle.armed    # settable

if not vehicle:
   raise Exception('Fail to connect Pixhawk!')

# show mission information
cmds = download_mission(vehicle.commands)
for cmd in cmds:
    print cmd.x, cmd.y, cmd.z

for i in range(100):

    time.sleep(1)
    print ("Range finder value:", vehicle.rangefinder.distance)
    time.sleep(0.25)

# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)

