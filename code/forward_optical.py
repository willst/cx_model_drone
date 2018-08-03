import numpy as np
import cv2
import central_complex
import cx_rate
import cx_basic
import time
import sys
import dronekit
import os
import camera_calibration
import matplotlib.pyplot as plt
import matplotlib.animation as animation

home = os.environ['HOME']
if home.split('/')[-1] == 'pi':
    show_frames = False
else:
    show_frames = True

def draw_flow(img, flow, step=10):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T *3
    fy[:] = 0
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    #print(flow.shape)
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    tn1 = cx.tn1_output(velocity)
    tn2 = cx.tn2_output(velocity)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

# connect to PX4
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 57600, heartbeat_timeout=15)
# API Error
except dronekit.APIException:
    print 'Timeout!'
# Other error
except:
    print 'Some other error!'


# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)



# initialize camera
frame_num = 0 
cap = cv2.VideoCapture(sys.argv[1])
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,200)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,130)
#fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

for i in range(200):              
    ret, frame1 = cap.read()      # Skip frames
    frame_num += 1
#frame1 = cv2.resize(frame1, (0,0), fx=0.25, fy=0.25)
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = camera_calibration.undistort(temp, 1.0)
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))

# visulize computed speed 
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='time (s)', ylabel='right')
x_axis = np.linspace(0, 100, num=100, endpoint=False)
speed_left = np.zeros(100)
speed_right = np.zeros(100)
plt.show()

# filter for speed retrieval
row = np.linspace(0, fw, num=fw, endpoint=False)
match_filter = np.sin((row/fw -0.5)*0.822*np.pi)

start_time = time.time()
while(1):    
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    frame_num += 1
    #frame2 = cv2.resize(frame2, (0,0), fx=0.25, fy=0.25)
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = camera_calibration.undistort(temp, 1.0)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    hori_flow = flow[:,:,0]
    
    left_frame_shift = fw/4
    frame_left = np.roll(hori_flow, -left_frame_shift, axis=1)
    frame_left[:,fw-left_frame_shift:fw-1] = 0
    right_frame_shift = fw/4
    frame_right = np.roll(hori_flow, right_frame_shift, axis=1)
    frame_right[:,0:right_frame_shift-1] = 0
    elapsed_time = time.time() - start_time
    # left speed
    mag = np.abs(frame_left/match_filter)
    mag[mag < 0.5] = 0
    mag[mag > 0.0] = 1.0
    count = np.sum(mag)
    weight = mag/((0.034+elapsed_time)*count*100+1)
    sl = np.sum(frame_left * (match_filter)*weight)
    # right speed
    mag = np.abs(frame_right/match_filter)
    mag[mag < 0.5] = 0
    mag[mag > 0.0] = 1.0
    count = np.sum(mag)
    weight = mag/((0.034+elapsed_time)*count*100+1)
    sr = np.sum(frame_right * (match_filter)*weight)

    # visulize computed speed 
    speed_left = np.roll(speed_left, 1)
    speed_left[0] = (sl) # + np.sum(speed_left[1:3]))/4
    speed_right = np.roll(speed_right, 1)
    speed_right[0] = (sr) # + np.sum(speed_right[1:3]))/4
    ax1.clear()
    ax2.clear()
    ax1.plot(x_axis, speed_left, 'r-')
    ax1.plot(x_axis, speed_right, 'k-')
    plt.draw()

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=0, velocity=velocity, tb1=tb1, memory=memory, cx=cx)
    angle, distance = cx.decode_cpu4(cpu4)
    #print((angle/np.pi) * 180, distance)

    # show frames
    print('Frame number:', frame_num)
    cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=2, fy=2))
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
    prvs = next
    start_time = time.time()

cap.release()
cv2.destroyAllWindows()





