import numpy as np
import cv2
import sys, os, time
import dronekit
import matplotlib
import matplotlib.pyplot as plt
from CX_model import cx_rate, central_complex
from CX_model.optical_flow import Optical_flow, FRAME_DIM
from CX_model.central_complex import update_cells
from CX_model.drone_basic import arm, arm_and_takeoff
from CX_model.graphics import draw_flow

# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

# initialize camera
frame_num = 0 
cap = cv2.VideoCapture(sys.argv[1])
for i in range(1):              
    ret, frame1 = cap.read()      # Skip frames
    frame_num += 1
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
dim = temp.shape[::-1]

# intialise optical flow object
optflow = Optical_flow((312,286));
prvs = temp #optflow.undistort(temp)
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))

# log information
str = sys.argv[1]
try:
    error_log_path = 'log/'+str.split('.')[0].split('/')[1]+'.log'
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
    time_list = []
    for i in range(len(model_info)):
        elapsed_time = model_info[i].split('elapsed_time:')[-1]
        time_list.append(float(elapsed_time))
    data_length = len(model_info)
except:
    print "Fail to find log file"
    data_length = 200
    time_list=np.ones(data_length, dtype=float)*0.1

angle_list = np.zeros(data_length, dtype=float)
distance_list = np.zeros(data_length, dtype=float)

# visulize computed speed 
plt.ion()
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharey=False)
ax1.set(title='speed', ylabel='left')
ax2.set(ylabel='right')
ax3.set(xlabel='time (s)', ylabel='compare')
ax4.set(xlabel='group', ylabel='number')
plt.show()
x_axis = np.linspace(0, data_length, num=data_length, endpoint=False)
index = np.arange(36)


speed_left = np.zeros(data_length)
speed_right = np.zeros(data_length)
speed_rotation = np.zeros(data_length)
optical_direction = np.zeros(36)

left_filter, right_filter = optflow.get_filter(fh, fw)

while True:    
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    if not ret:
        break
    frame_num += 1    
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = temp #optflow.undistort(temp)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    
    # speed
    try:
        sl, sr = optflow.get_speed(flow, left_filter, right_filter, time_list[frame_num])
    except:
        print "Optical flow speed retrieval failed"
        break
    '''
    # optical flow heading
    __, phi = optflow.cart2pol(flow[:,:,0], flow[:,:,1])
    phi = phi/np.pi*180.0
    deg = np.ceil(phi/10).flatten()
    deg = np.sort(np.append(deg, np.arange(-17,19)))  # add elements to make sure every group have at least one element
    _shift = np.roll(deg, -1)
    count_deg = np.array(np.nonzero(_shift - deg)).flatten()
    _shift = np.roll(count_deg, 1)
    _shift[0] = 0
    count_deg = count_deg - _shift
    optical_direction = np.array(count_deg).flatten()
    '''
    # visulize computed speed 
    speed_left = np.roll(speed_left, -1)
    speed_left[-1] = sl 
    speed_right = np.roll(speed_right, -1)
    speed_right[-1] = sr

    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()
    ax1.plot(x_axis, speed_left, 'r-')
    ax2.plot(x_axis, speed_right, 'b-')
    ax3.plot(x_axis, speed_left, 'r-')
    ax3.plot(x_axis, speed_right, 'b-')
    #ax4.bar(index, optical_direction, color='r', label='degree')
    plt.draw()

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=np.pi, velocity=velocity, tb1=tb1, memory=memory, cx=cx)
    angle, distance = cx.decode_cpu4(cpu4)
    angle_list[frame_num] = angle/np.pi*180.0
    distance_list[frame_num] = distance

    # show frames
    cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=3.0, fy=3.0))
    #print('Frame number: ', frame_num)
    ch = cv2.waitKey(5) & 0xFF 
    if ch == ord('q'):
        break
    elif ch == ord(' '):
        while True:
            ch = cv2.waitKey(5) & 0xFF
            if ch == ord(' ') or ch == ord('q'):
                break
            time.sleep(0.2)
    prvs = next
    start_time = time.time()

fig2, (ax6, ax5) = plt.subplots(2, sharey=False)
ax6.plot(x_axis, angle_list, 'b-')
ax5.plot(x_axis, distance_list, 'b-')
plt.show()

cv2.waitKey()
cap.release()
cv2.destroyAllWindows()





