import matplotlib.pyplot as plt
import numpy as np
import cv2
from numpy import ma
from numpy import linalg as LA
from CX_model.optical_flow import Optical_flow, FRAME_DIM

def draw_flow(img, flow, step=10):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T *5
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = img
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    #print(flow.shape)
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def rotate_vector(vector, angle):
    vector = np.array([i-fh/2, j-fw/2])
    rho = LA.norm(vector)
    phi = np.arctan2(vector[0], vector[1])
    phi_l = (phi - np.pi/4)
    vector[1] = rho * np.cos(phi_l)     # x axis
    vector[0] = rho * np.sin(phi_l)     # y axis
    
'''
fh = 110*4
fw = 200*4

vertical_views = (np.arange(fh, dtype=float)-fh/2)/fh*(110.0/180.0*np.pi)
horizontal_views = (np.arange(fw, dtype=float)-fw/2)/fw*(130.0/180.0*np.pi)

D = np.ones([fh,fw,3])*-1
D[:,:,1] = np.tan(vertical_views).reshape(fh, 1)
D[:,:,0] = np.tan(horizontal_views)
#mag_temp = LA.norm(D[:,:,0:2], axis = 2) + 0.0000001
#normlizer = np.array([mag_temp, mag_temp, mag_temp])
#D /= normlizer
mag_temp = LA.norm(D, axis = 2) + 0.0000001
normlizer = mag_temp.reshape(fh,fw,1)
D /= normlizer

a = np.array([1/np.sqrt(2), -1/np.sqrt(2), 0])        
matched_filter = np.cross(np.cross(D,a),D)[:,:,0:2]
# show vector map
img = np.ones([fh,fw,3])
vector_map = draw_flow(img, matched_filter)
cv2.imshow('filter1', vector_map)

D = np.zeros([fh,fw,3])
for i in range(fh):
    for j in range(fw):
        D[i,j]=np.array([np.tan(horizontal_views[j]), np.tan(vertical_views[i]), -1])
        D[i,j] /= LA.norm(D[i,j])

#a = np.array([1/np.sqrt(2), 1/np.sqrt(2), 0]) 
a = np.array([0, 0, -1]) 
#matched_filter = np.cross(np.cross(D,a),D)[:,:,0:2]
matched_filter = np.cross(a, D, axisa=0, axisb=2, axisc=2)[:,:,0:2]
#matched_filter = np.ones([fh,fw,2]) * np.array([0,1])
# show vector map
img = np.ones([fh,fw,3])
vector_map = draw_flow(img, matched_filter)
cv2.imshow('filter2', vector_map)
'''
optflow = Optical_flow((324, 248));
left_filter, right_filter, rot_filter = optflow.get_filter(110, 200)
img = np.ones([110,200,3])
vector_map = draw_flow(img, left_filter)
cv2.imshow('filter_left', vector_map)
img = np.ones([110,200,3])
vector_map = draw_flow(img, rot_filter)
cv2.imshow('rotation_right', vector_map)
cv2.waitKey()
cv2.destroyAllWindows()

