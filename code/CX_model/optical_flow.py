import numpy as np
import cv2
from numpy import linalg as LA
FRAME_DIM = {'small':(216, 162), 'medium':(320, 240), \
             'large':(656, 496), 'origin':(1296, 976)}
#K=np.array([[1440.318444287085, 0.0, 676.9511026584912], 
#           [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
#D=np.array([[-0.8909302058344544], [3.1817023042732813], 
#            [-12.598093051063067], [17.641313727690882]])

#K=np.array([[649.7237194130138, 0.0, 570.0013929289199], 
#           [0.0, 627.6183259974277, 532.3632845985546], [0.0, 0.0, 1.0]])
#D=np.array([[-0.1428222048947462], [0.22455805794512237], 
#           [-0.2695633377157125], [0.1381009248014135]])
K=np.array([[743.8040173335771, 0.0, 647.9940524434143], 
           [0.0, 728.8909350192504, 485.7950206412609], [0.0, 0.0, 1.0]])
D=np.array([[-0.20926662485054526], [-0.04800755535197234], 
           [0.26419146114701453], [-0.1540385750579161]])
DIM=(1296, 972)
UNDISTORTED_DIM = (200, 108)

class Optical_flow():

    def __init__(self, dim):
        self.speed_left_buffer = np.array([0, 0, 0, 0], dtype=float)
        self.speed_right_buffer = np.array([0, 0, 0, 0], dtype=float)
        self.accmax = 0.06 * (dim[0]/324.0)

        if dim[0]>1000:
            self.angle_range = (42.0,24.0)
        elif dim[0]>600:
            self.angle_range = (70.0,46.0)
        elif dim[0]>300:
            self.angle_range = (120.0,74.0)
        elif dim[0]>200:
            self.angle_range = (130.0,84.0)


        assert dim[0]/dim[1] == DIM[0]/DIM[1], \
        "Image to undistort needs to have same aspect ratio as the ones used in calibration"

        self.scaled_K = K * dim[0] / DIM[0]  # The values of K is to scale with image dimension.
        self.scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
        # Compute undistort map matrixes
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                   self.scaled_K, D, dim, np.eye(3), balance=1.0)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                   self.scaled_K, D, np.eye(3), self.new_K, dim, cv2.CV_16SC2)

        

    def undistort(self, img):
        ''' undistort and crop frames from the fisheye image 
            for resolution small, angle range [-65,65,-40:40]
            for resolution medium, angle range [-60:60, -37:37]
            for resolution large, angle range [-35:35, -23:23]
            for resolution origin, angle range [-21:21, -12:12] 
        '''
        dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
        assert dim1[0]/dim1[1] == DIM[0]/DIM[1], \
               "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        undistorted_img = cv2.remap(img, self.map1, self.map2, interpolation= \
                                    cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
         
        # crop images 
        fh_s = int(0.2377*dim1[1])
        fh_e = int(0.7377*dim1[1])
        fw_s = int(0.15*dim1[0])
        fw_e = int(0.85*dim1[0])
        undistorted_img = undistorted_img[fh_s:fh_e, fw_s:fw_e] 
        
        # to improve the sampling rates (FPS), the image patch has to be small (200,126)
        (height, width) = undistorted_img.shape
        w_half = UNDISTORTED_DIM[0]/2
        h_half = UNDISTORTED_DIM[1]/2
        if dim1[0] > 216:
            undistorted_img = undistorted_img[(height/2-h_half):(height/2+h_half), \
                                              (width/2-w_half):(width/2+w_half)]
        
        return undistorted_img


    def get_filter(self, fh, fw):
        ''' Generate match filter for optical flow computation, one for left 45 degree 
            one for right 45 degree 
        '''
        # filter for speed retrieval
        vertical_views = (np.arange(fh, dtype=float)-fh/2)/fh*(self.angle_range[1]/180.0*np.pi)
        horizontal_views = (np.arange(fw, dtype=float)-fw/2)/fw*(self.angle_range[0]/180.0*np.pi)
        D = np.ones([fh,fw,3])*-1
        D[:,:,1] = np.tan(vertical_views).reshape(fh, 1)
        D[:,:,0] = np.tan(horizontal_views)
        sin_theta = LA.norm(D[:,:,0:2], axis = 2) + 0.0000001
        mag_temp = LA.norm(D, axis = 2) + 0.0000001
        D /= mag_temp.reshape(fh,fw,1)
        a_l = np.array([1/np.sqrt(2), -1/np.sqrt(2), 0])
        a_r = np.array([1/np.sqrt(2), 1/np.sqrt(2), 0])
        left_filter = np.cross(np.cross(D,a_l),D)[:,:,0:2] / sin_theta.reshape(fh,fw,1)
        right_filter = np.cross(np.cross(D,a_r),D)[:,:,0:2] / sin_theta.reshape(fh,fw,1)
        return left_filter, right_filter


    def cart2pol(self, x, y):
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return(rho, phi)


    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)


    def get_speed(self, flow, left_filter, right_filter, elapsed_time):
        ''' calculate speeds from optical flow using match filters
        '''    
        mag = LA.norm(flow/left_filter, axis=2)
    
        mag[mag < 3.0] = 0  # filter out those noisy flow
        mag[mag > 0.0] = 1.0
        count = np.sum(mag)
    
        #print count
        weight = mag/(elapsed_time*count*40+1)
        weight = weight.reshape(weight.shape[0], weight.shape[1], 1)  # reshape for broadcasting
        self.speed_left_buffer = np.roll(self.speed_left_buffer, 1)
        self.speed_left_buffer[0] = np.sum(flow * left_filter * weight)
        self.speed_right_buffer = np.roll(self.speed_right_buffer, 1)
        self.speed_right_buffer[0] = np.sum(flow * right_filter * weight)
        sl = np.mean(self.speed_left_buffer)
        sl = np.max([np.min([self.speed_left_buffer[1]+self.accmax, sl]), self.speed_left_buffer[1]-self.accmax])
        sr = np.mean(self.speed_right_buffer)
        sr = np.max([np.min([self.speed_right_buffer[1]+self.accmax, sr]), self.speed_right_buffer[1]-self.accmax])
        self.speed_left_buffer[0] = sl
        self.speed_right_buffer[0] = sr

        return sl, sr
    
