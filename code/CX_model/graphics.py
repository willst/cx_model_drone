import numpy as np
import cv2

def draw_flow(img, flow, step=10):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T *1
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (255, 255, 255))
    #print(flow.shape)
    #for (x1, y1), (x2, y2) in lines:
        #cv2.circle(vis, (x1, y1), 1, (255, 255, 255), -1)
    return vis

def frame_preprocess(img, scale=1.0, crop_size = [0.0, 0.2]):
    '''
    Scale and crop images, crop size is the propotion (fh_p, fw_p) of image
    that need to be cropped away (from the border).
    '''    
    img = cv2.resize(img, (0,0), fx=scale, fy=scale)
    dim = img.shape
    crop_width = int(dim[0]*crop_size[0])
    crop_height = int(dim[1]*crop_size[1])
    return img[crop_width:(dim[0]-crop_width), crop_height:(dim[1]-crop_height)]


