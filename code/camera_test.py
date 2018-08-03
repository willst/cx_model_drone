
import sys
import numpy as np
import cv2
import os, time
from CX_model.optical_flow import Optical_flow, FRAME_DIM
from CX_model.video_threading import picameraThread

resolution = FRAME_DIM['medium']
fw, fh = resolution
scale =  2

'''
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,fw)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,fh)
cap.set(cv2.CAP_PROP_FPS, 30)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))
'''

picam = picameraThread(1, "picamera_video", resolution, 30)
picam.start()
optflow = Optical_flow(resolution);
picture_num = 100
column_num = 0
time.sleep(2)
while True:

    gray = picam.get_frame()
    # undistorte image
    gray = optflow.undistort(gray)

    # draw lines on image for calibration angles
    gray = cv2.line(gray,(1, column_num),(fw, column_num),(255,255,0),1)
    gray = cv2.line(gray,(1, int(fh/2)),(fw, int(fh/2)),(255,255,0),1)
    gray = cv2.line(gray,(column_num, 1),(column_num, fh),(255,255,0),1)
    gray = cv2.line(gray,(int(fw/2), 1),(int(fw/2), fh),(255,255,0),1)

    cv2.imshow('frame', cv2.resize(gray, (0, 0), fx=scale, fy=scale))
    ch = 0xFF & cv2.waitKey(1)
    if ch == ord('q'):
        break
    elif ch == ord('s'):
        picture_name = "image" + str(picture_num) + '.jpg'
        cv2.imwrite(picture_name,gray)
        picture_num = picture_num + 1
        print('Save frame: {}'.format(picture_name))
    elif ch == ord('='):
        column_num = (column_num + 1) % fw
        print('Now the column is: {}'.format(column_num))
    elif ch == ord('-'):
        column_num = (column_num - 1) % fw
        print('Now the column is: {}'.format(column_num))

# Release everything if job is finished
picam.stop()
cv2.destroyAllWindows()
