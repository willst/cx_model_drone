import sys
import numpy as np
import cv2
import os

cap = cv2.VideoCapture(sys.argv[1])
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(sys.argv[2],fourcc, 20.0, (fw,fh))

frame_num = 0

# skip frames
for i in range(350):
    ret, frame = cap.read()
    frame_num += 1

if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        if ret==True:
            frame_num += 1
            print('Frame number:', frame_num)
            cv2.imshow('vedio', frame)
            if cv2.waitKey(50) & 0xFF == ord('q') or frame_num==430:
                break
            # write the flipped frame
            out.write(frame)
        else:
            break
else:
    print('No camera!')
# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
