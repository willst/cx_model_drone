import threading
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

class videoThread (threading.Thread):
   
   def __init__(self, threadID, name, fw, fh, fps):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.cap = cv2.VideoCapture(0)
      self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, fw)
      self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, fh)
      self.cap.set(cv2.CAP_PROP_FPS, fps)
      ret, frame = self.cap.read()
      self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      self.stop_flag = 0

   def get_frame(self):
      return self.frame

   def stop(self):
      self.stop_flag = 1

   def run(self):
      while not self.stop_flag:
          ret, frame = self.cap.read()
          if ret:
              self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
          else:
              break
      self.cap.release()


class picameraThread(threading.Thread):
   
   def __init__(self, threadID, name, resolution, fps):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.camera = PiCamera()
      self.camera.resolution = resolution
      self.camera.framerate = fps
      self.rawCapture = PiRGBArray(self.camera, size=resolution)
      
      self.camera.capture(self.rawCapture, format="bgr")
      frame_temp = self.rawCapture.array
      self.rawCapture.truncate(0)    # clear the stream in preparation for the next frame
      self.frame = cv2.cvtColor(frame_temp, cv2.COLOR_BGR2GRAY)
      self.stop_flag = 0

   def get_frame(self):
      return self.frame

   def get_resolution(self):
      return self.camera.resolution

   def stop(self):
      self.stop_flag = 1

   def run(self):
      for image in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
          if self.stop_flag:
              break
          frame_temp = image.array
          self.frame = cv2.cvtColor(frame_temp,cv2.COLOR_BGR2GRAY)
          self.rawCapture.truncate(0)

      self.camera.close()


