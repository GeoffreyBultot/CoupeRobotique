import io
import socket
import struct
import time
import picamera
from threading import Condition
import argparse
import logging
import cv2
import numpy as np





class StreamingOutput(object):
	def __init__(self):
		self.frame = None
		self.buffer = io.BytesIO()
		self.image = None
		self.t1 = 0
		self.t2 = 0
	def write(self, buf):
		
		self.buffer.truncate()
		#self.frame = self.buffer.getvalue()
		#print(buf)
		#print(self.frame)
		if buf :#buf.startswith(b'\xff\xd8'):
			array = np.asarray(bytearray(buf), dtype=np.uint8)
			#self.image = cv2.imdecode(array, cv2.IMREAD_COLOR)
			self.image = cv2.imread( array)

			cv2.imshow("image",array)
			if cv2.waitKey(3) & 0xFF == ord('q'):
				return
			self.buffer.seek(0)
			print(1/(time.time()-self.t1))
			self.t1 = time.time()
			
		return self.buffer.write(buf)

while True:
	with picamera.PiCamera(resolution='1280x1080', framerate=24) as camera:
		output = StreamingOutput()
		#Uncomment the next line to change your Pi's Camera rotation (in degrees)
		#camera.rotation = 90
		print("start")
		camera.start_recording(output, format='bgr')
		while(True):
			if(False):#output.image is not None):
				cv2.imshow("image",output.image)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break
			pass
		print("started")
		camera.stop_recording()
		#print("stoped")