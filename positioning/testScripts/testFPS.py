import io
import time
import picamera
import cv2
import numpy
from PIL import Image

def outputs():
	stream = io.BytesIO()
	for i in range(40):
		# This returns the stream for the camera to capture to
		yield stream
		# Once the capture is complete, the loop continues here
		# (read up on generator functions in Python to understand
		# the yield statement). Here you could do some processing
		# on the image...
		#stream.seek(0)
		img = Image.open(stream)
		mat_array = numpy.array(img)
		print(i)
		#cv2.imshow("testFPS",mat_array)
		
		# Finally, reset the stream for the next capture
		stream.seek(0)
		stream.truncate()

with picamera.PiCamera() as camera:
	camera.resolution = (2592, 1944) 
	camera.framerate = 30
	#time.sleep(2)
	start = time.time()
	camera.capture_sequence(outputs(), 'jpeg', use_video_port=True)
	finish = time.time()
	print('Captured 40 images at %.2ffps' % (40 / (finish - start)))