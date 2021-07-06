#!/usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
import rospy
import math
import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# parser = argparse.ArgumentParser()
# parser.add_argument('--connect', default='udp:127.0.0.1:14550')
# args = parser.parse_args()

# # Connect to the Vehicle
# print 'Connecting to vehicle on: %s' % args.connect
# vehicle = connect(args.connect, baud=921600, wait_ready=True)

# # Function to arm and then takeoff to a user specified altitude
# def arm_and_takeoff(aTargetAltitude):

# 	print "Basic pre-arm checks"
# 	# Don't let the user try to arm until autopilot is ready
# 	while not vehicle.is_armable:
# 		print " Waiting for vehicle to initialise..."
# 		time.sleep(1)

# 	print "Arming motors"
# 	# Copter should arm in GUIDED mode
# 	vehicle.mode    = VehicleMode("GUIDED")
# 	vehicle.armed   = True

# 	while not vehicle.armed:
# 		print " Waiting for arming..."
# 		time.sleep(1)

# 	print "Taking off!"
# 	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

# 	  # Check that vehicle has reached takeoff altitude
# 	while True:
# 		print " Altitude: ", vehicle.location.global_relative_frame.alt 
# 		#Break and return from function just below target altitude.        
# 		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
# 			print "Reached target altitude"
# 			break
# 		time.sleep(1)
class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test', anonymous=True, disable_signals=False) #Initialise rosnode
		rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
		self.x_err_pub = rospy.Publisher('/err_x_val',Float32,queue_size = 10)
		self.y_err_pub = rospy.Publisher('/err_y_val',Float32,queue_size = 10)
		self.bool_pub = rospy.Publisher('/bool_pub',Float32,queue_size = 10)
		self.image = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.land = False
		self.recieved = False
		img_width = 400
		hfov_rad = 1.04  #horizontal field of view
		self.focal_length = (img_width/2)/math.tan(hfov_rad/2)
		self.x_err = 0
		self.y_err = 0
		self.check_detect = 0

	def image_callback(self, data):
		print("callback")
		try:
			self.image = self.bridge.imgmsg_to_cv2(data,"bgr8") # Converting the image to OpenCV standard image
			self.recieved = True
		except CvBridgeError as e:
			print(e)
			return

	def detect_marker(self):
		
		
		# Our operations on the frame come here
		if self.recieved:
			image=self.image
			image=cv2.resize(image,(400,400))
			[r,c,h]=image.shape
			print(r,c,h)
			print(image.dtype)
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

			aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
			parameters =  aruco.DetectorParameters_create()
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
			#frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
			# verify *at least* one ArUco marker was detected
			if len(corners) > 0:
				# flatten the ArUco IDs list
				ids = ids.flatten()
				# loop over the detected ArUCo corners
				for (markerCorner, markerID) in zip(corners, ids):
					# extract the marker corners (which are always returned in
					# top-left, top-right, bottom-right, and bottom-left order)
					corners = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corners
					# convert each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					topLeft = (int(topLeft[0]), int(topLeft[1]))
					# draw the bounding box of the ArUCo detection
					cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
					cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
					cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
					cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
					# compute and draw the center (x, y)-coordinates of the ArUco
					# marker
					# cX = int((topLeft[0] + bottomRight[0]) / 2.0)
					# cY = int((topLeft[1] + bottomRight[1]) / 2.0)
					# print(cX,cY)
					# err_x_m = ((cX-200)*(5.0)/self.focal_length) #calculating the x_error from drone to marker
					# err_y_m = ((cY-200)*(5.0)/self.focal_length) #calculating the y_error from drone to marker
					# print(err_x_m,err_y_m)
					# cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
					# draw the ArUco marker ID on the image
					cv2.putText(image, str(markerID),
						(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (0, 255, 0), 2)
					print("[INFO] ArUco marker ID: {}".format(markerID))
					if (int(markerID)==0):
						cX = int((topLeft[0] + bottomRight[0]) / 2.0)
						cY = int((topLeft[1] + bottomRight[1]) / 2.0)
						print(cX,cY)
						err_x_m = ((cX-200)*(3.0)/self.focal_length) #calculating the x_error from drone to marker
						err_y_m = ((cY-200)*(3.0)/self.focal_length) #calculating the y_error from drone to marker
						print(err_x_m,err_y_m)
						cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
						self.x_err_pub.publish(err_x_m)
						self.y_err_pub.publish(err_y_m)
						self.bool_pub.publish(1)
						# print("Now let's land")
						# vehicle.mode = VehicleMode("LAND")
						
			cv2.imshow("Image", image)

			#cv2.imshow('frame_marker', frame_markers)#display the detected image
			cv2.waitKey(2)#wait for 2 millisecond before closing the output

if __name__ == '__main__':
	time.sleep(1)
	print('detector starts')
	# arm_and_takeoff(5)
	# print("Set default/target airspeed to 3")
	# vehicle.airspeed = 3
	# print("Going towards first point for 30 seconds ...")
	# point1 = LocationGlobalRelative(-35.3632585, 149.16532155650,5)
	# vehicle.simple_goto(point1)
	# print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
	# point2 = LocationGlobalRelative(-35.3632585, 149.16532155650,5)
	# vehicle.simple_goto(point2)
	img = image_proc()
	r = rospy.Rate(10.0) # Rate at which the node runs
	while not rospy.is_shutdown():
		img.detect_marker()
		r.sleep()
		if img.land:
			break
