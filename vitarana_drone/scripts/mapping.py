#!/usr/bin/env python

'''**********************************
E-yantra
Theme: Vitran Drone
Task: task4
Purpose: Generate Occupancy Grid Map
Scale: 1 unit in map = 0.2 m
Team ID : 0583
Team name : !ABHIMANYU 
**********************************'''

# Importing the required libraries
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, String, Int8, Float64
import rospy
import numpy as np
import time
import tf
import math
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class occupancy_map():

	def __init__(self):

		rospy.init_node('mapping')
		self.belief_map = np.zeros((1000,1000))
		self.drone_location_matrix = [0, 0]
		self.drone_location_initial_gps = [0.0, 0.0, 0.0]
		self.drone_location_gps = [0.0, 0.0, 0.0]
		self.store_initial_gps = True
		self.rate = rospy.Rate(1)
		self.laser_negative_longitude = 0
		self.laser_negative_latitude = 0
		self.laser_positive_latitude = 0
		self.laser_positive_longitude = 0
		self.drone_yaw = 0
		rospy.Subscriber('/edrone/range_finder_top',
                         LaserScan, self.range_finder_callback)
		rospy.Subscriber('/edrone/yaw', Float64, self.yaw_callback)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		self.my_pub = rospy.Publisher('/edrone/my_pub', String, queue_size = 1)

	def yaw_callback(self,msg):
		self.drone_yaw = msg.data
		

	def range_finder_callback(self, msg):
        	if(self.drone_yaw < np.pi/4 and self.drone_yaw > -np.pi/4):
            		self.laser_negative_longitude, self.laser_positive_latitude, self.laser_positive_longitude, self.laser_negative_latitude, _ = msg.ranges

        	elif(self.drone_yaw < 3*np.pi/4 and self.drone_yaw > np.pi/4):
            		self.laser_negative_latitude, self.laser_negative_longitude, self.laser_positive_latitude, self.laser_positive_longitude, _ = msg.ranges

        	elif(self.drone_yaw > 3*np.pi/4 or self.drone_yaw < -3*np.pi/4):
            		self.laser_positive_longitude, self.laser_negative_latitude, self.laser_negative_longitude, self.laser_positive_latitude, _ = msg.ranges

        	elif(self.drone_yaw > -3*np.pi/4 and self.drone_yaw < -np.pi/4):
            		self.laser_positive_latitude, self.laser_positive_longitude, self.laser_negative_latitude, self.laser_negative_longitude, _ = msg.ranges
        

	def gps_callback(self, msg):

		#To store gps coordinates of initial location of drone
		if(self.store_initial_gps):
			self.drone_location_initial_gps[0] = msg.latitude
			self.drone_location_initial_gps[1] = msg.longitude
			self.drone_location_initial_gps[2] = msg.altitude
			self.store_initial_gps = False
		self.drone_location_gps[0] = msg.latitude
		self.drone_location_gps[1] = msg.longitude
		self.drone_location_gps[2] = msg.altitude

		#0.000013552 lat = 1.5m
		#0.000014245 long = 1.5m
		x = (self.drone_location_gps[0] - self.drone_location_initial_gps[0])*5*1.5/0.000013552
		y = (self.drone_location_gps[1] - self.drone_location_initial_gps[1])*5*1.5/0.000014245
		self.drone_location_matrix[0] = 500 + int(x)
		self.drone_location_matrix[1] = 500 + int(y)

		# print(self.drone_location_matrix)

	def frame_drone_to_matrix(self, theta, d):

		# ----------------> (x/latitude)
		# |
		# |
		# |
		# |
		# |
		# |
		# ^(y/longitude)

		#To convert gps coordinates to coordinates in map	
			
		# self.drone_location_matrix[0] = self.sensor_data.drone_location[0]*

		#To find coordinates in map cooresponding to a sensor reading
		x = self.drone_location_matrix[0] + d*np.cos(theta)
		y = self.drone_location_matrix[1] + d*np.sin(theta)
		x = int(x)
		y = int(y)
		return [x,y]

	def logit(self, p):
		if(p == 0 or p == 1):
			return 0
		else:
			return np.log(p/(1-p))


	def inverse_measurement_model(self):
		curr_data = []

		#Positive latitude range sensor
		d = self.laser_positive_latitude
		theta = self.drone_yaw
		if(not d == float('inf')):
			curr_data.append(self.frame_drone_to_matrix(d = d, theta = theta) + [d])

		#Negative longitude range sensor
		d = self.laser_negative_longitude
		theta = self.drone_yaw + np.pi/2
		if(not d == float('inf')):
			curr_data.append(self.frame_drone_to_matrix(d = d, theta = theta) + [d])
		
		#Negative latitude range sensor
		d = self.laser_negative_latitude
		theta = self.drone_yaw + np.pi
		if(not d == float('inf')):
			curr_data.append(self.frame_drone_to_matrix(d = d, theta = theta) + [d])

		#Positive longitude range sensor
		d = self.laser_positive_latitude
		theta = self.drone_yaw + np.pi*1.5
		if(not d == float('inf')):
			curr_data.append(self.frame_drone_to_matrix(d = d, theta = theta) + [d])

		return curr_data

def main():
	curr_data = my_map.inverse_measurement_model()
	for x,y,d in curr_data:
		# my_map.belief_map[x][y] -= my_map.logit(1/d)*5
		my_map.belief_map[x][y] = 1
		my_map.my_pub.publish(str(x) +' '+ str(y)+' ' + str(my_map.belief_map[x][y]))

	cv2.imshow('map', my_map.belief_map)
	cv2.waitKey(1)

if __name__ == '__main__':
	# pause of 4 sec to open and load the gazibo
    t = time.time()

    while time.time() - t < 10:
        pass

    my_map = occupancy_map()
    while not rospy.is_shutdown():
        main()
