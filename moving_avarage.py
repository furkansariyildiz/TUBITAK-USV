import numpy as np
from random import random
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import time
import rospy
import rospy
#import pygeodesy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from tf2_msgs.msg import TFMessage
import threading
#import pyproj
#from gps_common.msg import GPSFix
#from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import json 
import rostopic

#import ros_np_multiarray as rnm
#from rospy.numpy_msg import numpy_msg

class Moving_average():
	def __init__(self):
		rospy.init_node('moving_average')


		############
		#SUBSCRIBERS#
		############
		rospy.Subscriber('/imu', Imu, self.imu_callBack_x)
		rospy.Subscriber('/imu', Imu, self.imu_callBack_y) 
		rospy.Subscriber('/imu', Imu, self.imu_callBack_z)
		rospy.Subscriber('/imu', Imu, self.imu_angular_callBack)
		rospy.Subscriber('/water_datas', String, self.water_datas_callBack)
		self.h = rostopic.ROSTopicHz(-1)
		self.s1 = rospy.Subscriber('/water_datas', String, self.h.callback_hz, callback_args='/water_datas')

		###########
		#PUBLISHERS#
		############

		self.imu_publisher = rospy.Publisher('/fusioned_imu', String, queue_size = 10)
		self.water_datas_publisher = rospy.Publisher('/fusioned_waterdatas', String, queue_size=10)

		self.linear_acc_x = []		
		self.linear_acc_x_average = 0

		self.linear_acc_y = []
		self.linear_acc_y_average = 0

		self.linear_acc_z = []
		self.linear_acc_z_average = []

		self.angular_velocity = []
		self.angular_velocity_average = []
		self.heading = 0

		self.hz = ""

		
		self.dt = 0.1

		self.pH = []
		self.pH_average = 0

		self.orp = []
		self.orp_average = 0

		self.do = []
		self.do_average = 0

		self.temp = []
		self.temp_average = 0

		self.ec = []
		self.ec_average = 0
		

		self.imu_hz = 200		

	def imu_callBack_x(self, msg):
		self.linear_acc_x.append(msg.linear_acceleration.x)

		if len(self.linear_acc_x) > self.imu_hz:
			self.linear_acc_x = self.linear_acc_x[-self.imu_hz:]
			
		self.linear_acc_x_average =   (self.linear_acc_x_average + sum(self.linear_acc_x))/float(len(self.linear_acc_x))

		if abs(self.linear_acc_x_average) < 0.01:
			self.linear_acc_x_average = 0
		
		#rospy.loginfo(str(self.linear_acc_x_average) + "###" + str(self.linear_acc_y_average))

		self.imu_publisher.publish(str(self.linear_acc_x_average) + "," + str(self.linear_acc_y_average) + "," + str(self.linear_acc_z_average) + "," + str(self.angular_velocity_average))




	def imu_callBack_y(self, msg):
		self.linear_acc_y.append(msg.linear_acceleration.y)

		if len(self.linear_acc_y) > self.imu_hz:
			self.linear_acc_y = self.linear_acc_y[-self.imu_hz:]
		
		self.linear_acc_y_average = (self.linear_acc_y_average + sum(self.linear_acc_y))/float(len(self.linear_acc_y)+1)

		if abs(self.linear_acc_y_average) < 0.01:
			self.linear_acc_y_average = 0




	def imu_callBack_z(self, msg):
		self.linear_acc_z.append(msg.linear_acceleration.z)

		if len(self.linear_acc_z) > self.imu_hz:
			self.linear_acc_z = self.linear_acc_z[-self.imu_hz:]

		self.linear_acc_z_average = sum(self.linear_acc_z)/float(len(self.linear_acc_z))



		
	def imu_angular_callBack(self, msg):
		self.angular_velocity.append(msg.angular_velocity.z)
		if len(self.angular_velocity) > self.imu_hz:
			self.angular_velocity = self.angular_velocity[-self.imu_hz:]
		
		self.angular_velocity_average = sum(self.angular_velocity)/len(self.angular_velocity)
		if abs(self.angular_velocity_average) < 0.01:
			self.angular_velocity_average = 0

			
	def water_datas_callBack(self, data):
		msg = data.data
		msg = msg.split(";")

		self.pH.append(float(msg[0]))
		self.ec.append(float(msg[1]))
		self.orp.append(float(msg[2]))
		self.do.append(float(msg[3]))
		self.temp.append(float(msg[4]))


		


	def moving_average(self):
		rospy.loginfo("Moving average basladi.")
		while True:
			

			try:
				self.hz = self.h.get_hz('/water_datas')
				self.hz = self.hz[0]

				if len(self.pH) > float(self.hz):
					self.pH = self.pH[-int(self.hz):]
					self.ec = self.ec[-int(self.hz):]
					self.orp = self.orp[-int(self.hz):]
					self.do = self.do[-int(self.hz):]
					self.temp = self.temp[-int(self.hz):]

				self.pH_average = (self.pH_average + sum(self.pH))/float(int(len(self.pH))+1)
				self.ec_average = (self.ec_average + sum(self.ec))/float(int(len(self.ec))+1)
				self.orp_average = (self.orp_average + sum(self.orp))/float(int(len(self.orp))+1)
				self.do_average = (self.do_average + sum(self.do))/float(int(len(self.orp))+1)
				self.temp_average = (self.temp_average + sum(self.temp))/float(int(len(self.temp))+1)

				self.water_datas_publisher.publish(str(self.pH_average) + ";" + str(self.ec_average) + ";" + str(self.orp_average) + ";" + str(self.do_average) + ";" + str(self.temp_average))
				
				rospy.loginfo("pH:" + str(self.pH_average) + " EC:" + str(self.ec_average) + " ORP:" + str(self.orp_average) + " DO:" + str(self.do_average) + " TEMP:" + str(self.temp_average) + "\n")

				

			except:
				pass


			rospy.sleep(0.1)




mv_avrg = Moving_average()
linear_acc_msg = String()

if __name__ == "__main__":
	mv_avrg.moving_average()

