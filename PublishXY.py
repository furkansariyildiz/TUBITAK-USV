import rospy
import threading
import numpy as np
import math
import matplotlib.pyplot as plt
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
import ros_np_multiarray as rnm

class scanner():
	def __init__(self):

		self.rawData = [];
		self.newScanFlag = False;
		self.t1 = threading.Thread(target=self.processData)
		self.obstacles = []
		self.heading_radians = []
		
		self.msg = Float32MultiArray()

		self.robot_x = 0
		self.robot_y = 0
		self.radius = 0.5
		
		#PUBLISHERS
		self.obstacle_coordinates_publisher = rospy.Publisher('/obstacles', numpy_msg(Float32MultiArray), queue_size = 10) 
		self.radius_publisher = rospy.Publisher('/radius_of_circles', String, queue_size = 10)
		#SUBSCRIBERS
		rospy.init_node('scan_values')
		rospy.Subscriber('/scan', LaserScan, self.calbackLaser)
		rospy.Subscriber('/current_gps_xyz', String, self.get_polar_coordinates)
		rospy.Subscriber('/heading', Vector3Stamped, self.heading_callBack)		
		
		self.t1.start()

	def get_polar_coordinates(self,msg):
       		data = msg.data
        	data = data.split(",")
        	self.robot_x = float(data[0])
        	self.robot_y = float(data[1])	

	def calbackLaser(self,msg):
		infinity = float('inf')
		if(self.newScanFlag==False):
			for item in msg.ranges:
				self.rawData.append(item)
			self.newScanFlag = True
		else:
			for i in range(len(self.rawData)):
				self.rawData[i] = msg.ranges[i]

	def heading_callBack(self, msg):
		self.heading_radians = msg.vector.z
        #self.heading_trigonometric_angle = self.heading_radians*180/math.pi
		
	def calculatePoints(self,comp,index):
		radian = math.radians(index)
		xLength = comp*math.cos(radian)
		yLength = comp*math.sin(radian)
		xPoint = self.robot_x + xLength*math.cos(self.heading_radians) - yLength*math.sin(self.heading_radians)
		yPoint = self.robot_y + xLength*math.sin(self.heading_radians) + math.cos(self.heading_radians)*yLength
		self.obstacles.append((xPoint,yPoint))
	

	def processData(self):
		infinity = float('inf')
		while True:
			for i in range(len(self.rawData)):
				if self.rawData[i]!=infinity and self.heading_radians!=[]:
					self.calculatePoints(self.rawData[i],i)
			
			rospy.sleep(1)
			npArr = rnm.to_multiarray_f32(np.array(self.obstacles, dtype=np.float32))
			self.obstacle_coordinates_publisher.publish(npArr)
			self.radius_publisher.publish(str(self.radius))
			self.obstacles = []
						
	
if __name__ == '__main__':

	Scanner = scanner()
	#Scanner.listener()
	rospy.spin()


