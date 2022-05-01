#!/usr/bin/env python
import rospy
#import pygeodesy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
import threading
import time
#import pyproj
#from gps_common.msg import GPSFix
#from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
import json 
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import itertools
#self.motor_pub = rospy.Publisher('/motor/cmd', String, queue_size = 10)
#target_gps_pub = rospy.Publisher('/target_gps', NavSatFix, queue_size=10)


class Sim():
    def __init__(self):        
        self.sim_node = rospy.init_node("sim_node", anonymous=True)
        print(self.sim_node)

        self.degisen_gps = rospy.Publisher('/degisen_gps', String, queue_size=10)
        self.degisen_heading = rospy.Publisher('/degisen_heading', String, queue_size=10)
        self.turtle_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rrt_starter_publisher = rospy.Publisher("/rrt_starter", String, queue_size=10)

        #rospy.Subscriber('/imu/rpy', Vector3Stamped, self.um7_callback)
        rospy.Subscriber("/current_gps_xyz", String, self.current_xyz_callBack_sim)
        rospy.Subscriber("/heading", Vector3Stamped, self.current_heading_callBack_sim)
        rospy.Subscriber('/motor/cmd', String, self.speed_and_w_callback_sim)
        rospy.Subscriber("/target_gps_xyz", String, self.target_xyz_callBack_sim)
  
        
        self.stop_v_and_w = []
        
        self.speed = 0
        self.w = 0

        self.gps_x = 0
        self.gps_y = 0

        self.target_x = 0
        self.target_y = 0

        self.heading = 0
        self.phi = 0

        self.x_plot = []
        self.y_plot = []
        self.phi_plot = []

        self.dt = 1

        self.t = 0

        self.plt_sayaci = 0
        self.plot_dizi = []
        

    def current_xyz_callBack_sim(self, data):
        msg = data.data
        msg = msg.split(",")
        
        self.gps_x = float(msg[0])
        self.gps_y = float(msg[1])
        #print("GPS_X:",self.gps_x,"GPS_Y:",self.gps_y)

    def current_heading_callBack_sim(self,data):
        msg = data.vector.z
        #print("Heading:", msg)
        self.heading = msg
        
        

    def speed_and_w_callback_sim(self, data):
        msg = data.data
        msg = msg.split(",")
        self.speed = float(msg[0])
        #self.w = float(msg[1])*math.pi/180
        self.w = float(msg[1])
        #print("W:",self.w, "SPEED:", self.speed)

    def target_xyz_callBack_sim(self, data):
        msg = data.data
        msg = msg.split(",")
        self.target_x = msg[0]
        self.target_y = msg[1]
        #print("TARGET_X:",self.target_x, "TARGET_Y:",self.target_y)



    def simulation(self):
        
        time.sleep(2)
        rrt_controller = String()
        rrt_controller.data = "Start"
        x = self.gps_x
        y = self.gps_y
        self.phi = self.heading
        #print("FIRST HEADING:",self.heading*180/math.pi)
        while True:
            """
            self.phi = self.w*self.dt + self.phi  
            x =  x + 0.1*math.cos(self.phi)*self.dt
            y =  y + 0.1*math.sin(self.phi)*self.dt
            """
        #with open("current_gps.txt","w") as f:
           
                
             
            #f.write(str(x) + "," + str(y))
            print("ROBOT_X:",x, "ROBOT_Y:",y, "TARGET_X:",self.target_x, "TARGET_Y:",self.target_y)
            print("ROBOT_V:",0.1, "ROBOT_W(radian):",self.w,"ROBOT_HEADING (radian):",self.phi)

            
            
            self.rrt_starter_publisher.publish(rrt_controller)
            
            msg_twist = Twist()
            msg_twist.linear.x = self.speed
            msg_twist.angular.z = self.w
            
            

            self.turtle_publisher.publish(msg_twist)

            self.x_plot.append(x)
            self.y_plot.append(y)
            self.phi_plot.append(self.phi)

            self.t = self.t + self.dt
            self.degisen_gps.publish(str(x) + "," + str(y))
            self.degisen_heading.publish(str(self.phi))
            rospy.sleep(1)
            
            try:
                if(self.speed == 0 and self.w == 0):
                    msg_twist.linear.x = 0
                    msg_twist.angular.z = 0
                    self.turtle_publisher.publish(msg_twist)
                    #break
            except:
                print("#####HATA#####")

            print(self.t)

    def plot_metot(self,x_eksen, y_eksen, heading):

        index = 0
        

        self.plot_dizi.append(plt.gca())

        for x_robot in x_eksen:

            self.plot_dizi[self.plt_sayaci].plot(x_robot, y_eksen[index], marker=(3, 0, heading[index]*180/math.pi - 90), c='k',markersize=10, linestyle='None')

            self.plot_dizi[self.plt_sayaci].plot(x_robot, y_eksen[index], marker=(2, 0, heading[index]*180/math.pi - 90), c='k',markersize=20, linestyle='None')

            index = index + 1
        plt.show()

    


sim = Sim()


sim.simulation()
#sim.plot_metot(sim.x_plot, sim.y_plot, sim.phi_plot)
#plt.plot(sim.x_plot, sim.y_plot, 'v')
rospy.sleep(1)