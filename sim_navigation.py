#!/usr/bin/env python
# coding: utf8

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
import threading
import time
#from gps_common.msg import GPSFix
#from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
import json
import ros_np_multiarray as rnm
#from encoder import Encoder
from rospy.numpy_msg import numpy_msg
import os


class Navigation():
    def __init__(self):
        rospy.init_node("navigation", anonymous=False)

        self.dt = 0.1
        self.w = 0
        self.steer_prev = -60

        self.SPEED_CONST = 0.1

        self.P = 0
        self.I = 0
        self.D = 0
        
        self.kp = 0.2
        self.kd = 0.005
        self.ki = 0

        self.err = 0
        self.err_p = 0
        
        self.count_for_wp = 1

        self.target_local_x = []
        self.target_local_y = []

        self.waypoint_list = []

        self.current_local_x = []
        self.current_local_y = []

        self.distance_to_target = []
        self.distance_to_waypoints = []
        self.heading_trigonometric_angle = []
        self.heading_radians = []

        self.degisen_gps_x = []
        self.degisen_gps_y = []
        self.degisen_heading = 0
        self.cont = False

        self.nav_on = False

        self.obstacle_coordinates = []

        ###############
        ##SUBSCRIBERS##
        ###############
        self.target_xyz_sub = rospy.Subscriber("/target_gps_xyz", String, self.target_xyz_callBack) 
        self.current_xyz_sub = rospy.Subscriber("/current_gps_xyz", String, self.current_xyz_callBack)
        self.filtered_xyz_sub = rospy.Subscriber("/filtered_xyz", String, self.filtered_xyz_callBack)#Kalman Filtered
        self.distance_sub = rospy.Subscriber("/distance", String, self.distance_callBack)
        self.heading_sub = rospy.Subscriber("/heading", Vector3Stamped, self.heading_callBack)
        self.degisen_gps_sub = rospy.Subscriber("/degisen_gps", String, self.degisen_gps_callBack)
        self.degisen_heading_sub = rospy.Subscriber('/degisen_heading', String, self.degisen_heading_CallBack)
        self.obstacles = rospy.Subscriber('/obstacles', numpy_msg(Float32MultiArray), self.obstacle_callBack)
        self.waypoints = rospy.Subscriber('/waypoints', numpy_msg(Float32MultiArray), self.waypoints_callBack)
        self.distance_to_waypoints_sub = rospy.Subscriber('/distance_to_waypoint', numpy_msg(Float32MultiArray), self.distance_to_waypoints_callBack)
        self.nav_status_feedback = rospy.Subscriber('/nav_status/feedback', String, self.nav_status_feedback_callBack)
        ##############    
        ##PUBLISHERS##
        ##############
        self.target_gps_pub = rospy.Publisher("/rudder_controller", NavSatFix, queue_size=10)
        self.motor_pub = rospy.Publisher('/motor/cmd', String, queue_size = 10)
        self.nav_status_pub = rospy.Publisher('/nav/status', String , queue_size=10)
        self.distance_status = rospy.Publisher('/check_distance', String, queue_size=10)
        self.desired_waypoint = rospy.Publisher('/desired_waypoint', String, queue_size=10)
        self.mission_process_publisher = rospy.Publisher('/mission_process', String, queue_size=10)
        



    def nav_status_feedback_callBack(self, data):
        msg = data.data

        if msg == "True":
            self.cont = True
            self.nav_on = True

        elif msg == "False":
            self.cont = False
            self.nav_on = False




    def filtered_xyz_callBack(self, msg):
        pass

    def target_xyz_callBack(self,data):
        msg = data.data
        msg = msg.split(",")
        self.target_local_x = float(msg[0])
        self.target_local_y = float(msg[1])
        #self.nav_on = True

    def degisen_gps_callBack(self, data):
        msg  = data.data
        msg = msg.split(",")
        self.degisen_gps_x = float(msg[0])
        self.degisen_gps_y = float(msg[1])
        #print(self.degisen_gps_x, self.degisen_gps_y, "DEGISTI")
        
    def obstacle_callBack(self, data):
        self.obstacle_coordinates = rnm.to_numpy_f32(data)    

    def waypoints_callBack(self, data):
        self.waypoint_list = rnm.to_numpy_f32(data)
        self.count_for_wp = 1
        #print("Waypoint Count:", self.count_for_wp)

    def degisen_heading_CallBack(self,data):
        self.degisen_heading = float(data.data)

    def current_xyz_callBack(self, data):
        msg = data.data
        msg = msg.split(",")
        self.current_local_x = float(msg[0])
        self.current_local_y = float(msg[1])
    
    def distance_callBack(self, data):
        msg = data.data
        self.distance_to_target = float(msg)

    def distance_to_waypoints_callBack(self, data):
        self.distance_to_waypoints = rnm.to_numpy_f32(data) 
    
        #print("DISTANCE_TO_WP",self.distance_to_waypoints)

    def heading_callBack(self, data):
        self.heading_radians = data.vector.z
        self.heading_trigonometric_angle = self.heading_radians*180/math.pi

    def navigation_loop(self):
        while True:
            while self.nav_on:
                rospy.sleep(1)
                prev_time = time.time()
                x_= np.zeros((3,1))
                #cont = True
                
               
                while self.cont:
                    if self.distance_to_waypoints != [] and self.count_for_wp < len(self.distance_to_waypoints):
                        if float(self.distance_to_waypoints[self.count_for_wp]) < 0.5:
                            self.count_for_wp = self.count_for_wp + 1
                            print("HEDEFE GELDIIIIIIIIII##############")
                    else:
                        print("ELSE ICERISINDE!")
                        


                    self.SPEED_CONST = 0.1

                    print("DISTANCE TO TARGET:::::::###############",self.distance_to_target)
                    if self.distance_to_target < 1.5:
                        self.cont = False
                        break

                    curr_time = time.time()
                    self.dt = curr_time - prev_time
                    """
                    if(self.degisen_gps_x == [] or self.degisen_gps_y == []):

                        
                        heading_ = -self.heading_radians + math.atan2(self.target_local_y - self.current_local_y, self.target_local_x - self.current_local_x)
                        desired_heading = math.atan2(math.sin(heading_), math.cos(heading_))
                        #print(str(heading) + ' ' + str(desired_heading) + ' ' +  str(heading_))
                        print("ROBOT_X:",self.current_local_x,"ROBOT_Y:",self.current_local_y, "DESIRED HEADING:",desired_heading)
                        new_heading = self.heading_radians + desired_heading
                        print("IF ICERISINDE")
                    else:
                        print("ELSE ICERISINDE")
                        heading_ = -self.degisen_heading + math.atan2(self.target_local_y - self.current_local_y, self.target_local_x - self.current_local_x)
                        desired_heading = math.atan2(math.sin(heading_), math.cos(heading_))
                        new_heading = self.heading_radians + desired_heading
                        print("ROBOT_X:",self.current_local_x, "ROBOT_Y:",self.current_local_y,"DESIRED HEADING:",desired_heading)
                    """

                    if len(self.waypoint_list) != 0 and self.count_for_wp <= len(self.waypoint_list):
                        if(self.current_local_x != [] and self.current_local_y != []):
                            try:
                                #heading_ = -self.heading_radians + math.atan2(self.target_local_y - self.current_local_y, self.target_local_x - self.current_local_x)
                                heading_ = -self.heading_radians + math.atan2(self.waypoint_list[self.count_for_wp][1] - self.current_local_y, self.waypoint_list[self.count_for_wp][0] - self.current_local_x)
                                desired_heading = math.atan2(math.sin(heading_), math.cos(heading_))
                                print("ROBOT_X:",self.current_local_x,"ROBOT_Y:",self.current_local_y, "DESIRED HEADING:",desired_heading)
                                new_heading = self.heading_radians + desired_heading
                                rospy.loginfo("Count for waypoints:" + str(self.count_for_wp))
                                self.desired_waypoint.publish(str(self.waypoint_list[self.count_for_wp][0]) + "," + str(self.waypoint_list[self.count_for_wp][1]))
                        
                            except:
                                desired_heading = 0
                                

                        self.err = desired_heading

                        self.P = self.kp*self.err
                        self.I = self.I + (self.ki*self.err)*self.dt
                        self.D = self.kd * (self.err - self.err_p)/self.dt

                        self.err_p = self.err

                        self.w = self.P + self.I + self.D
               
                        print("SELF.ERR:",self.err)

                        print('PID:' + str(self.P) + "|" + str(self.I) + "|" + str(self.D))
                    
                        """
                        if(steer_cmd > 40 or self.err> math.radians(15)):
                            print("######if icersinde#######")
                            steer_cmd = 40
           

                        elif(steer_cmd < -40 or self.err < math.radians(-15)):
                            print("########else icierisinde#####")
                            steer_cmd = -40
                        """

                        """
                        startpos = (0., 0.)
                        endpos = (10., 11.)
                        obstacles = [(2., 3.), (4., 5.), (10, 10)]
                        n_iter = 250
                        radius = 0.5
                        stepSize = 0.7
                        
                        os.system("python rrt.py")
                        """
              


                        if(abs(self.err)>0.1):

                            command = String()
                            if self.w > math.pi/4:
                                self.w = math.pi/4
                            
                            elif self.w < -math.pi/4:
                                self.w = -math.pi/4

                            else:
                                command.data = str(self.SPEED_CONST) + ","  + str(self.w)
                                self.motor_pub.publish(command)
                                self.steer_prev = self.w
                        else:
                            self.w = 0
                            command = String()
                            command.data = str(self.SPEED_CONST) + "," + str(self.w)
                            self.motor_pub.publish(command)
                        
                        
                        rospy.loginfo("Motor command from NAV:" + str(self.SPEED_CONST) + ',' + str(self.w))

                        command = String()
                  
                    rospy.sleep(1)

                stop = String()
                stop.data = str(0) + ',' + str(0)    #speed = 0, w = 0

                rospy.loginfo("Stop data passed to route thread")
                self.motor_pub.publish(stop)

                msg = String()
                msg.data = "complated"
                self.SPEED_CONST = 0
                self.mission_process_publisher.publish("Start to monitoring") #su kalitesi kontrolunu baslat.
                rospy.sleep(60) #1 dakika boyunca olcum al
                self.mission_process_publisher.publish("")


                self.nav_status_pub.publish(msg)

                self.nav_on = False

                self.steer_prev = -60

                
            rospy.sleep(1)

                

nav = Navigation()
#while True:

def print_ciktisi():
    while True:  
        rospy.sleep(1)

navigationThread = threading.Thread(target = nav.navigation_loop)
navigationThread.start()
printThread = threading.Thread(target = print_ciktisi)
printThread.start()
rospy.sleep(1)