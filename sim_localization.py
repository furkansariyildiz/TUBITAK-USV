#!/usr/bin/env python
import rospy
#import pygeodesy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from tf2_msgs.msg import TFMessage
import threading
import time
#import pyproj
#from gps_common.msg import GPSFix
#from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import json 
import ros_np_multiarray as rnm
from rospy.numpy_msg import numpy_msg
from threading import Thread
#from matplotlib import pyplot as plt
import pyproj as proj

#from encoder import Encoder


#LOCALIZATION CURRENT_LAT CURRENT_LOT -> CURRENT_X, CURRENT_Y
#LOCALIZATION TARGET_LAT TARGET_LON -> TARGET_X, TARGET_Y

class Localization():
    def __init__(self):
        self.localization_node = rospy.init_node("localization_node", anonymous=True)
        #print(self.localization_node)
        self.current_lon = []
        self.current_lat = []

        self.current_lon_local = []
        self.current_lat_local = []

        self.target_lat = []
        self.target_lon = []

        self.target_lats = []
        self.target_longs = []

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    

        self.target_lat_local = []
        self.target_lon_local = []
     
        self.origin_lon = 29.065981
        self.origin_lat = 40.996194
        self.ahrs_vals = []

        self.degisen_gps_x = []
        self.degisen_gps_y = []

        self.current_msg = TFMessage()
        self.current_x = []
        self.current_y = []

        self.target_x = []
        self.target_y = []

        self.waypoints = []

        self.heading_msg = Vector3Stamped()
        self.distance_msg = String()
        self.local_current_x_y_msg = String()
        self.local_target_x_y_msg = String()
        self.filtered_gps_x_y_msg = String()
        
        self.x0_head = np.array([[0],[0],[0]])

        self.gps_cov = []
        self.Q = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.5]])
        self.R  = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0]]) #Noise degisirse bu matris de degismeli!

        self.acc_x = []
        self.acc_y = []
        self.w_speed = []

        self.prediction_list = []
        self.filtred_list = []
        self.odom_x = []
        self.odom_y = []

        self.robot_w = 0
        

        self.threshold = 5 #meters for max. distance
        self.threshold_array_for_target = []
        self.threshold_flag = True
        self.threshold_target_x_array = []
        self.threshold_target_y_array = []
        self.threshold_targets = []
        self.count_for_threshold = 0
        
        self.nav_status_check = ""
        self.count_for_targets = 0
        self.distance_to_target = ""

        self.crs_wgs = proj.Proj(init='epsg:4326')

        self.cust = proj.Proj("+proj=aeqd +lat_0={0} +lon_0={1} +datum=WGS84 +units=m".format(self.origin_lat, self.origin_lon))


        ##############
        ##PUBLISHERS##
        ##############
        self.current_gps_publisher_with_x_y_z = rospy.Publisher("/current_gps_xyz", String, queue_size=10)
        self.target_gps_publisher_with_x_y_z = rospy.Publisher("/target_gps_xyz", String, queue_size=10)
        self.heading_trigonometric_publisher = rospy.Publisher('/heading', Vector3Stamped, queue_size=10)
        self.distance_metric_publisher = rospy.Publisher('/distance', String, queue_size=10) #distance to target
        self.distance_to_waypoint_publisher = rospy.Publisher('/distance_to_waypoint', numpy_msg(Float32MultiArray), queue_size=10) #distance to next waypoint 
        self.threshold_target_points_publisher = rospy.Publisher('/threshold_targets', numpy_msg(Float32MultiArray), queue_size=10)
        self.move_command_to_nav_publisher = rospy.Publisher('/move_command', String, queue_size=10)
        self.nav_status_feedback_publisher = rospy.Publisher('/nav_status/feedback', String, queue_size=10)
        self.rrt_starter_str_publisher = rospy.Publisher('/rrt_starter', String, queue_size=10)
        ###############
        ##SUBSCRIBERS##
        ###############
        rospy.Subscriber('/odom', Odometry, self.odometry_callBack)
        rospy.Subscriber('/fusioned_imu', String, self.imu_acc_callBack)
        #rospy.Subscriber('/imu', Vector3Stamped, self.um7_callback)
        #rospy.Subscriber('/imu', Imu, self.um7_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.callback_gps_current_fix)
        rospy.Subscriber('/target_gps', String, self.callback_target_gps_fix)
        rospy.Subscriber('/degisen_gps', String, self.degisen_gps_callBack)
        rospy.Subscriber('/tf', TFMessage, self.turtlebot3_pos_callBack)
        rospy.Subscriber('/odom', Odometry, self.get_rotation)
        rospy.Subscriber('/waypoints', numpy_msg(Float32MultiArray), self.waypoints_callBack)
        rospy.Subscriber('/nav/status', String, self.nav_status_callBack)
        
        #rospy.Subscriber('/waypoints', String, self.waypoints_callBack)

    def odometry_callBack(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
    def imu_acc_callBack(self, msg):
        self.acc_x = float(msg.data.split(",")[0])
        self.acc_y = float(msg.data.split(",")[1])
        self.w_speed = float(msg.data.split(",")[3])

    

    def turtlebot3_pos_callBack(self,data):
        pass

    def callback_gps_current_fix(self, data):
        self.current_lat = data.latitude
        self.current_lon = data.longitude
        self.gps_cov = data.position_covariance
        #su anki lon ve lat degerleri aliniyor.
        #rospy.loginfo(str(self.current_lon) + ' ' + str(self.current_lat))

        #kovaryansi gormek #SUBSCRIBERSicin:
        #pos_cov_vec = data.position_covarianca
        #pos_cov = np.reshape(pos_cov_vec, (3,3))

        #pos_cov_det = np.linalg.det(pos_cov)
    
    def get_rotation(self,data):
        orientation_q  = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        #print(self.yaw)
        self.ahrs_vals = self.yaw
        #print("YAW:",self.ahrs_vals)

    def waypoints_callBack(self, data):
        self.waypoints = rnm.to_numpy_f32(data)  
        #print("Waypoints:", self.waypoints)  


    def callback_target_gps_fix(self, data):
        target_lats_and_longs = data.data
        target_lats_and_longs = target_lats_and_longs.split("\n")
        
        #target_lats_and_longs = target_lats_and_longs.split(",")
    

        for i in target_lats_and_longs:

            if i != "":
                i = i.split(",")
                self.target_lats.append(i[0])
                self.target_longs.append(i[1])

        




        
        rospy.loginfo("Count for targets:" + str(self.count_for_targets) + "||| Length Target Lats:" + str(len(self.target_lats)))
                    
        if self.count_for_targets <len(self.target_lats):
            
            self.nav_status_feedback_publisher.publish("True")
            if self.nav_status_check == "complated" and self.distance_to_target != "" and self.distance_to_target < 1.5:
                self.count_for_targets = self.count_for_targets + 1
                self.threshold_flag = True
                self.rrt_starter_str_publisher.publish("Start")
                
            
            if self.count_for_targets < len(self.target_lats):
                self.target_lon = float(self.target_longs[self.count_for_targets])
                self.target_lat = float(self.target_lats[self.count_for_targets])


            
            rospy.loginfo("Count for next target:" + str(self.count_for_targets))

            #print(target_lats_and_longs)

        else:
            self.rrt_starter_str_publisher.publish("Stop")
            self.threshold_flag = False
            self.nav_status_feedback_publisher.publish("False")
        
        """
        target_lats_and_longs = target_lats_and_longs.split("[")[1]
        target_lats_and_longs = target_lats_and_longs.split("]")[0]
        
        
        
        target_lats_and_longs = target_lats_and_longs.split(",")
        
        self.target_lat = float(target_lats_and_longs[0].split("'")[1])
        self.target_lon = float(target_lats_and_longs[1].split("'")[0])

        print("targets!!!!!###:", target_lats_and_longs[0])

         

        #rospy.loginfo("TARGET GPS LATITUDES AND LONGITUDES:" + str(self.target_lat) + ","  + str(self.target_lon))
        """
        rospy.loginfo("TARGET GPS LATITUDES AND LONGITUDES:" + str(self.target_lat) + ","  + str(self.target_lon))
        rospy.loginfo("Count:" + str(self.count_for_targets))
        self.target_lats = []
        self.target_longs = []


    def xy_to_latlon(self, x_point, y_point):
        lon , lat = self.cust(x_point, y_point, inverse = True)
        return lon, lat


    def um7_callback(self, data):
        #print(data.vector.z)
        
        if (data is not None):
            heading = data.vector.z

            if(heading>0 and heading<math.pi/2):
                angle = math.pi/2 - heading #angle: degree in xyz 
                self.ahrs_vals = angle
                #print(angle)
            elif(heading>math.pi/2 and heading<math.pi):
                angle = heading - math.pi/2
                angle = math.pi/2 - angle
                angle = math.pi/2 + angle
                self.ahrs_vals = angle
            elif(heading>math.pi and heading<3*math.pi/2):
                angle = heading - math.pi
                angle = math.pi/2 - angle
                angle = math.pi + angle
                self.ahrs_vals = angle
            else:
                angle = heading - 3*math.pi/2
                angle = math.pi - angle
            
                self.ahrs_vals.append(angle)
         


         
        if(data is not None):
            heading = data.vector.z
            #Nort ->0, East->3pi/2, West-> pi/2, South -> pi

            if(heading>math.pi and heading<2*math.pi):
                heading = heading + math.pi*5/2
                bolen = int(heading /(2*math.pi))
                heading = heading - bolen*2*math.pi
                rospy.loginfo(str(heading))
                self.ahrs_vals = heading
            else:
            
                heading = heading + math.pi/2
                
                self.ahrs_vals = heading
            """
        if(data is not None):
            heading = data.vector.z
            
            if(heading > -math.pi and heading<-math.pi/2 ):
                heading = heading + math.pi*5/2
            else:
                heading = heading + math.pi/2 

            self.ahrs_vals = heading

            rospy.loginfo(self.ahrs_vals*180/math.pi)
            """

    def nav_status_callBack(self, data):
        self.nav_status_check = data.data

        

    def dist_to_target(self):
        if(self.target_lat != [] and self.target_lon != [] and self.current_lat != [] and self.current_lon != []):
            #rospy.loginfo("TARGET_LAT AND LONG:" + str(self.target_lat) + "," + str(self.target_lon))
            R = 6371.0

            lat1 = math.radians(float(self.target_lat))
            lon1 = math.radians(float(self.target_lon))

            lat2 = math.radians(float(self.current_lat))
            lon2 = math.radians(float(self.current_lon))

            

            

            dlat = lat2 - lat1
            dlon = lon2 - lon1

            a = math.sin(dlat/2)*math.sin(dlat/2) + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)* math.sin(dlon/2)
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            dist = R * c * 1000
            return dist
        else:

            return -1


    def degisen_gps_callBack(self,data):
        msg = data.data
        msg = msg.split(",")
        self.degisen_gps_x = float(msg[0])
        self.degisen_gps_y = float(msg[1])        

        

    def get_local_coord(self, lat, lon):
        WORLD_POLAR_M = 6356752.3142
        WORLD_EQUATORIAL_M = 6378137.0

        eccentricity = math.acos(WORLD_POLAR_M/WORLD_EQUATORIAL_M)        
        n_prime = 1/(math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))),2.0)*math.pow(math.sin(eccentricity), 2.0)))        
        m = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)        
        n = WORLD_EQUATORIAL_M * n_prime

        diffLon = float(lon) - float(self.origin_lon)
        diffLat = float(lat) - float(self.origin_lat)

        surfdistLon = math.pi /180.0 * math.cos(math.radians(float(lat))) * n
        surfdistLat = math.pi/180.00 * m

        x = diffLon * surfdistLon
        y = diffLat * surfdistLat

        return x,y      


    def publish_data(self):
        while True:
            if self.ahrs_vals != []:
                self.heading_msg.vector.z = self.ahrs_vals
                deger = self.heading_msg.vector.z
                rospy.loginfo("Heading with Angle:" + str(self.ahrs_vals*180/math.pi))
                self.heading_trigonometric_publisher.publish(self.heading_msg)

            if self.current_lat != [] and self.current_lon != [] and self.target_lat != [] and self.target_lon != []:
                self.target_x, self.target_y = self.get_local_coord(self.target_lat, self.target_lon)
                rospy.loginfo("Current_lat:" + str(self.current_lat) + " Current_lon:" + str(self.current_lon))
                self.current_x, self.current_y = self.get_local_coord(self.current_lat, self.current_lon)

                if self.current_x != [] and self.current_y != []:
                    distance = math.sqrt((self.current_x - self.target_x)**2 + (self.current_y - self.target_y)**2)
                    self.distance_to_target = distance
                    #print(self.current_x , self.current_y)
                    #print("Distance is real?:", distance)
                    self.distance_msg.data = str(distance)

                    self.distance_metric_publisher.publish(self.distance_msg) 
                    if self.waypoints != []:
                        distance_to_waypoint = []

                        for i in range(0, len(self.waypoints)):
                            distance_to_waypoint.append(math.sqrt((self.current_x - self.waypoints[i][0])**2 + (self.current_y - self.waypoints[i][1])**2))
                
                        self.distance_to_waypoint_publisher.publish(rnm.to_multiarray_f32(np.array(distance_to_waypoint, dtype=np.float32)))

                        rospy.loginfo("DISTANCE TO NEXT WAYPOINTS WITH ARRAY:" + str(distance_to_waypoint))

                    else:
                        pass
                    
                    rospy.loginfo("CURRENT" + str(self.current_x) + "," + str(self.current_y))
                    rospy.loginfo("TARGET" + str(self.target_x) + "," + str(self.target_y))
                    #print(pygeodesy.latlon2n_xyz(10.0, 0.0))
                    rospy.loginfo("DISTANCE:" + str(distance))
                    rospy.loginfo("HEADING:" + str(self.ahrs_vals))
                    self.local_current_x_y_msg.data = str(self.current_x) + "," + str(self.current_y)

                    if distance > self.threshold and self.threshold_flag == True:
                        scale = int(distance/self.threshold)
                        #print("Scale:" + str(scale))
                        scaled_x = self.target_x/float(scale)
                        scaled_y = self.target_y/float(scale)
                        for i in range(0, scale):
                            self.threshold_targets.append([scaled_x + scaled_x*i, scaled_y + scaled_y*i])

                        self.threshold_flag = False 

                    rospy.loginfo("THRESHOLD X AND Y:" + str(self.threshold_targets))

                    if self.threshold_flag == False:
                        distance_to_threshold_target = math.sqrt((self.current_x - self.threshold_targets[self.count_for_threshold][0])**2 + (self.current_y - self.threshold_targets[self.count_for_threshold][1])**2 )
                        if distance_to_threshold_target < 1:
                            self.count_for_threshold = self.count_for_threshold + 1
                        
                        try:
                            self.threshold_target_points_publisher.publish(rnm.to_multiarray_f32(np.array(self.threshold_targets[self.count_for_threshold], dtype = np.float32)))
                        except:
                            pass
                    

                    self.local_target_x_y_msg.data = str(self.target_x) + "," + str(self.target_y)
                    self.target_gps_publisher_with_x_y_z.publish(self.local_target_x_y_msg)
                    #self.current_gps_publisher_with_x_y_z.publish(self.filtered_gps_x_y_msg)

            rospy.sleep(1)  


    def kalman_filter(self):
        """
        x = [P; P_dot] P=>x_position, y_positon , P_dot => vel_x, vel_y
        P = [x;y] , P_dot = [V],
        First, must be calculate xk_inverse_head and Pk_inverse_head. This step is prediction. After, this program will calculate K(gain), get Yk from /gps/fix topic,
        then xk_head and Pk_head. This is corrected values with kalman filter.
        """
        rospy.sleep(1)
        v_limit = np.array([0])
        x_prev_list = self.x0_head
        c = 0
        k = 0
        t = 0

        Pk =  np.array([[self.gps_cov[0],0,0],[0,self.gps_cov[4],0],[0,0,0]])
        rospy.loginfo("Kalman Filter Started!")
        
        while True:
            if self.acc_x != [] and self.current_x != [] and self.current_y != []:
                #print("Kalman while girdi!")
                
                dt = 1
           

                #a = self.acc_x #turtlebot'un ivme ivme => 
                


                if abs(self.w_speed) > 0.01:
                    self.robot_w = self.robot_w + self.w_speed*dt
                    #print("ROBOT_HEADING:",self.robot_w)

            #MOTION MODEL#
                #print(a)
                F = np.array([[1,0,dt*math.cos(self.robot_w)],[0,1,dt*math.sin(self.robot_w)],[0,0,1]])
                F_transpose = np.transpose(F)
                G = np.array([
                    [0,0,0],
                    [0,0,0],
                    [0,0,dt]])
                H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                H_transpose = np.transpose(H)

                
                xk_prev = x_prev_list[0] #x-1
                #print(xk_prev)
                yk_prev = x_prev_list[1] #y-1
                #print(yk_prev)
                V_prev = x_prev_list[2] #V-1
                #print(V_prev)
                v_limit = V_prev  
                #print("v_limit",v_limit[0])
                
                
                
                xk_state_prev = np.array([xk_prev,yk_prev,V_prev])

                self.prediction_list = xk_state_prev

                #mutlak = math.sqrt(self.acc_x**2 + self.acc_y**2)


                #print("xk_state_prev:",xk_state_prev)
                uk_prev = np.array([[0],[0],[self.acc_x]])
                #print("uk_prev",uk_prev)

                
                Pk_inverse_head = np.dot(np.dot(F,Pk), F_transpose) + self.Q #Pk_inverse_head = F*P*F_t + Q
                #print(Pk_inverse_head)


                


                #prediction
                #print("F",F)
                #print("***********")
                #print(G)
                #print("***********")
                #print(xk_state_prev)
                #print("***********")
                #print(uk_prev)
                #print("***********")

                xk_state_inverse_head = np.dot(F,xk_state_prev) + np.dot(G,uk_prev)
                #print("Ivme degeri:" + str(self.acc_x))
                #print(".:Predict X:.")
                #print(xk_state_inverse_head)

                
                
                
                c = xk_state_inverse_head[2] 
                t = xk_state_inverse_head[0] + math.cos(self.robot_w)*dt
                k = xk_state_inverse_head[1] + math.sin(self.robot_w)*dt

                #x_prev_list = np.array()

                
                #print(xk_state_inverse_head[2][0])

                #print("************************")

                x_prev_list = np.array([t,k,c])
               
                #print(x_prev_list)

                #CORRECTION
                #Yk = np.dot(H,)
                #optimal gain
                
               
                #print(K)
            

                #print(H)
                #print(Pk_inverse_head)
                #t = np.linalg.inv((np.dot(np.dot(H, Pk_inverse_head), H_transpose))+ self.R)
                #t = np.dot(np.dot(H, Pk_inverse_head), H_transpose) + self.R
                K = np.dot(np.dot(Pk_inverse_head,H_transpose),np.linalg.inv((np.dot(np.dot(H, Pk_inverse_head), H_transpose))+ self.R))
                Yk = np.array([[self.current_x],[self.current_y], [c]]) #value from sesnors 
                #print(Yk)

                
                #print(K) #Optimal Gain
                xk_head = xk_state_inverse_head + np.dot(K,(Yk - np.dot(H,xk_state_inverse_head)))
                I = np.array([[1,0,0], [0,1,0], [0,0,1]])
                Pk = np.dot((I-np.dot(K,H)),Pk_inverse_head)
                #print("Xk Kalman:")
                #print(xk_head)
                #print("*********************")
                #print("Pk Kalman:")
                #print(Pk)
                #print("************************")
                #print("Robot_heading:" + str(self.robot_w))
            

                

                if xk_head[2][0] < 0.1:
                    xk_head[2][0] = 0

                self.filtred_list = xk_head

                x_prev_list = xk_head
                self.filtred_list = np.array(self.filtred_list, dtype = float)
                self.filtered_gps_x_y_msg.data = str(self.filtred_list[0][0]) + "," + str(self.filtred_list[1][0])
                self.current_gps_publisher_with_x_y_z.publish(self.filtered_gps_x_y_msg)
                lon, lat = self.xy_to_latlon(self.filtred_list[0][0], self.filtred_list[1][0])
                rospy.loginfo("Filtred_Lat:" + str(lat) + " Filtred_Lon:" + str(lon))
                with open("incoming_messages_deneme.txt",'a') as f_lat_and_long:
                    f_lat_and_long.write("09:33:39 connected:T armed:F guided:T mode:AUTO.LOITER;lat: "+ str(lat)+ " lon: " + str(lon) + ";x: 0.0 y: -0.02 z: 0.02;0.04;1.00;roll: -0.000472840241855 pitch: 0.000221150286961 yaw: -0.00126029911917;heading: "+ str(self.ahrs_vals*180/math.pi) +";\n")
                    
          
            

            rospy.sleep(1)


    def plot_data(self):
        t = 0
        while True:
            
            if self.odom_x != [] and self.current_x != []:
                plt.subplot(2,1,1)
                plt.axis([-10 + self.filtred_list[0][0], 10+self.filtred_list[0][0], -10 + self.filtred_list[1][0], 10 + self.filtred_list[1][0]])
                plt.ion()
                plt.show()
                #print("Filtered List:")
                #print(self.filtred_list[0][0], self.filtred_list[1][0])
                plt.plot(self.filtred_list[0][0], self.filtred_list[1][0], 'o', color = 'black')
                plt.plot(self.current_x, self.current_y, 'o', color = "red")
                plt.plot(self.odom_x, self.odom_y, 'o', color = "yellow")
                #print("Error plot 1")
                #plt.plot(self.odom_x, self.odom_y, "o-", color="blue")
                plt.draw()
                plt.pause(0.001)
                plt.subplot(2,1,2)
                plt.axis([-10+t,10+t,-10,10])
                plt.ion()
                plt.show()
                err = math.sqrt((self.odom_x - self.filtred_list[0][0])**2 + (self.odom_y - self.filtred_list[1][0])**2)
                err_with_gps = math.sqrt((self.odom_x - self.current_x)**2 + (self.odom_y - self.current_y)**2)
                #print("Error Value:" + str(err))
                plt.plot(t, err, ".-", color = "black")
                plt.plot(t, err_with_gps, ".-", color = "red")
                plt.grid(True)
                plt.draw()
                t = t + 1               
             
            rospy.sleep(1)
                    

local = Localization()

if __name__ == "__main__":
    Thread(target = local.publish_data).start()
    Thread(target = local.kalman_filter).start()
    #Thread(target = local.plot_data).start()
