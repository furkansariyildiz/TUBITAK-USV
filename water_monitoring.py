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
import random



class Water_Monitoring():
    def __init__(self):
        rospy.init_node('water_monitoring', anonymous=False)
        


        ############
        #Subscribers#
        ###########
        self.mission_process_subscriber = rospy.Subscriber('/mission_process', String, self.mission_process_callBack)
        
        
        

        ############
        #Publishers#
        ###########

        self.water_datas_publisher = rospy.Publisher('/water_datas', String, queue_size=10)





        self.mission_check = ""
        self.dt = 0.1

    def mission_process_callBack(self, data):
        msg = data.data
        if msg == "Start to monitoring":
            self.mission_check = msg

        else:
            self.mission_check = ""


    




    def water_monitoring(self):
        while True:
            if self.mission_check == "Start to monitoring":
                rospy.loginfo("Olcum yapiliyor!..")
                pH = random.uniform(7.5,10)
                ec = 185
                orp = 700
                do = 10
                temp = 20

                self.water_datas_publisher.publish(str(pH) + ";"+ str(ec) + ";" +str(orp) + ";" + str(do) + ";" + str(temp))
            else:
                rospy.loginfo("Olcum icin botun hedefe gitmesi bekleniyor.")

            rospy.sleep(self.dt)





water_monitoring = Water_Monitoring()

if __name__ == "__main__":
    water_monitoring.water_monitoring()