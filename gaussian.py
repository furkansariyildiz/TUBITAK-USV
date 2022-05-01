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
import rostopic


#GAUSSIAN ISLEMLERINI BURADAN YAP, BUNU MOVING AVERAGE'A IMPORT ET.


class Gauss():
    def __init__(self):
        rospy.init_node('gaussian_node', anonymous=False)

        self.h = rostopic.ROSTopicHz(-1)
        self.s1 = rospy.Subscriber('/water_datas', String, self.h.callback_hz, callback_args='/fusioned_waterdatas')
        ############
        #SUBSCRIBERS#
        ###########
        rospy.Subscriber('/fusioned_waterdatas', String, self.waterdatas_callBack)
        rospy.Subscriber('/mission_process', String, self.mission_process_callBack)
        rospy.Subscriber('/gps/fix', NavSatFix, self.current_gps_callBack)
        ############
        #PUBLISHERS#
        ###########

        ###########

        self.pH = None
        self.ec = None
        self.orp = None
        self.do = None
        self.temp = None

        self.pH_average = 0
        self.ec_average = 0
        self.orp_average = 0
        self.do_average = 0
        self.temp_average = 0

        self.count_for_warning = 0 #uyarinin gerceklesmesi icin gerekli olan sayac
        self.count_for_reset = 0 #orneklemenin sifirlanmasi icin gerekli olan sayac
        self.count_for_result = 0 #orneklemenin sonucu icin gerekli olan sayac
        #self.count_for_json = 0 #json'a yazilacak olan lat-lon degerleri icin gerekli olan sayac




        self.alt_sinir_pH = 5 
        self.ust_sinir_pH = 8

        self.alt_sinir_ec = 180
        self.ust_sinir_ec = 200

        self.alt_sinir_orp = 600
        self.ust_sinir_orp = 800

        self.alt_sinir_do = 9.5
        self.ust_sinir_do = 12

        self.alt_sinir_temp = 5
        self.ust_sinir_temp = 25

        self.hz = ""

        self.mission_status = ""
        
        self.dangerous_latitude = []
        self.dangerous_longitude = []

        self.dangerous_zone = {
            "dangerous_latitude": [],
            "dangerous_longitude": []
        }

    def waterdatas_callBack(self, data):
        """
        gelen verilerin siralamasi su sekilde; ilk veri pH; ikinci veri EC; ucuncu veri ORP; dorduncu veri DO; besinci veri TEMP;
        ornegin: 7;100;100;100;25 gibi olacaktir.
        """
        msg = data.data
        msg = msg.split(";")

        self.pH = float(msg[0])
        self.ec = float(msg[1])
        self.orp = float(msg[2])
        self.do = float(msg[3])
        self.temp = float(msg[4])



    def mission_process_callBack(self, data):
        msg = data.data
        self.mission_status = msg



    def current_gps_callBack(self, data):
        self.dangerous_latitude = data.latitude
        self.dangerous_longitude = data.longitude



    def gaussian_process(self):
        while True:
            if self.mission_status == "Start to monitoring":                
                try:
                    
                    self.hz = self.h.get_hz('/fusioned_waterdatas')
                    self.hz = self.hz[0]
                    if self.pH != None:
                        if (self.pH > self.ust_sinir_pH or self.pH < self.alt_sinir_pH) or (self.ec > self.ust_sinir_ec or self.ec < self.alt_sinir_ec) or (self.orp > self.ust_sinir_orp or self.orp < self.alt_sinir_orp) or (self.do > self.ust_sinir_do or self.do < self.alt_sinir_do) or (self.temp > self.ust_sinir_temp or self.temp < self.alt_sinir_temp):
                            self.count_for_warning = self.count_for_warning + 1


                    

                    if self.count_for_warning >=5*self.hz:
                        #rospy.loginfo("Tehlikeli bolge!")
                        self.count_for_result = self.count_for_result + 1
                        self.count_for_warning = 0
                        

                    if self.count_for_reset >= 30*self.hz:
                        self.count_for_warning = 0   #30 saniyede bir uyarmayi yapan sayaci sifirlamayi saglar.
                        self.count_for_reset = 0 #eger sayac 30 saniyenin toplami kadar yani sistemin saniyede gerceklestirdigi salinim HZ*30 kadar artarsa kendini sifirlayacaktir.
                
                    self.count_for_reset = self.count_for_reset + 1

                    rospy.sleep(1/(float(self.hz)))
                
                except Exception as e: 
                    print(e)
                    rospy.sleep(1)
                #rospy.loginfo("Hata")

            else:

                if self.count_for_result >= 6: 
                    rospy.loginfo("Tehlikeli bolge!")
                    #self.dangerous_zone["dangerous_latitude"].append(self.dangerous_latitude)
                    #self.dangerous_zone["dangerous_longitude"].append(self.dangerous_longitude)

                    
                    with open('dangerous_zone.json', 'r') as f:
                        #json.dump(self.dangerous_zone, f)
                        json_data = json.load(f)

                    json_data["dangerous_latitude"].append(self.dangerous_latitude)
                    json_data["dangerous_longitude"].append(self.dangerous_longitude)

                    with open('dangerous_zone.json', "w") as f:
                        json.dump(json_data, f)
                    
              
                self.count_for_warning = 0
                self.count_for_reset = 0
                self.count_for_result = 0



            



g = Gauss()

if __name__ == "__main__":
    rospy.loginfo("Gaussian Sureci baslatiliyor...")
    g.gaussian_process()