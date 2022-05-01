
from genpy.message import fill_message_args
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
import threading
import time
#from gps_common.msg import GPSFix
#from gps_common.msg import GPSStatus
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
import json 
import os
#from encoder import Encoder








def Publisher():
    #pb = rospy.Publisher('/fix', NavSatFix, queue_size=10)
    target_gps_pub = rospy.Publisher('/target_gps', String, queue_size=10)
    rpy_publisher = rospy.Publisher('/imu/rpy', Vector3Stamped, queue_size= 10)
    rospy.init_node("publisher", anonymous=False)
    rate = rospy.Rate(1)
    msg = Vector3Stamped()
    msg.vector.z = math.pi
    deger = NavSatFix()
    #deger.latitude = 40.996948
    #deger.longitude = 29.065001




    target_gps = String()
  

    flag_for_json = True

    path_for_target_gps = os.getcwd()

    if os.path.exists('./dangerous_zone.json'):
        path_for_target_gps = os.path.join(path_for_target_gps, 'dangerous_zone.json')
        with open(path_for_target_gps, 'r') as json_file:
            json_data = json.load(json_file)

        if json_data["dangerous_latitude"] != [] and json_data["dangerous_longitude"] != []:
            json_data["latitude"] = json_data["dangerous_latitude"] 
            json_data["longitude"] = json_data["dangerous_longitude"]

            json_data["dangerous_latitude"] = []
            json_data["dangerous_longitude"] = []

            with open(path_for_target_gps, 'w') as json_file:
                json.dump(json_data, json_file)
        
        else:
            flag_for_json = False





    while not rospy.is_shutdown():
        parsed_str = ""
        with open('current_gps.txt', 'r') as f:
            try:
                data = f.read().splitlines(True)
                array = data[0]
                array = array.split(",")
                array[0] = float(array[0])
                array[1] = float(array[1])
            #current_gps
                deger.latitude = array[0]
                deger.longitude = array[1]

            except: 
                print("HATA var")
                deger.latitude = 0.0
                deger.longitude = 0.0

        try:
            path_for_target_gps = os.getcwd()
            
            if os.path.exists('./dangerous_zone.json'):

                if flag_for_json == True:
                   path_for_target_gps = os.path.join(path_for_target_gps, 'dangerous_zone.json')

                else:
                    path_for_target_gps = os.path.join(path_for_target_gps, 'denemerobodragos/robodragos/base/UAV/target_gps_boat.txt')
                    
            with open(path_for_target_gps, 'r') as f:
                if path_for_target_gps.endswith('target_gps_boat.txt'):
                    try:
                        data = f.read().splitlines(True)
                    
                        target_lats_and_longs = []
                        for value in data:
                            value = value.split("\n")
                            str_value = str(value[0])
                            target_lats_and_longs.append(str_value)
                            parsed_str = parsed_str + str_value + "\n"
                            #rospy.loginfo("Parsed Str:" + parsed_str)
                            print("target_gps_boat.txt")

                            
                    
                        if(target_lats_and_longs != []):
                            target_gps = parsed_str
                        else:
                            print("else")
                            target_gps = str(-1)

                        print("Target GPS:",target_gps)

                    except:
                        print("Can't find the GPS values!")
                        target_gps = str(-1) 

                elif path_for_target_gps.endswith('dangerous_zone.json'):
                    json_file = open(path_for_target_gps)

                    json_data = json.load(json_file)

                    parsed_str = ""

                    for lat,lon in zip(json_data['latitude'], json_data['longitude']):
                        parsed_str = parsed_str + str(lat) + "," + str(lon) + "\n"

                    print(str(parsed_str)) 
                    target_gps = str(parsed_str)

                    print("dangerous_zone.json")

        except:
            print("Can't find the target_gps.txt!")
            target_gps = str(-1)
            
    
        print("Target GPS:",target_gps)
        #pb.publish(deger)
        target_gps = str(target_gps)
        target_gps_pub.publish(target_gps)
        #rpy_publisher.publish(msg)
        rate.sleep()
        
        

if __name__ == "__main__":
    try:
        Publisher()
        print("try")
    except rospy.ROSInternalException:
        pass
