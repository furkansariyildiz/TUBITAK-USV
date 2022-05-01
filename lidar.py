import port_finder
import json
import os
import subprocess

class LIDAR():
    def __init__(self):
        self.json_file = "sensors_order.json"

        self.lidarCommand = ""
        self.devpath_lidar_tag = ""

        self.lidar_port = ""
        self.flag = False
        self.lidarIDVendor = "Cygnal Integrated Products, Inc.\n"


    def read_json(self):
        f = open(self.json_file)
        data = json.load(f)
        f.close()

        self.lidarCommand = data["lidar_command"]
        self.devpath_lidar_tag = data["lidardevpath"]
        print(self.lidarCommand)
        print(self.devpath_lidar_tag)

    def findLIDARPort(self):
        port = port_finder.PortFinder()
        port.listAllPorts()
        self.lidar_port, self.flag = port.findPorts(self.devpath_lidar_tag, self.lidarIDVendor)
        print(self.lidar_port, self.flag)


    def executeLIDAR(self):
        if self.flag == True:
            with open("rplidar_a3.launch", "r") as f:
                self.LidarXMLData = f.readlines()
                self.LidarXMLData[2] = '  <param name="serial_port"         type="string" value="' + self.lidar_port + '"/> \n'

            with open("rplidar_a3.launch", "w") as f:
                f.writelines(self.LidarXMLData)
            
            subprocess.Popen(self.lidarCommand,shell=True,stdout=subprocess.PIPE).stdout
            print("LIDAR is opened.")


        else:
            print("LIDAR can not start.")

    


if __name__ == '__main__':
    lidar = LIDAR()
    lidar.read_json()
    lidar.findLIDARPort()
    lidar.executeLIDAR()






