import port_finder
import json
import os
import subprocess

class AHRS():
    def __init__(self):
        self.json_file = "sensors_order.json"
    
        self.ahrsCommand = ""
        self.devpath_ahrs_tag = ""

        self.ahrs_port = ""
        self.flag = False
        self.ahrsIDVendor = "Cygnal Integrated Products, Inc.\n"


    def read_json(self):
        f = open(self.json_file)
        data = json.load(f)
        f.close()

        self.ahrsCommand = data["ahrs_command"]
        self.devpath_ahrs_tag = data["ahrsdevpath"]
        print(self.devpath_ahrs_tag)

    def findAHRSPort(self):
        port = port_finder.PortFinder()
        port.listAllPorts()
        self.ahrs_port, self.flag = port.findPorts(self.devpath_ahrs_tag, self.ahrsIDVendor)

    def executeAHRS(self):
        if self.flag == True:
            subprocess.Popen(self.ahrsCommand.format(self.ahrs_port),shell=True,stdout=subprocess.PIPE).stdout
            print("AHRS is opened.")
        else:
            print("AHRS can not start.")        

if __name__ == '__main__':
    ahrs = AHRS()
    ahrs.read_json()
    ahrs.findAHRSPort()
    ahrs.executeAHRS()
