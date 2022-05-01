import port_finder
import json
import os
import subprocess


class GPSD():
    def __init__(self):
        self.json_file = "sensors_order.json"
    
        self.gpsCommand = ""
        self.devpath_gps_tag = ""
        
        self.gps_port = ""
        self.flag = False
        self.gpsIDVendor = "Garmin International\n"
        self.PASSWORD = ""

    def read_json(self):
        f = open(self.json_file)
        data = json.load(f)
        f.close()

        self.gpsCommand = data["gps_command"]
        self.devpath_gps_tag = data["gpsdevpath"]
        self.PASSWORD = str(data["password"])
        print(self.gpsCommand)
        print(self.devpath_gps_tag)
        print(self.PASSWORD)

    def findGPSDPort(self):
        port = port_finder.PortFinder()
        port.listAllPorts()
        self.gps_port, self.flag = port.findPorts(self.devpath_gps_tag, self.gpsIDVendor)
        print(self.gps_port, self.flag)

    def executeGPSD(self):
        if self.flag == True:
            subprocess.Popen(self.gpsCommand,shell=True,stdout=subprocess.PIPE).stdout
            print("GPSD is opened.")
        else:
            print("GPSD can not start.")

        
    
if __name__ == '__main__':
    gpsd = GPSD()
    gpsd.read_json()
    gpsd.findGPSDPort()
    gpsd.executeGPSD()
        
