import subprocess
import json
import rospy


class PortFinder():
    def __init__(self):
        self.list_ports_command = "ls -l /dev/tty*"
        self.vendor_command = 'udevadm info --name={} | grep -i "vendor_from"'
        self.devpath_command = 'udevadm info --name={} | grep -i "devpath"'

        self.allports = ""
        self.allportsID = []

        self.idvendor = ""
        self.devPath = ""



    def listAllPorts(self):
        self.allports = subprocess.Popen(self.list_ports_command,shell=True,stdout=subprocess.PIPE).stdout
        self.allports = self.allports.read()
        self.allports = str(self.allports)
        print(type(self.allports))
        self.allports = self.allports.split("\n")
        for port in self.allports:
            try:
                port = port.split("/dev/tty")[1]
                self.allportsID.append("/dev/tty" + port)
                
                #self.allportsID.append("/dev/tty" + port.split("/dev/tty"))
            except:
                break



    def findIDVendor(self, port):
        self.idvendor = subprocess.Popen(self.vendor_command.format(port),shell=True,stdout=subprocess.PIPE).stdout
        self.idvendor = self.idvendor.read()
        return self.idvendor



    def findDevPath(self, port):
        self.devPath = subprocess.Popen(self.devpath_command.format(port), shell=True, stdout=subprocess.PIPE ).stdout
        self.devPath = self.devPath.read()
        return self.devPath


    
    def findPorts(self, device_path, id_vendor):
        for i in self.allportsID:
            try:
                if self.findIDVendor(i).split("=")[1] ==  id_vendor and self.findDevPath(i).split("/")[10] == device_path:
                    return i, True
            except:
                pass

        return None, False
    


if __name__ == '__main__':
    port = PortFinder()
    port.listAllPorts()
    port.readJsonFile()
    port.findPorts()

    print("STM32: {}, Lidar: {}, AHRS: {}, Water Monitoring: {}, Motor Controller: {}, GPS: {}".format(port.stm32Flag, port.lidarFlag, port.ahrsFlag, port.waterMonitoringFlag, port.motorControllerFlag, port.gpsFlag))
    port.execution_order()





