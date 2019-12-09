#!/usr/bin/python
import smbus
import math
import rospy
from navigation.msg import IMU_msg

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

class IMU:

    def __init__(self):
        rospy.init_node("IMU", anonymous=True)
        self.pub = rospy.Publisher("IMU", IMU_msg, queue_size=10)
        self.bus = 0
        self.address = 0
        self.calGyroX = 0.0
        self.calGyroZ = 0.0
        self.calAccelX = 0.0
        self.calAccelZ = 0.0

        self.filter_angX= 0.0 
        self.deltaT =0.04
        self.constanT = 1
    
    def read_byte(self,reg):
        return self.bus.read_byte_data(self.address, reg)

    def read_word(self,reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        value = (h << 8) + l
        return value

    def read_word_2c(self,reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def dist(self,a,b):
        return math.sqrt((a*a)+(b*b))

    def get_y_rotation(self,x,y,z):
        radians = math.atan2(x, self.dist(y,z))
        return -math.degrees(radians)

    def get_x_rotation(self,x,y,z):
        radians = math.atan2(y, self.dist(x,z))
        return math.degrees(radians)

    def get_z_rotation(self, x, y, z):
        radians = math.atan2(z, self.dist(x,y))
        return math.degrees(radians)

    def calibrate(self):
        gyroZAvg = 0.0
        accelZAvg = 0.0
        gyroXAvg = 0.0
        accelXAvg = 0.0

        for x in range(1,5):
            self.bus = smbus.SMBus(1)  # bus = smbus.SMBus(0) fuer Revision 1
            self.address = 0x68  # via i2cdetect

            self.bus.write_byte_data(self.address, power_mgmt_1, 0)

            #X-axis cal
            gyroskop_xout = self.read_word_2c(0x43)
            gyroXAvg += gyroskop_xout / 131
            accel_xout = self.read_word_2c(0x3b)
            accelXAvg += accel_xout / 16384.0

            #Z-axis cal
            gyroskop_zout = self.read_word_2c(0x47)
            gyroZAvg += gyroskop_zout/131
            accel_zout = self.read_word_2c(0x3f)
            accelZAvg += accel_zout/16384.0

        self.calGyroZ = gyroZAvg/5.0
        self.calAccelZ = accelZAvg/5.0
        self.calGyroX = gyroZAvg/5.0
        self.calAccelX = accelXAvg/5.0


    def pub_imu(self):

        self.bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
        self.address = 0x68       # via i2cdetect

        self.bus.write_byte_data(self.address, power_mgmt_1, 0)


        gyroskop_xout = self.read_word_2c(0x43)
        gyroskop_yout = self.read_word_2c(0x45)
        gyroskop_zout = self.read_word_2c(0x47)


        beschleunigung_xout = self.read_word_2c(0x3b)
        beschleunigung_yout = self.read_word_2c(0x3d)
        beschleunigung_zout = self.read_word_2c(0x3f)

        beschleunigung_xout_skaliert = (beschleunigung_xout / 16384.0) - self.calAccelX
        beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
        beschleunigung_zout_skaliert = (beschleunigung_zout / 16384.0) - self.calAccelZ

        print "X Rotation: " , self.get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
        print "Y Rotation: " , self.get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
        print "Z Rotation: ",gyroskop_zout/131-self.calGyroZ
        xRot = self.get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
        yRot = self.get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)

        xGyro = gyroskop_xout/131 - self.calGyroX
        zGyro = gyroskop_zout/131 - self.calGyroZ



        #filter attempt
        a = self.constanT/(self.constanT+self.deltaT)
        gyro_ang = self.filter_angX + (xGyro*self.deltaT)
        self.filter_angX = a * gyro_ang + (1-a) * beschleunigung_xout_skaliert

        msg = IMU_msg()
        msg.xRotation = self.filter_angX
        msg.yRotation = yRot
        msg.zRotation = zGyro

        self.pub.publish(msg)

if __name__=="__main__":
	
    imu = IMU()
    imu.calibrate()

    while not rospy.is_shutdown():
        print("started IMU")
        imu.pub_imu()

