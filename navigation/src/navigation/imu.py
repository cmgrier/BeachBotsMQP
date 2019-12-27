#!/usr/bin/python
import smbus
import math
import rospy
import time
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
        self.calGyroY = 0.0

        self.calAccelX = 0.0
        self.calAccelY = 0.0
        self.calAccelZ = 0.0

        self.gyro_xAng = 0.0
        self.gyro_yAng = 0.0
        self.gyro_zAng = 0.0

        self.oldTime = 0.0
        self.currentTime = 0.0

        self.timer = time.time()

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

    def get_y_rotation(self, x, y, z):
        radians = math.atan2(x, self.dist(y, z))
        return -math.degrees(radians)

    def get_x_rotation(self, x, y, z):
        radians = math.atan2(y, self.dist(x, z))
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
            gyro_xout = self.read_word_2c(0x43)
            gyroXAvg += gyro_xout / 131
            accel_xout = self.read_word_2c(0x3b)
            accelXAvg += accel_xout / 16384.0

            #Z-axis cal
            gyro_zout = self.read_word_2c(0x47)
            gyroZAvg += gyro_zout/131
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

        #Time
        dt = time.time() - self.timer
        self.timer = time.time()

        gyro_xout = self.read_word_2c(0x43)
        gyro_yout = self.read_word_2c(0x45)
        gyro_zout = self.read_word_2c(0x47)


        accel_xout = self.read_word_2c(0x3b)
        accel_yout = self.read_word_2c(0x3d)
        accel_zout = self.read_word_2c(0x3f)

#********************Scaled Data************************************
        accel_xout_scaled = (accel_xout / 16384.0) #- self.calAccelX
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = (accel_zout / 16384.0) #- self.calAccelZ


        xGyroRaw = gyro_xout/131.0 #- self.calGyroX
        yGyroRaw = gyro_yout/131.0 #- self.calGyroY
        zGyroRaw = gyro_zout/131.0 #- self.calGyroZ
#*******************************************************************


# Angles
        accel_xAng = self.get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        accel_yAng = self.get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        accel_zAng = 0.0

        print(self.timer)
        print(dt)
        gyro_xAng = (xGyroRaw * dt)
        gyro_yAng = (yGyroRaw * dt)
        gyro_zAng = (zGyroRaw * dt)

        alpha = 0.96

        self.roll = alpha * (self.roll + gyro_xAng) + ((1.0 - alpha)*accel_xAng)
        self.pitch = alpha * (self.pitch + gyro_yAng) + ((1.0 - alpha)*accel_yAng)
        self.yaw = self.yaw + gyro_zAng

        print ("Accel: ", accel_xAng, " : ", accel_yAng, " : ", accel_zAng)
        print ("Gyro: ", gyro_xAng, " : ", gyro_yAng, " : ", gyro_zAng)
        msg = IMU_msg()
        msg.xRotation = self.roll
        msg.yRotation = self.pitch
        msg.zRotation = self.yaw

        self.pub.publish(msg)


if __name__=="__main__":
	
    imu = IMU()
    imu.calibrate()

    while not rospy.is_shutdown():
        print("started IMU")
        imu.pub_imu()

