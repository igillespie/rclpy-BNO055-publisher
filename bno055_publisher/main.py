#!/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import board
import adafruit_bno055
# https://github.com/adafruit/Adafruit_CircuitPython_BNO055

class bno055_ros2(Node):

    def __init__(self) -> None:
        super().__init__("bno055")

        self.pub = self.create_publisher(Imu, 'data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'mag' ,10)
        self.temp_pub = self.create_publisher(Temperature, 'temp' ,10)

        self.declare_parameter('pub_rate',30)
        self.declare_parameter('i2c_address', 40) #40 is hexidecimal 0x28 

        hz = self.get_parameter('pub_rate').get_parameter_value().integer_value
        # i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value

#BNO055 Setting =========================================================
        self.get_logger().info('Initializing I2C to bno0533 IMU')
        i2c = board.I2C()
        # 40 is 0x28
        # 41 is 0x29
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, i2c_address)
        self.last_val = 0xFFFF

        # Start Loop ============================================================
        period = 1/hz
        
        self.get_logger().info('bno0555 publishing at hz: '+str(hz))
        
        self.create_timer(period, self.publish_readings)

# except:
            # msg.orientation.x = 0
    def publish_readings(self):
       # try:
            qx, qy, qz, qw = self.sensor.quaternion
            gx, gy, gz = self.sensor.gyro
            ax, ay, az = self.sensor.linear_acceleration

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.orientation.x = float(qx)
            msg.orientation.y = float(qy)
            msg.orientation.z = float(qz)
            msg.orientation.w = float(qw)
            #msg.orientation_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)
            #msg.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            #msg.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
            
            
            x,y,z = self.sensor.magnetic
            
            mag_msg = MagneticField()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = 'imu_link'
            
            mag_msg.magnetic_field.x = x;
            mag_msg.magnetic_field.y = y;
            mag_msg.magnetic_field.z = z;
            
            temp_msg = Temperature()
            temp_msg.header.stamp = msg.header.stamp
            temp_msg.header.frame_id = 'imu_link'
            temp_msg.temperature = float(self.sensor.temperature)
            
            self.pub.publish(msg)
            self.mag_pub.publish(mag_msg)
            self.temp_pub.publish(temp_msg)
            
        #except:
         #   pass



def ros_main(args = None):
    rclpy.init(args=args)
    ros_class = bno055_ros2()
    
    try:
        rclpy.spin(ros_class)
    except KeyboardInterrupt:
        pass
    finally:
        ros_class.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    ros_main()
