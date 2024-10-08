#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.setpoint = setpoint

        self.prev_error = 0
        self.integral = 0
        self.last_time = rospy.Time.now()

    def compute(self, current_value):
        error = self.setpoint - current_value
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt == 0:
            dt = 10e-4    
        self.integral += error * dt    
        derivative = (error - self.prev_error) / dt

        # Proportional term
        p = self.kp * error

        # Integral term
        i = self.ki * self.integral

        # Derivative term
        d = self.kd * derivative

        # PID output
        sum = p + i + d

        # Update previous values
        self.prev_error = error
        self.last_time = current_time

        return sum
    
class Control:
    def __init__(self):
        rospy.init_node('pid_rotation_class')

        self.kp_yaw = rospy.get_param("~kp_yaw", 1.0)
        self.ki_yaw = rospy.get_param("~ki_yaw", 0.0)
        self.kd_yaw = rospy.get_param("~kd_yaw", 0.0)

        # Target positions for x, y, and yaw
        self.target_angle = rospy.get_param("~target_angle", 0.0)  

        self.pid_yaw = PID(self.kp_yaw, self.ki_yaw, self.kd_yaw, self.target_angle)

        rospy.Subscriber('/imu', Float32MultiArray, self.imu_callback)  

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def imu_callback(self, imu_msg):
        #receiving data each time it's publised on imu topic
        yaw_deg = imu_msg.data[5] 

        #after pid
        rotation_angle= self.pid_yaw.compute(yaw_deg)
        #converting angle diffrence to radian then to velocity 
        rad_angle = math.radians(rotation_angle)  
        linear_rad_velocity_x = math.cos(rad_angle)  
        linear_rad_velocity_y = math.sin(rad_angle) 
        #converting again to degrees/sec
        linear_velocity_x=math.degrees(linear_rad_velocity_x) 
        linear_velocity_y=math.degrees(linear_rad_velocity_y) 

        
        # Saving the velocities in a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity_x  
        twist_msg.linear.y = linear_velocity_y  
        twist_msg.angular.z = rotation_angle


        # Publish the command to the robot
        self.cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        control = Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
