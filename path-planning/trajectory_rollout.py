#!/usr/bin/env python3
import numpy as np
import rospy

import math
import tf

from gazebo_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *


####Class####

class TurtleBot:

    def __init__(self):

        self.robot_name = 'pioneer3dx'
        self.rate = rospy.Rate(20) # Rate in Hz

        self.pose_gazebo = np.zeros((3,1), dtype=float)
        self.pose_odom = np.zeros((3,1), dtype=float)
        
        self.angular_velocity = np.zeros((3,1), dtype=float)
        self.linear_acceleration = np.zeros((3,1), dtype=float)

        self.headingAngle_gazebo = None
        self.headingAngle_odom = None
        self.laser_ranges = None

        self.gazebo_publisher = Twist()
        self.odom_publisher = Twist()
        self.motion_model_publisher = Twist()

    
    ###Subscribers###
    
        #-Gazebo-#
        self.subGazStates = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cbGazStates, queue_size=10)
        #-Laser-#
        self.subLaserScan = rospy.Subscriber('/scan', LaserScan, self.cbLaserScan, queue_size=10)
        #-Imu-#
        self.subImu = rospy.Subscriber('/imu', Imu, self.cbImu, queue_size=10)
        #-Odometry-#
        self.subOdometry = rospy.Subscriber('/odom', Odometry, self.cbOdometry, queue_size=10)
        
        ###Publishers###
        self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.gazebo_pub = rospy.Publisher('/gazebo_pub', Twist, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom_pub', Twist, queue_size=10)
        self.motion_model_pub = rospy.Publisher('/motion_model', Twist, queue_size=1) # Created to publish motion model value in same frequency as /cmd_vel

    
    def cbGazStates(self,msg):
        # Takes robot x,y and theta  from gazebo space
        if not msg == None:
            name = msg.name
            for i in range(0,len(name)):
                if name[i] == self.robot_name:
                    self.pose_gazebo[0] = msg.pose[i].position.x
                    self.pose_gazebo[1] = msg.pose[i].position.y
                    self.pose_gazebo[2] = msg.pose[i].position.z

                    orientation_quarter = (msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w)
                    euler = tf.transformations.euler_from_quaternion(orientation_quarter)
                    
                    self.headingAngle_gazebo = euler[2]

                    self.gazebo_publisher.linear.x = self.pose_gazebo[0]
                    self.gazebo_publisher.linear.y = self.pose_gazebo[1]
                    self.gazebo_publisher.angular.z = self.headingAngle_gazebo

                    #self.gazebo_pub.publish(self.gazebo_publisher)
                    #print("Turtle-bot x position: {0}\nTurtle-bot y position: {1}\nTurtle-bot heading: {2}\n".format(self.pose_gazebo[0], self.pose_gazebo[1],self.headingAngle_gazebo))
    def cbLaserScan(self, msg):
        # Takes lidar data 
        self.laser_ranges = msg.ranges
        # print("-----Lidar reading--------")
        # print("Number of lidar message: ", len(self.laser_ranges))
    
    def cbImu(self, msg):
        # Imu sensor data
        self.angular_velocity[0] = msg.angular_velocity.x
        self.angular_velocity[1] = msg.angular_velocity.y 
        self.angular_velocity[2] = msg.angular_velocity.z 

        self.linear_acceleration[0]= msg.linear_acceleration.x
        self.linear_acceleration[1]= msg.linear_acceleration.y
        self.linear_acceleration[2]= msg.linear_acceleration.z

        # print("-----Imu reading--------")
        # print("Angular Velocity x: ",self.angular_velocity[0])
        # print("Angular Velocity y: ",self.angular_velocity[1])
        # print("Angular Velocity z: ",self.angular_velocity[2])
        # print("------------------------")
        # print("Linear accelaration x: ",self.linear_acceleration[0])
        # print("Linear accelaration y: ",self.linear_acceleration[1])
        # print("Linear accelaration z: ",self.linear_acceleration[2])
    
    def cbOdometry(self, msg):
        # Takes robot x,y and theta from odometry message
        self.pose_odom[0] = msg.pose.pose.position.x
        self.pose_odom[1] = msg.pose.pose.position.y 
        self.pose_odom[2] = msg.pose.pose.position.z

        orientation_quarter = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(orientation_quarter)

        self.headingAngle_odom = euler[2]

        self.odom_publisher.linear.x = self.pose_odom
        self.odom_publisher.linear.y = self.pose_odom
        self.odom_publisher.angular.z = self.headingAngle_odom
        # print("Odom x position: {0}\nOdom y position: {1}\nOdom heading: {2}\n".format(self.pose_odom[0], self.pose_odom[1],self.headingAngle_odom))        

    def process_LIDAR(self):
        if self.laser_ranges is not None:
            # I have a 360x1 range vector
            for i in range(0,len(self.laser_ranges)):
                # Look for non-Inf data
                if self.laser_ranges[i] is not np.inf:
                    self.idx_bearing = i
                    self.bearing = (i/180.)*np.pi
                    self.dist = self.laser_ranges[i]
                    # print('Detection dist:', self.dist, 'bearing:', self.bearing)
                else:
                    self.bearing = 99.
                    self.dist = None
                    # print('Object not detected by LIDAR')
            # print("Done!!")

    def move(self, linearx, lineary, angularz):
        #Taking cmd_vel values and publish them
        #Move Comment 
        msg = Twist()
        
        self.linearx = linearx
        self.lineary = lineary
        self.angularz = math.radians(angularz)

        msg.linear.x = linearx
        msg.linear.y = lineary
        msg.angular.z = angularz
        self.pubVel.publish(msg)

    def motion_model_(self):
        #Motion Model of TurtleBot
        self.motion_model_publisher.linear.x = (self.pose_odom[0] + (self.linearx*abs(math.cos(self.headingAngle_odom))))
        self.motion_model_publisher.linear.y = (self.pose_odom[1] + (self.linearx*abs(math.sin(self.headingAngle_odom))))
        self.motion_model_publisher.angular.z = (self.headingAngle_odom + (self.angular_velocity[2]*180/math.pi))

        # self.linearx = (self.pose_odom[0] + (self.linearx*abs(math.cos(self.headingAngle_odom))))
        # self.lineary = (self.pose_odom[1] + (self.linearx*abs(math.sin(self.headingAngle_odom))))
        # self.angularz = (self.headingAngle_odom + (self.angular_velocity[2]*180/math.pi))
        #In slide ME-597-3-Motion Modeling page 31, There is an motion model of TurtleBot
        #[X1,t]                 [X1,t-1 + u1,t * cosX3,t-1 * dt]
        #|X2,t| = g(Xt-1,ut) =  |X2,t-1 + u1,t * sinX3,t-1 * dt|
        #[X3,t]                 [X3,t-1 + u2,t * dt]
        #
        print("Turtle bot x positon: {0} \nTurtle bot y positon: {1}\nTurtle bot heading: {2}\n\n\n".format(self.linearx, self.lineary, self.angularz))
        self.motion_model_pub.publish(self.motion_model_publisher)


def main():
    rospy.init_node('robot_node', anonymous=True)
    robot = TurtleBot()
    
    linearx = 0.1
    lineary = 0.0
    angularz = 0.04
    
    while not rospy.is_shutdown():
        #robot.gazebo_pub.publish(robot.gazebo_publisher)
        robot.move(linearx, lineary, angularz) 
        robot.motion_model_()
        #robot.process_LIDAR()
        robot.rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except:
        pass