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
        self.gazebo_publisher = Twist()
    
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
        self.gazebo_pub = rospy.Publisher('/gazebo_pub', Twist, queue_size=1)
    
    def cbGazStates(self,msg):
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

                    self.gazebo_pub.publish(self.gazebo_publisher)

                    print("Turtle-bot x position: {0}\nTurtle-bot y position: {1}\nTurtle-bot heading: {2}\n".format(self.pose_gazebo[0], self.pose_gazebo[1],self.headingAngle_gazebo))
    
    def cbLaserScan():
        pass
    
    def cbImu():
        pass

    def cbOdometry():
        pass



def main():
    rospy.init_node('robot_node', anonymous=True)
    robot = TurtleBot()
    while not rospy.is_shutdown():
        robot.rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except:
        pass