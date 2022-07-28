import imp
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

    self.rate = rospy.Rate(20) # Rate in Hz

    
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
    self.pubVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    def cbGazStates():
        pass
    
    def cbLaserScan():
        pass
    
    def cbImu():
        pass

    def cbOdometry():
        pass



def main():
    pass

if __name__ == '__main__':
    try:
        main()
    except:
        pass