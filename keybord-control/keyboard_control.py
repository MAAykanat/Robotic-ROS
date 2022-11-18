#!/usr/bin/env python3
import rospy
import math
from pynput import keyboard

from nav_msgs import *
from geometry_msgs.msg import *

def move(linearx, lineary, angularz):
    msg = Twist()
    msg.linear.x = linearx
    msg.linear.y = lineary
    msg.angular.z = angularz
    
    pubVel.publish(msg)

rospy.init_node('robot_node', anonymous=True)

linearx = 0.0
lineary = 0.0
angularz = 0.0

pubVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

while(1):
    with keyboard.Events() as events:
        # Block for as much as possible
        event = events.get(0.05)
        if event is not None:
            if event.key == keyboard.KeyCode.from_char("w"):
                move(0.3,0.0,0.0)
                print("UP!!!")
            elif event.key == keyboard.KeyCode.from_char("d"):
                move(0.0,0.0,-0.5)
                print("RIGHT!!!")
            elif event.key == keyboard.KeyCode.from_char("a"):
                move(0.0,0.0,0.5)
                print("LEFT!!!")
            elif event.key == keyboard.KeyCode.from_char("s"):
                print("DOWN!!!")
                move(-0.3,0.0,0.0)
            elif event == None:
                print("Stop")
            elif event.key == keyboard.Key.esc:
                break
            else:
                pass
        else:
            move(0.0,0.0,0.0)
            print("Stop")