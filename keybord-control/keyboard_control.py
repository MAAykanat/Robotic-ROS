#!/usr/bin/env python3
import rospy
import math
from pynput import keyboard

from nav_msgs import *
from geometry_msgs.msg import *

def move(linearx, lineary, angularz):
    # Define movement function
    msg = Twist()
    msg.linear.x = linearx
    msg.linear.y = lineary
    msg.angular.z = angularz
    
    pubVel.publish(msg)

rospy.init_node('teleop_control_node', anonymous=True)

linearx = 0.0
lineary = 0.0
angularz = 0.0

pubVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

while(True):
    with keyboard.Events() as events:
        # Take keybord event every 0.05 sec
        event = events.get(0.05)
        if event is not None:
            if event.key == keyboard.KeyCode.from_char("w"):
                # Move Forward
                move(0.3,0.0,0.0)
            elif event.key == keyboard.KeyCode.from_char("d"):
                # Turn Right
                print("RIGHT!!!")
            elif event.key == keyboard.KeyCode.from_char("a"):
                # Turn Left
                print("LEFT!!!")
            elif event.key == keyboard.KeyCode.from_char("s"):
                # Move Backward
                move(-0.3,0.0,0.0)
            elif event.key == keyboard.Key.esc:
                # Quit keyboard control by using ESC
                break
            else:
                pass
        else:
            # No event from keyboard stop robot
            move(0.0,0.0,0.0)