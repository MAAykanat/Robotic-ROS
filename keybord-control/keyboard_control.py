import rospy
from pynput import keyboard

from nav_msgs import *

while(1):
    with keyboard.Events() as events:
        # Block for as much as possible
        event = events.get(0.05)
        if event is not None:
            if event.key == keyboard.KeyCode.from_char("w"):
                print("UP!!!")
            elif event.key == keyboard.KeyCode.from_char("d"):
                print("RIGHT!!!")
            elif event.key == keyboard.KeyCode.from_char("a"):
                print("LEFT!!!")
            elif event.key == keyboard.KeyCode.from_char("s"):
                print("DOWN!!!")
            elif event == None:
                print("Stop")
            elif event.key == keyboard.Key.esc:
                break
            else:
                pass
        else:
            print("Stop")