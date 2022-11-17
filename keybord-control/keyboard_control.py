import rospy
from pynput import keyboard

from nav_msgs import *

while(1):
    with keyboard.Events() as events:
        # Block for as much as possible
        event = events.get(1e6)
        if event.key == keyboard.KeyCode.from_char("w"):
            print("UP!!!")
        elif event.key == keyboard.KeyCode.from_char("d"):
            print("RIGHT!!!")
        elif event.key == keyboard.KeyCode.from_char("a"):
            print("LEFT!!!")
        elif event.key == keyboard.KeyCode.from_char("s"):
            print("DOWN!!!")
        elif event.key == keyboard.KeyCode.from_char("p"):
            print("It is pause!!!")
        elif event.key == keyboard.KeyCode.from_char("q"):
            break
        else:
            pass