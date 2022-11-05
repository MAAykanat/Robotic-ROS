import rospy


# direction = input("Enter a keybord: ")

while(1):
    direction = input("Enter a keybord: ")
    # print(type(direction))
    if direction != "":
        if direction == "\x1b[A":
            print("UP!!!")
        elif direction == "\x1b[B":
            print("DOWN!!!")
        elif direction== "\x1b[C":
            print("RIGHT")
        elif direction == "\x1b[D":
            print("LEFT")
        elif direction == "s" or direction == "S":
            print("STOP")
        else:
            pass
    else:
        break