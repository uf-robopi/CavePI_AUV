#!/usr/bin/env python3

#This file is based on teleop_twist_keyboard.py

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#from geometry_msgs.msg import Twist
from nemogator_interfaces.msg import CavepiInput

import sys, select, termios, tty


msg = """
Reading from the keyboard
---------------------------
Planar Movement:
   q    w    e
   a         d
   z    x    c

i : up (+z)
k : down (-z)

Anything Else : stop

r/v : increase/decrease thruster power by 10% of full
---------------------------
CTRL-C to quit
"""

#heave-yaw-surge
moveBindings = {
    'q':( (2**0.5)/2, 0, 0, 0, 0,-(2**0.5)/2),
    'w':( 1, 0, 0, 0, 0, 0),
    'e':( (2**0.5)/2, 0, 0, 0, 0, (2**0.5)/2),
    'a':( 0, 0, 0, 0, 0,-1),
    'd':( 0, 0, 0, 0, 0, 1),
    'z':(-(2**0.5)/2, 0, 0, 0, 0,-(2**0.5)/2),
    'x':(-1, 0, 0, 0, 0, 0),
    'c':(-(2**0.5)/2, 0, 0, 0, 0, (2**0.5)/2),

    'i':( 0, 0,-1, 0, 0, 0),
    'k':( 0, 0, 1, 0, 0, 0),
}

speedBindings = {
    'r':(0.1,0),
    'v':(-0.1,0),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    user_input_pub = rospy.Publisher("/cavepi/user_input", CavepiInput, queue_size = 1)
    rospy.init_node('cavepi_teleop_keyboard')

    speed = rospy.get_param("~speed", 0.05)
    surge = 0
    heave = 0
    roll = 0
    yaw = 0    
    status = 0

    try:
        print(msg)
        print("Thruster input power is currently at: " + str(speed*100) + "%")

        while(True):
            key = getKey()
            if key in moveBindings.keys():
                surge = moveBindings[key][0]
                heave = moveBindings[key][2]
                roll = moveBindings[key][3]
                yaw = moveBindings[key][5]                

            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                if speed > 0.9:
                    speed = 1
                    print("Thruster power at maximum.")
                elif speed < 0.1:
                    speed = 0
                    print("Thurster power at minimum.")
                else:
                    print("Thruster power is currently at:" + str(speed*100) + "%")

            else:
                surge = 0
                heave = 0
                roll = 0
                yaw = 0
                if (key == '\x03'):
                    break

            user_input = CavepiInput()
            user_input.surge = surge * speed
            user_input.heave = heave * speed
            user_input.roll = roll * speed
            user_input.yaw = yaw * speed            
            user_input_pub.publish(user_input)

    except Exception as e:
        print(e)

    finally:
        user_input = CavepiInput()
        user_input.surge = 0
        user_input.heave = 0
        user_input.roll = 0
        user_input.yaw = 0        
        user_input_pub.publish(user_input)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)