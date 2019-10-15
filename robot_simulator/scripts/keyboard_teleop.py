#!/usr/bin/env python

"""
    Control of the robot with keyboad.
    Based on https://github.com/ros-teleop
"""

from __future__ import print_function
import rospy
from robot_simulator.msg import Command
import sys, select, termios, tty

msg = """
---------------------------
Moving around:
   u    i    o    p
   j    k    l    m
anything else : stop
q/z : increase/decrease max speeds by 10%
CTRL-C to quit
"""

moveKeys = ['i', 'j', 'k', 'l', 'p', 'm', 'u', 'o']

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


def vels(speed):
    return "currently:\tspeed %s " % (speed)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('command', Command, queue_size = 1)
    rospy.init_node('keyboard_teleop')

    speed = rospy.get_param("~init_speed", 0.5)
    inc_delta = rospy.get_param("~inc_delta", 0.1)  # increment for deltas
    inc_P = rospy.get_param("~inc_P", 0.1)  # increment for P
    delta_r = 0
    delta_e = 0
    n = speed
    P = 0

    try:
        print(msg)
        print(vels(speed))
        while(1):
            key = getKey()
            if key in moveKeys:
                b_print = True

                if key == 'u':
                    delta_e += inc_delta
                elif key == 'o':
                    delta_e -= inc_delta
                elif key == 'j':
                    delta_r -= inc_delta
                elif key == 'l':
                    delta_r += inc_delta
                elif key == 'p':
                    P -= inc_P
                elif key == 'm':
                    P += inc_P
                else:
                    b_print = False

                if key == 'i':
                    n = speed
                elif key == 'k':
                    n = -speed
                else:
                    n = 0.0

                if b_print:
                    print("n={:.1f} ; delta_r={:.2f} ; delta_e={:.2f} ; P={:.3f}".format(
                        n, delta_r, delta_e, P))

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                n = speed

                print(vels(speed))
            else:
                n = 0.0

                if (key == '\x03'):
                    break

            command = Command()
            command.n = n
            command.delta_r = delta_r
            command.delta_e = delta_e
            command.P = P
            pub.publish(command)

    except Exception as e:
        print(e)

    finally:
        command = Command()
        pub.publish(command)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
