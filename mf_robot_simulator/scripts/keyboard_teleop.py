#!/usr/bin/env python

"""
    Control of the robot with keyboad.
    Based on https://github.com/ros-teleop
"""

from __future__ import print_function
import rospy
from mf_robot_simulator.msg import Command, CartesianCommand
import sys, select, termios, tty

msg = """
---------------------------
Moving around:
   u    i    o    p
   j    k    l    m
Cartesian command:
   r    t    y
   f    g    h
anything else : stop
q/z : increase/decrease max speeds by 10%
CTRL-C to quit
"""

moveKeys = ['i', 'j', 'k', 'l', 'p', 'm', 'u', 'o', 'r', 't', 'y',
    'f', 'g', 'h']

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

TIMEOUT = -1  # to signal timeout on key input

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    key = None

    if rlist:
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    else:
        key = TIMEOUT

    return key


def vels(speed):
    return "currently:\tspeed %s " % (speed)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('command', Command, queue_size = 1)
    cart_pub = rospy.Publisher('cart_command', CartesianCommand, queue_size = 1)

    speed = rospy.get_param("~init_speed", 0.5)
    init_cmd_vx = rospy.get_param("~init_cmd_vx", 1.0)
    init_cmd_vy = rospy.get_param("~init_cmd_vy", 1.0)
    init_cmd_vz = rospy.get_param("~init_cmd_vz", 1.0)
    speed = rospy.get_param("~init_speed", 0.5)
    speed = rospy.get_param("~init_speed", 0.5)
    inc_delta = rospy.get_param("~inc_delta", 0.1)  # increment for deltas
    inc_P = rospy.get_param("~inc_P", 0.1)  # increment for P
    delta_r = 0
    delta_e = 0
    n = 0
    P = 0

    v_x = 0.0  # current cartesian speed
    v_y = 0.0
    v_z = 0.0
    cmd_vx = init_cmd_vx  # cartesian command speed
    cmd_vy = init_cmd_vy
    cmd_vz = init_cmd_vz


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
                elif key == 'y':
                    v_x = cmd_vx
                elif key == 'r':
                    v_x = -cmd_vx
                elif key == 'f':
                    v_y = cmd_vy
                elif key == 'h':
                    v_y = -cmd_vy
                elif key == 'g':
                    v_z = cmd_vz
                elif key == 't':
                    v_z = -cmd_vz
                else:
                    n = 0.0
                    v_x = 0.0
                    v_y = 0.0
                    v_z = 0.0

                if b_print:
                    print("n={:.1f} ; delta_r={:.2f} ; delta_e={:.2f} ; P={:.3f}".format(
                        n, delta_r, delta_e, P))

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                v_x = v_x * speedBindings[key][0]
                v_y = v_y * speedBindings[key][0]
                v_z = v_z * speedBindings[key][0]
                n = speed

                print(vels(speed))
            elif key == TIMEOUT:
                v_x = 0.0
                v_y = 0.0
                v_z = 0.0
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

            cart_command = CartesianCommand()
            cart_command.v_x = v_x
            cart_command.v_y = v_y
            cart_command.v_z = v_z
            cart_pub.publish(cart_command)

    except Exception as e:
        print(e)

    finally:
        command = Command()
        pub.publish(command)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
