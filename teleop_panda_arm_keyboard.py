#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : Toggle up (+z)
b : Toggle down (-z)

anything else : stop

q/z : increase/decrease distance by 0.01m (1cm)

CTRL-C to quit
"""

moveBindings = {
        'u':(1,1,0,0),
        'i':(1,0,0,0),
        'o':(1,-1,0,0),
        'j':(0,1,0,0),
        'k':(0,0,0,0),
        'l':(0,-1,0,0),
        'm':(-1,1,0,0),
        ',':(-1,0,0,0),
        '.':(-1,-1,0,0),
        'U':(1,1,0,0),
        'I':(1,0,0,0),
        'O':(1,-1,0,0),
        'J':(0,1,0,0),
        'K':(0,0,0,0),
        'L':(0,-1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
    }

zBindings={
        'q':(0.01,0.01),
        'z':(-0.01,-0.01)
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.dist = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, dist):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.dist = dist
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.dist
            twist.linear.y = self.y * self.dist
            twist.linear.z = self.z * self.dist
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.dist

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(dist):
    return "currently:\tDistance %s" % (dist)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_panda_arm_keyboard')

    dist = rospy.get_param("~dist", 0.01)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    up = False
    down = False

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, dist)

        print(msg)
        print(vels(dist))
        while(1):
            key = getKey(key_timeout)
            # print(key)
            if key == 't':
                if up == True:
                    up = False
                    print("XY Movement")
                else:
                    up = True
                    down = False
                    print("+Z Movement")
            if key == 'b':
                if down == True:
                    down = False
                    print("XY Movement")
                else:
                    up = False
                    down = True
                    print("-Z Movement")
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                if up == True:
                    th = +1
                elif down == True:
                    th = -1
                else:
                    th = 0
            elif key in zBindings.keys():
                dist = dist + zBindings[key][0]

                print(vels(dist))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, dist)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
