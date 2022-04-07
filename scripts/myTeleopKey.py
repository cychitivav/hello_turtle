#!/usr/bin/env
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportRelative, TeleportAbsolute
from std_srvs.srv import Empty
from pynput.keyboard import Key, Listener, KeyCode
import numpy as np  

def onPress(key):
    print ("\033[A")
    
    if key == KeyCode.from_char('w'):
        pubVel(1,0)

    if key == KeyCode.from_char('s'):
        pubVel(-1,0)

    if key == KeyCode.from_char('a'):
        pubVel(0,np.pi/2)

    if key == KeyCode.from_char('d'):
        pubVel(0,-np.pi/2)
        
    if key == Key.space:
        try:
            rospy.wait_for_service('/turtle1/teleport_relative')
            rotateTurtle = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
            moveRsp = rotateTurtle(0, np.pi)

            rospy.loginfo('Turtle rotated')
        except rospy.ServiceException as e:
            rospy.logwarn("Service teleport_relative call failed")

    if key == KeyCode.from_char('r'):
        try:
            rospy.wait_for_service('/turtle1/teleport_absolute')
            resetTurtle = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
            resetRsp = resetTurtle(6,6, np.pi)

            rospy.wait_for_service('/clear')
            clearTraje = rospy.ServiceProxy('/clear', Empty)
            resetRsp = clearTraje()

            rospy.loginfo('Turtle reset')
        except rospy.ServiceException as e:
            rospy.logwarn("Service teleport_absolute call failed")

    if key == Key.esc:
        rospy.signal_shutdown('test')

def onRelease(key):
    pubVel(0, 0)

def pubVel(linear,angular):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular

    pub.publish(msg)

def move():
    rospy.init_node('TeleopKey', anonymous=False)
    
    with Listener(on_press=onPress, on_release=onRelease) as listener:
        welcome = """\nControls:
        * w: move forward
        * a: rotate left
        * s: move backward
        * d: rotate right

        * Space: Rotate turtle,
        * r: Reset position
        
        * Esc: Quit"""
    
        rospy.loginfo(welcome)
        listener.join()
    
    rospy.spin()


if __name__ == "__main__":
    move()