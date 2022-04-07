import rospy # ROS libraries
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportRelative, TeleportAbsolute
from std_srvs.srv import Empty
from pynput.keyboard import Key, Listener, KeyCode # Libraries for keyboard input
import numpy as np


class myKeyboard(): 
    def __init__(self):
        self.COMBINATION_STOP = {Key.ctrl, KeyCode.from_char('c')} # Combination to stop the program
        self.keysPressed = set() # Var to store the keys pressed

        with Listener(on_press=self.onPress, on_release=self.onRelease) as self.listener: # Listener to listen for keyboard input
            welcome = """\nControls:
            * w: move forward
            * a: rotate left
            * s: move backward
            * d: rotate right

            * Space: Rotate turtle
            * r: Reset position"""

            rospy.loginfo(welcome)  # Show welcome message
            self.listener.join()  # Start listening for key presses

    def onPress(self, key):
        self.keysPressed.add(key) # Add key to the set of keys pressed
        print("\033[A")  # Print especial character to erase key pressed
        
        if KeyCode.from_char('w') in self.keysPressed: # Ifs to move the turtle in the desired direction
            self.pubVel(1,0)

        if KeyCode.from_char('s') in self.keysPressed:
            self.pubVel(-1,0)

        if KeyCode.from_char('a') in self.keysPressed:
            self.pubVel(0,np.pi/2)

        if KeyCode.from_char('d') in self.keysPressed:
            self.pubVel(0,-np.pi/2)

        if Key.space in self.keysPressed: # When this key is pressed, call the ros services to teleport the turtle
            try:
                rospy.wait_for_service('/turtle1/teleport_relative') # Wait for the service to be available
                rotateTurtle = rospy.ServiceProxy(
                    '/turtle1/teleport_relative', TeleportRelative) # Create handle to call the service
                moveRsp = rotateTurtle(0, np.pi)

                rospy.loginfo('Turtle rotated')
            except rospy.ServiceException as e: # If the service is not available, print a warning
                rospy.logwarn("Service teleport_relative call failed")

        if KeyCode.from_char('r') in self.keysPressed:
            try:
                rospy.wait_for_service('/turtle1/teleport_absolute')
                resetTurtle = rospy.ServiceProxy(
                    '/turtle1/teleport_absolute', TeleportAbsolute)
                resetRsp = resetTurtle(6, 6, np.pi)

                rospy.wait_for_service('/clear')
                clearTraje = rospy.ServiceProxy('/clear', Empty)
                resetRsp = clearTraje()

                rospy.loginfo('Turtle reset')
            except rospy.ServiceException as e:
                rospy.logwarn("Service teleport_absolute call failed")

        if key in self.COMBINATION_STOP: # If the combination to stop the program is pressed, stop the turtle and the keyboard listener
            if all(k in self.COMBINATION_STOP for k in self.keysPressed):
                self.listener.stop()
                rospy.signal_shutdown('Quit')

    def onRelease(self, key): # When a key is released, remove it from the set of keys pressed and stop the turtle
        self.keysPressed.remove(key)
        if not rospy.is_shutdown():
            self.pubVel(0, 0)

    def pubVel(self, linear, angular): # Function to publish the velocity to the turtle
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        pub.publish(msg)


if __name__ == "__main__": 
    rospy.init_node('TeleopKey', anonymous=False) # Initialize the node and instantiate the class myKeyboard
    myKeyboard()
    rospy.spin()