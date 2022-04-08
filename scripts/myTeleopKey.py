import rospy  # ROS libraries
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportRelative, TeleportAbsolute
from std_srvs.srv import Empty
from pynput.keyboard import Key, Listener, KeyCode  # Libraries for keyboard input
import numpy as np


class myKeyboard():
    def __init__(self):
        self.keysPressed = set()  # Var to store the keys pressed

        # Listener to listen for keyboard input
        listener = Listener(on_press=self.onPress, on_release=self.onRelease)
        welcome = """\nControls:
        * w: move forward
        * a: rotate left
        * s: move backward
        * d: rotate right

        * Space: Rotate turtle
        * r: Reset position"""

        rospy.loginfo(welcome)  # Show welcome message
        listener.start()  # Start listening for key presses

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            linear = 0
            angular = 0

            # Ifs to move the turtle in the desired direction
            if KeyCode.from_char('w') in self.keysPressed:
                linear += 1

            if KeyCode.from_char('s') in self.keysPressed:
                linear -= 1

            if KeyCode.from_char('a') in self.keysPressed:
                angular += np.pi/2

            if KeyCode.from_char('d') in self.keysPressed:
                angular -= np.pi/2

            if Key.space in self.keysPressed:  # When this keys are pressed, call the ros services to teleport the turtle
                try:
                    # Wait for the service to be available
                    rospy.wait_for_service('/turtle1/teleport_relative')
                    # Create handle to call the service
                    rotateTurtle = rospy.ServiceProxy(
                        '/turtle1/teleport_relative', TeleportRelative)
                    moveRsp = rotateTurtle(0, np.pi)

                    rospy.loginfo('Turtle rotated')
                except rospy.ServiceException as e:  # If the service is not available, print a warning
                    rospy.logwarn("Service teleport_relative call failed")

            if KeyCode.from_char('r') in self.keysPressed:
                try:
                    rospy.wait_for_service('/turtle1/teleport_absolute')
                    resetTurtle = rospy.ServiceProxy(
                        '/turtle1/teleport_absolute', TeleportAbsolute)
                    resetRsp = resetTurtle(6, 6, np.pi)

                    rospy.wait_for_service('/clear')  # Clear the trajectory
                    clearTrajec = rospy.ServiceProxy('/clear', Empty)
                    resetRsp = clearTrajec()

                    rospy.loginfo('Turtle reset')
                except rospy.ServiceException as e:
                    rospy.logwarn("Service teleport_absolute call failed")

            self.pubVel(linear, angular)  # Publish the accumulated velocity
            rate.sleep()

    def onPress(self, key):
        print("\033[A")  # Print especial character to erase key pressed
        self.keysPressed.add(key)  # Add key to the set of keys pressed

    def onRelease(self, key):
        self.keysPressed.remove(key)

    def pubVel(self, linear, angular):  # Function to publish the velocity to the turtle
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        pub.publish(msg)

if __name__ == "__main__":
    # Initialize the node and instantiate the class myKeyboard
    rospy.init_node('TeleopKey', anonymous=False)
    myKeyboard()
