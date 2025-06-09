#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS4JoystickCmdVel:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ps4_joystick_cmdvel')

        # Parameters
        self.linear_axis = rospy.get_param('~axis_linear', 1)           # Left stick vertical (axis 1)
        self.l2_axis = rospy.get_param('~axis_l2', 2)                    # L2 trigger (axis 2)
        self.r2_axis = rospy.get_param('~axis_r2', 5)                    # R2 trigger (axis 5)
        self.deadzone = rospy.get_param('~deadzone', 0.1)                # Deadzone for stick
        self.max_linear = rospy.get_param('~max_linear_vel', 0.5)        # Max linear velocity (m/s)
        self.max_angular = rospy.get_param('~max_angular_vel', 1.0)      # Max angular velocity (rad/s)
        self.cmd_topic = rospy.get_param('~cmd_topic', 'cmd_vel')        # cmd_vel topic

        # Publisher
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)

        # Subscriber
        rospy.Subscriber('joy', Joy, self.joy_callback)

        rospy.loginfo("PS4JoystickCmdVel node initialized.")

    def joy_callback(self, joy_msg):
        twist = Twist()

        # Linear movement: left stick vertical
        lin_input = joy_msg.axes[self.linear_axis]
        if abs(lin_input) > self.deadzone:
            # Invert axis if needed (push forward = positive)
            twist.linear.x = lin_input * self.max_linear
        else:
            twist.linear.x = 0.0

        # Rotation: triggers L2 and R2
        # PS4 triggers go from -1 (released) to +1 (fully pressed)
        l2 = (joy_msg.axes[self.l2_axis] + 1.0) / 2.0
        r2 = (joy_msg.axes[self.r2_axis] + 1.0) / 2.0
        rot_input = r2 - l2  # positive => turn right, negative => turn left
        if abs(rot_input) > self.deadzone:
            twist.angular.z = rot_input * self.max_angular
        else:
            twist.angular.z = 0.0

        # Publish Twist message
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PS4JoystickCmdVel()
        node.run()
    except rospy.ROSInterruptException:
        pass
