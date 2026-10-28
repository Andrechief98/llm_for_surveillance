#!/usr/bin/env python3
import rospy, os, json, math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from cob_srvs.srv import SetString, SetStringResponse

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


teleport_actor = rospy.get_param('/teleport_actor', True) 
actor_control_modality = rospy.get_param('/actor_control_modality', "ps4")

world_id = rospy.get_param('world_id', '1')

script_dir = os.path.dirname(__file__) 

class actorNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('actor_node')

        with open(f"{script_dir}/../../llm_interface/config/ROS/info_map{world_id}.json", "r") as f:
            info = json.load(f)

        self.areas_dict = info["areas"]

        # Publisher
        self.actor_cmd_pub = rospy.Publisher("/actor/cmd_vel", Twist, queue_size=10)

        if teleport_actor:
            # Server
            self.teleport_server = rospy.Service('/teleport_actor', SetString, self.actor_teleport_service)

            # Client
            self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Subscriber
        if actor_control_modality == "ps4":
            # We read the joystick controller command and convert values in cmd_vel commands
            self.actor_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
            
            # Parameters for ps4 controller
            self.linear_axis = rospy.get_param('~axis_linear', 1)           # Left stick vertical (axis 1)
            self.l2_axis = rospy.get_param('~axis_l2', 2)                    # L2 trigger (axis 2)
            self.r2_axis = rospy.get_param('~axis_r2', 5)                    # R2 trigger (axis 5)
            self.deadzone = rospy.get_param('~deadzone', 0.1)                # Deadzone for stick
            self.max_linear = rospy.get_param('~max_linear_vel', 0.5)        # Max linear velocity (m/s)
            self.max_angular = rospy.get_param('~max_angular_vel', 1.0)      # Max angular velocity (rad/s)

        else:
            # We forward cmd_vel commands from turtlebot_teleop_keyboard from cmd_vel to /actor/cmd_vel
            self.actor_sub = rospy.Subscriber('/cmd_vel', Twist, self.turtlebot_cmd_vel_callback)

        rospy.loginfo("actor_node node initialized.")


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
        self.actor_cmd_pub.publish(twist)

        return

    def turtlebot_cmd_vel_callback(self, msg):
        self.actor_cmd_pub.publish(msg)
        return


    def actor_teleport_service(self, req):

        # Read and validate area letter
        area = req.data.strip().upper()

        if area not in list(self.areas_dict.keys()):
            available = ', '.join(sorted(self.areas_dict.keys()))
            return SetStringResponse(
                success=False,
                message="Invalid area '{}'. Available areas: {}".format(
                    area, available
                )
            )

        # Retrieve area coordinates from JSON
        try:
            coordinates = self.areas_dict[area]['coordinates']
            x = float(coordinates['x'])
            y = float(coordinates['y'])

            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("Coordinates must be finite")

        except (KeyError, TypeError, ValueError) as exc:
            return SetStringResponse(
                success=False,
                message="Invalid coordinates for area {}: {}".format(
                    area, exc
                )
            )

        # Prepare Gazebo model state
        model_state = ModelState()
        model_state.model_name = 'actor'
        model_state.reference_frame = 'world'

        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.5

        # Neutral orientation
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0

        # Reset velocity
        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0
        model_state.twist.linear.z = 0.0
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0

        # Call Gazebo service
        try:
            rospy.wait_for_service(
                '/gazebo/set_model_state',
                timeout=5.0
            )

            response = self.set_model_state(model_state)

            if not response.success:
                return SetStringResponse(
                    success=False,
                    message="Gazebo error: {}".format(
                        response.status_message
                    )
                )

            message = "Actor teleported to area {} (x={}, y={})".format(
                area, x, y
            )

            return SetStringResponse(
                success=True,
                message=message
            )

        except (rospy.ROSException, rospy.ServiceException) as exc:
            message = "Failed to teleport actor: {}".format(exc)
            rospy.logerr(message)

            return SetStringResponse(
                success=False,
                message=message
            )


        # Recupera coordinate e richiama /gazebo/set_model_state

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = actorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
