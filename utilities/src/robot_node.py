#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from llm_interface.srv import triggerGpt, triggerGptRequest
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMGoal
from utilities.srv import TeleportRobot, TeleportRobotResponse
import actionlib
import json
import os
import re


script_dir = os.path.dirname(__file__) 


class robotNode():
    def __init__(self):
        rospy.init_node("robot_node", anonymous=True)

        # Get the world_id parameter from ROS parameter server
        self.world_id = rospy.get_param('world_id', '1')

        with open(f"{script_dir}/../../llm_interface/config/ROS/info_map{self.world_id}.json", "r") as f:
            info = json.load(f)

        self.robots_dict = info["ros_publishers"]["robots"]
        self.robots_list = self.robots_dict.keys()

        self.initial_pose_publishers = {}

        # Retrieve initial poses of all robots from ROS Parameter Server
        self.initial_poses = {}

        for robot_name in self.robots_list:
            param_name = f"/{robot_name}_initial_pose"
            pose_str = rospy.get_param(param_name)
            x, y, yaw = map(float, pose_str.split())
            self.initial_poses[robot_name] = {
                "x": x,
                "y": y,
                "yaw": yaw
            }


        self.areas_dict = info["areas"]

        self.reset_robot_pose_server = rospy.Service('/reset_robot_pose', Empty, self.reset_robot_pose)
        self.teleport_robot_server = rospy.Service('/teleport_robot', TeleportRobot, self.teleport_callback)

        self.gazebo_client_get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.gazebo_client_set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.actor_model_name = "actor"

        self.goalLLM_action_client = actionlib.SimpleActionClient('/robot_goal_checker', goalCheckerLLMAction)

        self.goal_results_subs_list = []

        for robot in self.robots_list:
            self.goal_results_subs_list.append(rospy.Subscriber(f'/{robot}/move_base/result', MoveBaseActionResult, self.result_callback))


    def reset_robot_pose(self, req):

        try:
            for robot_name, pose in self.initial_poses.items():

                # Create ModelState message
                model_state = ModelState()

                model_state.model_name = robot_name
                model_state.reference_frame = "world"

                # Set initial position
                model_state.pose.position.x = pose["x"]
                model_state.pose.position.y = pose["y"]
                model_state.pose.position.z = 0.0

                # Convert yaw to quaternion
                q = quaternion_from_euler(0.0, 0.0, pose["yaw"])

                model_state.pose.orientation.x = q[0]
                model_state.pose.orientation.y = q[1]
                model_state.pose.orientation.z = q[2]
                model_state.pose.orientation.w = q[3]

                # Reset linear and angular velocities
                model_state.twist.linear.x = 0.0
                model_state.twist.linear.y = 0.0
                model_state.twist.linear.z = 0.0

                model_state.twist.angular.x = 0.0
                model_state.twist.angular.y = 0.0
                model_state.twist.angular.z = 0.0

                # Call Gazebo set_model_state service
                response = self.gazebo_client_set_state(model_state)

                if response.success:
                    rospy.loginfo(
                        f"[{robot_name}] Position successfully reset "
                        f"to x={pose['x']}, y={pose['y']}, yaw={pose['yaw']}"
                    )
                else:
                    rospy.logwarn(
                        f"[{robot_name}] Reset failed: {response.status_message}"
                    )

        except rospy.ServiceException as e:
            rospy.logerr(f"Error resetting robot positions: {e}")
            return EmptyResponse()

        return EmptyResponse()

    def teleport_callback(self, req):

        try:
            robot_name = req.robot_name
            area = req.area

            # Recupera coordinate dell'area
            area_x = self.areas_dict[area]["coordinates"]["x"]-0.3  # small offset to allow the robot to actually reach the goal triggering the move_base
            area_y = self.areas_dict[area]["coordinates"]["y"]-0.3  # small offset to allow the robot to actually reach the goal triggering the move_base

            # Prepara stato del modello Gazebo
            model_state = ModelState()
            model_state.model_name = robot_name

            model_state.pose.position.x = area_x 
            model_state.pose.position.y = area_y
            model_state.pose.position.z = 0.0

            # Orientation 
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.0
            model_state.pose.orientation.w = 1.0

            # Null linear and angular velocity
            model_state.twist.linear.x = 0.0
            model_state.twist.linear.y = 0.0
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0

            model_state.reference_frame = "map"

            result = self.gazebo_client_set_state(model_state)

            if not result.success:
                raise RuntimeError(result.status_message)

            # 2. Pubblica nuova initialpose
            initial_pose = PoseWithCovarianceStamped()

            initial_pose.header.stamp = rospy.Time.now()
            initial_pose.header.frame_id = "map"

            initial_pose.pose.pose.position.x = area_x
            initial_pose.pose.pose.position.y = area_y
            initial_pose.pose.pose.position.z = 0.0

            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = 0.0
            initial_pose.pose.pose.orientation.w = 1.0

            # Covarianza iniziale
            initial_pose.pose.covariance[0] = 0.25
            initial_pose.pose.covariance[7] = 0.25
            initial_pose.pose.covariance[35] = 0.0685

            # Publisher specifico del robot
            initial_pose_topic = f"/{robot_name}/initialpose"

            if robot_name not in self.initial_pose_publishers:

                self.initial_pose_publishers[robot_name] = rospy.Publisher(
                    initial_pose_topic,
                    PoseWithCovarianceStamped,
                    queue_size=1
                )

            publisher = self.initial_pose_publishers[robot_name]
            publisher.publish(initial_pose)

            rospy.loginfo(
                f"Robot {robot_name} teleported to {area} "
                f"({area_x}, {area_y})"
            )


            clear_costmaps_service = f"/{robot_name}/move_base/clear_costmaps"

            rospy.wait_for_service(clear_costmaps_service)

            clear_costmaps = rospy.ServiceProxy(
                clear_costmaps_service,
                Empty
            )

            clear_costmaps()

            return TeleportRobotResponse(
                success=True,
                message=f"{robot_name} teleported to {area}"
            )

        except Exception as e:
            rospy.logerr(f"Teleport error: {e}")

            return TeleportRobotResponse(
                success=False,
                message=str(e)
            )
    
    def result_callback(self, msg):   
        # Estrai il testo del risultato
        # print(msg)
        goal_status_text = msg.status.text
        goal_status = msg.status.status

        # Estrai l'ID del goal
        goal_id = msg.status.goal_id.id

        # Usa una regex per prendere il nome del robot dal goal_id
        match = re.match(r"/([^/]+)/move_base.*", goal_id)
        if match:
            robot_name = match.group(1)
        else:
            robot_name = "unknown"
            
            
        if goal_status == 3: # SUCCESS

            # Call Gazebo to receive the actor position
            gazebo_req = GetModelStateRequest()
            gazebo_req.model_name = self.actor_model_name

            try:
                response = self.gazebo_client_get_state(gazebo_req)

                if response.success:
                    actor_pose = response.pose
                    x_actor = actor_pose.position.x
                    y_actor = actor_pose.position.y

                    for area_name, area_info in self.areas_dict.items():
                        if area_name == "OUT":
                            actor_area = "OUT"
                        else:
                            x_range = area_info["coordinate_ranges"]["x"]
                            y_range = area_info["coordinate_ranges"]["y"]

                            if x_range["min"] <= x_actor <= x_range["max"] and y_range["min"] <= y_actor <= y_range["max"]:
                                actor_area = area_name
                                break
                            else:
                                actor_area = "unknown"
                
                else:
                    rospy.logwarn(f"[{self.actor_model_name}] State not obtained: {response.status_message}")

            except rospy.ServiceException as e:
                rospy.logerr(f"Error in the service call: {e}")



            # Call Gazebo to receive the robot position
            gazebo_req = GetModelStateRequest()
            gazebo_req.model_name = robot_name

            try:
                response = self.gazebo_client_get_state(gazebo_req)

                if response.success:
                    robot_pose = response.pose
                    x_robot = robot_pose.position.x
                    y_robot = robot_pose.position.y

                    for area_name, area_info in self.areas_dict.items():
                        if area_name == "OUT":
                            continue
                        else:
                            x_range = area_info["coordinate_ranges"]["x"]
                            y_range = area_info["coordinate_ranges"]["y"]

                            if x_range["min"] <= x_robot <= x_range["max"] and y_range["min"] <= y_robot <= y_range["max"]:
                                robot_area = area_name
                                break
                            else:
                                robot_area = "uknown"
                
                else:
                    rospy.logwarn(f"[{robot_name}] State not obtained: {response.status_message}")

            except rospy.ServiceException as e:
                rospy.logerr(f"Error in the service call: {e}")


            if robot_area == actor_area:
                intruder_identification_text= f"Area {robot_area}: Intruder identified"
            else:
                intruder_identification_text= f"Area {robot_area} clear, no intruders identified"


            LLM_message = f"{robot_name}: {goal_status_text} {intruder_identification_text}"

            print(LLM_message)

            # Call action to send the robot's report to the LLM
            try:
                action_server_goal = goalCheckerLLMGoal()
                action_server_goal.message_for_LLM = LLM_message

                print(action_server_goal)
                self.goalLLM_action_client.send_goal(action_server_goal)

                rospy.loginfo(f"Goal and intrusion check: call to LLM executed")

            except rospy.ROSException as e:
                rospy.logerr(f"Error triggering the LLM by the Goal and intrusion checker node: {e}")
                return None
        else:
            # Call action to tell the LLM the robot failed to reach the goal
            LLM_message = f"{robot_name}: {goal_status_text}"

            try:
                action_server_goal = goalCheckerLLMGoal()
                action_server_goal.message_for_LLM = LLM_message

                print(action_server_goal)
                self.goalLLM_action_client.send_goal(action_server_goal)

                rospy.loginfo(f"Goal and intrusion check: call to LLM executed")

            except rospy.ROSException as e:
                rospy.logerr(f"Error triggering the LLM by the Goal and intrusion checker node: {e}")
                return None

if __name__=="__main__":

    check_robot_goal_node = robotNode()
    rospy.spin()