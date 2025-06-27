#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from llm_interface.srv import triggerGpt, triggerGptRequest
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMGoal
import actionlib
import json
import os
import re

script_dir = os.path.dirname(__file__) 


class checkRobotGoalNode():
    def __init__(self):
        rospy.init_node("check_robot_goal_node", anonymous=True)

        with open(f"{script_dir}/../../llm_interface/config/info.json", "r") as f:
            info = json.load(f)

        self.robots_dict = info["ros_publishers"]["robots"]
        self.robots_list = self.robots_dict.keys()

        self.areas_dict = info["areas"]


        self.gazebo_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.actor_model_name = "actor"

        self.goalLLM_action_client = actionlib.SimpleActionClient('/robot_goal_checker', goalCheckerLLMAction)

        self.goal_results_subs_list = []

        for robot in self.robots_list:
            self.goal_results_subs_list.append(rospy.Subscriber(f'/{robot}/move_base/result', MoveBaseActionResult, self.result_callback))


    def result_callback(self, msg):   
        # Estrai il testo del risultato
        goal_status_text = msg.status.text

        # Estrai l'ID del goal
        goal_id = msg.status.goal_id.id

        # Usa una regex per prendere il nome del robot dal goal_id
        match = re.match(r"/([^/]+)/move_base.*", goal_id)
        if match:
            robot_name = match.group(1)
        else:
            robot_name = "unknown"

        
        # Call Gazebo to receive the actor position
        gazebo_req = GetModelStateRequest()
        gazebo_req.model_name = self.actor_model_name

        try:
            response = self.gazebo_client(gazebo_req)

            if response.success:
                actor_pose = response.pose
                x_actor = actor_pose.position.x
                y_actor = actor_pose.position.y

                for area_name, area_info in self.areas_dict.items():
                    x_range = area_info["coordinate_ranges"]["x"]
                    y_range = area_info["coordinate_ranges"]["y"]

                    if x_range["min"] <= x_actor <= x_range["max"] and y_range["min"] <= y_actor <= y_range["max"]:
                        actor_area = area_name
                        break
            
            else:
                rospy.logwarn(f"[{self.model_name}] State not obtained: {response.status_message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Error in the service call: {e}")



        # Call Gazebo to receive the robot position
        gazebo_req = GetModelStateRequest()
        gazebo_req.model_name = robot_name.replace("turtlebot3", "turtlebot")

        try:
            response = self.gazebo_client(gazebo_req)

            if response.success:
                robot_pose = response.pose
                x_robot = robot_pose.position.x
                y_robot = robot_pose.position.y

                for area_name, area_info in self.areas_dict.items():
                    x_range = area_info["coordinate_ranges"]["x"]
                    y_range = area_info["coordinate_ranges"]["y"]

                    if x_range["min"] <= x_robot <= x_range["max"] and y_range["min"] <= y_robot <= y_range["max"]:
                        robot_area = area_name
                        break
            
            else:
                rospy.logwarn(f"[{robot_name}] State not obtained: {response.status_message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Error in the service call: {e}")


        if robot_area == actor_area:
            intruder_identification_text= f"Area {robot_area}: Intruder identified"
        else:
            intruder_identification_text= f"Area {robot_area} clear, no intruders identified"


        LLM_message = f"{robot_name}: {goal_status_text} {intruder_identification_text}"
        # # Call service per avvertire LLM
        # print(f"{robot_name}: {goal_status_text}. {intruder_identification_text}")
        print(LLM_message)

        try:
            action_server_goal = goalCheckerLLMGoal()
            action_server_goal.message_for_LLM = LLM_message

            print(action_server_goal)
            self.goalLLM_action_client.send_goal(action_server_goal)

            rospy.loginfo(f"Goal and intrusion checker: call to LLM executed")

        except rospy.ROSException as e:
            rospy.logerr(f"Error triggering the LLM by the Goal and intrusion checker node: {e}")
            return None


if __name__=="__main__":

    check_robot_goal_node = checkRobotGoalNode()
    rospy.spin()