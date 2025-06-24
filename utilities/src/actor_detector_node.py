#!/usr/bin/env python3
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import numpy as np
from tf.transformations import quaternion_matrix


class ActorDetectorNode:
    def __init__(self):
        rospy.init_node("actor_detector_node", anonymous=True)

        # Nomi parziali per matching più robusto
        self.robot_names = ["turtlebot3_1", "turtlebot3_2"]
        self.actor_name = "actor"

        self.robot_poses = {}
        self.actor_pose = None

        # Subscriber
        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.actor_detection, queue_size=1)

        # Rate di ciclo
        self.rate = rospy.Rate(10)


    def actor_detection(self, msg_gazebo_models):

        # Extraction of all the robots name
        robots_indices = [index for index, model_name in enumerate(msg_gazebo_models.name) if "turtlebot" in model_name]
        robots_list_names = []
        for index in robots_indices:
            robots_list_names.append(msg_gazebo_models.name[index])

        # Extraction of all the actors name
        actors_indices = [index for index, model_name in enumerate(msg_gazebo_models.name) if "actor" in model_name]
        actors_list_names = []
        for index in actors_indices:
            actors_list_names.append(msg_gazebo_models.name[index])


        for actor_name, actor_index in zip(actors_list_names, actors_indices):
            actor_pose = msg_gazebo_models.pose[actor_index]
            actor_position = np.array([round(actor_pose.position.x, 2), round(actor_pose.position.y, 2), round(actor_pose.position.z, 2)])     

            for robot_name, robot_index in zip(robots_list_names, robots_indices):
                robot_pose =  msg_gazebo_models.pose[robot_index]
                robot_position = np.array([round(robot_pose.position.x, 2), round(robot_pose.position.y, 2), round(robot_pose.position.z, 2)]) 
                robot_orientation_quat = [round(robot_pose.orientation.x, 2), round(robot_pose.orientation.y, 2), round(robot_pose.orientation.z, 2), round(robot_pose.orientation.w, 2)] 
                
                rotation_matrix = quaternion_matrix(robot_orientation_quat)[:3,:3]
                relative_actor_position = actor_position-robot_position

                actor_position_from_robot = np.dot(rotation_matrix.T, relative_actor_position)

                
                if (actor_position_from_robot[0]>= 0 and actor_position_from_robot[0]<= 2) and (actor_position_from_robot[1]>= -1 and actor_position_from_robot[1]<= 1):
                    rospy.loginfo(f"{robot_name}: intrusion detected")
                    print(actor_position_from_robot)


if __name__ == "__main__":
    actor_detector_node=ActorDetectorNode()
    rospy.spin() 