#!/usr/bin/env python3
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import numpy as np

class ActorDetector:
    def __init__(self):
        rospy.init_node("actor_detector_node")

        self.robot_names = ["turtlebot3_1", "turtlebot3_2"]
        self.actor_name = "actor"

        self.robot_poses = {}
        self.actor_pose = None

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        self.rate = rospy.Rate(10)

    def model_states_callback(self, msg):
        try:
            actor_index = msg.name.index(self.actor_name)
            self.actor_pose = msg.pose[actor_index]

            for robot_name in self.robot_names:
                if robot_name in msg.name:
                    idx = msg.name.index(robot_name)
                    self.robot_poses[robot_name] = msg.pose[idx]
        except ValueError:
            rospy.logwarn("Model not found in gazebo model states")

    def is_actor_in_front_zone(self, robot_pose, actor_pose):
        # Trasforma la posizione dell’actor nel sistema del robot
        dx = actor_pose.position.x - robot_pose.position.x
        dy = actor_pose.position.y - robot_pose.position.y

        # Orientamento del robot (yaw)
        quaternion = (
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # Rotazione inversa per portare il punto nel frame del robot
        rot_matrix = np.array([
            [np.cos(-yaw), -np.sin(-yaw)],
            [np.sin(-yaw),  np.cos(-yaw)],
        ])
        local_pos = np.dot(rot_matrix, np.array([dx, dy]))

        x_local, y_local = local_pos

        # Controlla se è nella zona 2x2 metri davanti al robot (x: 0-2, y: -1 a 1)
        return 0.0 <= x_local <= 2.0 and -1.0 <= y_local <= 1.0




    # DA MODIFICARE E RISCRIVERE MEGLIO
    def run(self):
        while not rospy.is_shutdown():
            if self.actor_pose and all(robot in self.robot_poses for robot in self.robot_names):
                for robot_name in self.robot_names:
                    in_zone = self.is_actor_in_front_zone(self.robot_poses[robot_name], self.actor_pose)
                    rospy.loginfo(f"{robot_name} sees actor: {in_zone}")
            self.rate.sleep()

if __name__ == "__main__":
    try:
        detector = ActorDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
