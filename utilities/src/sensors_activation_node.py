#!/usr/bin/env python3
import rospy
import json
from OpenAI_interface.srv import triggerGpt, triggerGptRequest, retrieveSystemState, retrieveSystemStateResponse
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse 
from tf.transformations import quaternion_matrix
from gazebo_msgs.msg import ModelStates
import numpy as np


class Node():
    def __init__(self):

        rospy.init_node("sensors_activation_node",anonymous=True)
        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.publishActivatedSensors, queue_size=1)
        self.activated_sensors_publisher = rospy.Publisher("/activated_sensors",String, queue_size=1)
        self.server = rospy.Service("/resetSensorActivation", Empty, self.reset_sensor_activation)
        self.activated_sensors = []

    def publishActivatedSensors(self, gazeboMsgData):
        self.msg_gazebo_models = gazeboMsgData

        # SENSORS CONTROL LOGIC
        # Extracion of all the sensors name
        sensors_indices = [index for index, model_name in enumerate(self.msg_gazebo_models.name) if "Lidar" in model_name or "Camera" in model_name]
        sensors_list_names = []
        for index in sensors_indices:
            sensors_list_names.append(self.msg_gazebo_models.name[index])

        # Extraction of all the actors name
        actors_indices = [index for index, model_name in enumerate(self.msg_gazebo_models.name) if "actor" in model_name]
        actors_list_names = []
        for index in actors_indices:
            actors_list_names.append(self.msg_gazebo_models.name[index])


        for actor_name, actor_index in zip(actors_list_names, actors_indices):
            actor_pose = self.msg_gazebo_models.pose[actor_index]
            actor_position = np.array([round(actor_pose.position.x, 2), round(actor_pose.position.y, 2), round(actor_pose.position.z, 2)]) 
            
            for sensor_name, sensor_index in zip(sensors_list_names, sensors_indices):
                sensor_pose =  self.msg_gazebo_models.pose[sensor_index]
                sensor_position = np.array([round(sensor_pose.position.x, 2), round(sensor_pose.position.y, 2), round(sensor_pose.position.z, 2)]) 
                sensor_orientation_quat = [round(sensor_pose.orientation.x, 2), round(sensor_pose.orientation.y, 2), round(sensor_pose.orientation.z, 2), round(sensor_pose.orientation.w, 2)] 
                
                rotation_matrix = quaternion_matrix(sensor_orientation_quat)[:3,:3]
                relative_actor_position = actor_position-sensor_position

                actor_position_from_sensor = np.dot(rotation_matrix.T, relative_actor_position)

                if sensor_name not in self.activated_sensors:
                    if "Lidar" in sensor_name:
                        # Check if the actor is in the area of the Lidar. 
                        # We check if the actor is in an area of [0, 1.2]m along the x axis and [-0.1, 0.1]m along the y axis
                        if (actor_position_from_sensor[0]>= 0 and actor_position_from_sensor[0]<= 1.2) and (actor_position_from_sensor[1]>= -0.15 and actor_position_from_sensor[1]<= 0.15):
                            self.activated_sensors.append(sensor_name)
                    elif "Camera" in sensor_name:
                        # We check if the actor is in an area of [0, 5]m along the x axis and [-1, 1]m along the y axis
                        if (actor_position_from_sensor[0]>= 0 and actor_position_from_sensor[0]<= 5) and (actor_position_from_sensor[1]>= -1 and actor_position_from_sensor[1]<= 1):
                            self.activated_sensors.append(sensor_name)
                    else:
                        continue
                else:
                    continue


        str_msg = json.dumps(self.activated_sensors)
        self.activated_sensors_publisher.publish(str_msg)

    def reset_sensor_activation(self, req):
        self.activated_sensors = []
        return EmptyResponse()
    

if __name__=="__main__":

    sensors_activation_node=Node()
    rospy.spin()