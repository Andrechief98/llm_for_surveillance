#!/usr/bin/env python3
import rospy
from  visualization_msgs.msg import Marker
from  visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Range
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
import json
import os


script_dir = os.path.dirname(__file__)


class Node():

    def __init__(self):

        rospy.init_node("marker_visualization_node",anonymous=True)


        self.subscriber = rospy.Subscriber("/data", String, self.publishMarkerArrayCallback, queue_size=1)
        self.publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)
        
        self.rate=rospy.Rate(10)

        with open(f"{script_dir}/../../llm_interface/config/info.json", "r") as f:
            info = json.load(f)
    
        self.areas_dict = info["areas"]

        self.areas_names = self.areas_dict.keys()



    def publishMarkerArrayCallback(self, data_json_string):
        self.msg_marker_array = MarkerArray()
        
        self.data = json.loads(data_json_string.data)

        sensors_list = self.data["sensors_list_names"]
        robots_list = self.data["robots_list_names"]
        actuator_list = self.data["actuators_list_names"]

        models_list = sensors_list + robots_list + actuator_list
        activated_sensors = self.data["activated_sensors"]
        # print(activated_sensors)

        counter = 0
        # print(gazeboMsgData)

        for model_name in models_list:

            if model_name in sensors_list:
                model_position = self.data["sensors"][model_name]["position"]
                model_orientation = self.data["sensors"][model_name]["orientation"]
            elif model_name in robots_list:
                model_position = self.data["robots"][model_name]["current_position"]
                model_orientation = self.data["robots"][model_name]["current_orientation"]
            else:
                model_position = self.data["actuators"][model_name]["position"]

            model_position_x, model_position_y, model_position_z = model_position
            model_orientation_x, model_orientation_y, model_orientation_z, model_orientation_w = model_orientation
            

            # Create the text marker
            self.msg_marker_text = Marker()
            self.msg_marker_text.header.frame_id = 'map'

            self.msg_marker_text.type = 9 # 9 = text

            self.msg_marker_text.pose.orientation.x = 0.0
            self.msg_marker_text.pose.orientation.y = 0.0
            self.msg_marker_text.pose.orientation.z = 0.0
            self.msg_marker_text.pose.orientation.w = 1.0

     
            self.msg_marker_text.scale.z = 0.25
            self.msg_marker_text.text = model_name
            
            self.msg_marker_text.color.r = 0.0
            self.msg_marker_text.color.g = 0.0
            self.msg_marker_text.color.b = 0.0
            self.msg_marker_text.color.a = 1.0
            self.msg_marker_text.header.stamp = rospy.Time.now()
            self.msg_marker_text.id = counter # unique identifier for the text markers 

            if model_name == "Camera_1":
                self.msg_marker_text.pose.position.x = model_position_x-0.5
            elif model_name == "Camera_2":
                self.msg_marker_text.pose.position.x = model_position_x+0.5
            elif model_name == "door_1":
                self.msg_marker_text.pose.position.y = model_position_y+0.5
            elif model_name == "door_2" or model_name == "door_3" or model_name == "door_4" or model_name == "door_7" or model_name == "door_9":
                self.msg_marker_text.pose.position.x = model_position_x+0.5
            else:
                self.msg_marker_text.pose.position.x = model_position_x

            self.msg_marker_text.pose.position.y = model_position_y-0.25
            self.msg_marker_text.pose.position.z = 0

            self.msg_marker_array.markers.append(self.msg_marker_text)



            # Create a position marker
            self.msg_marker_position = Marker()
            self.msg_marker_position.header.frame_id = 'map'

            if "Lidar" in model_name:
                # Create a lidar position marker (Cube)
                self.msg_marker_position.type = 1 # 1 = cube
                self.msg_marker_position.id = counter+500 #unique identifier for the lidar position markers 

                self.msg_marker_position.pose.orientation.x = 0.0
                self.msg_marker_position.pose.orientation.y = 0.0
                self.msg_marker_position.pose.orientation.z = 0.0
                self.msg_marker_position.pose.orientation.w = 1.0
                
                self.msg_marker_position.scale.x = 0.15
                self.msg_marker_position.scale.y = 0.15
                self.msg_marker_position.scale.z = 0.15

                self.msg_marker_position.pose.position.z = model_position_z

            elif "Camera" in model_name:
                # Create a camera position marker (Arrow)
                self.msg_marker_position.type = 0 # 0 = arrow
                self.msg_marker_position.id = counter+1000 # unique identifier for camera position markers 

                self.msg_marker_position.pose.position.z = 0

                self.msg_marker_position.pose.orientation.x = model_orientation_x
                self.msg_marker_position.pose.orientation.y = model_orientation_y
                self.msg_marker_position.pose.orientation.z = model_orientation_z
                self.msg_marker_position.pose.orientation.w = model_orientation_w
                
                self.msg_marker_position.scale.x = 2.50 # arrow length
                self.msg_marker_position.scale.y = 0.2 # arrow width
                self.msg_marker_position.scale.z = 0.2 # arrow height
            
            elif "door" in model_name:
                counter += 1
                continue

            elif "turtlebot" in model_name:
                counter += 1
                continue

            else:
                print("Error")
            

            if model_name in activated_sensors:
                self.msg_marker_position.color.r = 0.0
                self.msg_marker_position.color.g = 1.0
                self.msg_marker_position.color.b = 0.0
                self.msg_marker_position.color.a = 1.0
            else:
                self.msg_marker_position.color.r = 1.0
                self.msg_marker_position.color.g = 0.0
                self.msg_marker_position.color.b = 0.0
                self.msg_marker_position.color.a = 1.0

            
            self.msg_marker_position.header.stamp = rospy.Time.now()
            

            self.msg_marker_position.pose.position.x = model_position_x
            self.msg_marker_position.pose.position.y = model_position_y
            
            self.msg_marker_array.markers.append(self.msg_marker_position)
            
            counter += 1
        


        for area_name in self.areas_names:

            # Text marker for areas
            self.msg_marker_text = Marker()
            self.msg_marker_text.header.frame_id = 'map'

            self.msg_marker_text.type = 9 # 9 = text
            self.msg_marker_text.text = area_name

            area_x = self.areas_dict[area_name]["coordinates"]["x"]
            area_y = self.areas_dict[area_name]["coordinates"]["y"]


            self.msg_marker_text.pose.orientation.x = 0.0
            self.msg_marker_text.pose.orientation.y = 0.0
            self.msg_marker_text.pose.orientation.z = 0.0
            self.msg_marker_text.pose.orientation.w = 1.0

                 
            self.msg_marker_text.color.r = 24.0 / 255.0
            self.msg_marker_text.color.g = 102.0 / 255.0
            self.msg_marker_text.color.b = 138.0 / 255.0
            self.msg_marker_text.color.a = 1.0            

            self.msg_marker_text.pose.position.x = area_x
            self.msg_marker_text.pose.position.y = area_y
            self.msg_marker_text.pose.position.z = 0.5

            self.msg_marker_text.scale.x = 0.8 # length
            self.msg_marker_text.scale.y = 0.8 # width
            self.msg_marker_text.scale.z = 0.8 # width

            self.msg_marker_text.header.stamp = rospy.Time.now()
            self.msg_marker_text.id = counter + 2000 # unique identifier for the area text markers 

            self.msg_marker_array.markers.append(self.msg_marker_text)

            counter += 1

        self.publisher.publish(self.msg_marker_array)
        #self.rate.sleep()



if __name__=="__main__":

    MarkerVisualizer=Node()
    rospy.spin()

