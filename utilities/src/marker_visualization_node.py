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

        # Get the world_id parameter from ROS parameter server
        self.world_id = str(rospy.get_param('world_id', '1'))

        # Opening the correct Json configuration file
        with open(f"{script_dir}/../../llm_interface/config/info_map{self.world_id}.json", "r") as f:
            info = json.load(f)

        self.areas_dict = info["areas"]
        self.areas_names = self.areas_dict.keys()

        # Dictionary for RViz marker offset: 
        # structure of the dictionary: { mode: { model_name: (offset_x, offset_y, offset_z) } }
        self.textual_offsets = {
            "1": {
                "Camera_1": (-0.5, -0.25, 0.0),
                "Camera_2": (0.5, -0.25, 0.0),
                "Lidar_4": (0.6, -0.25, 0.0), 
                "door_1": (0.0, 0.25, 0.0), 
                "door_3": (0.5, 0.25, 0.0), 
                "door_5": (-0.25, -0.25, 0.0),
                "door_6": (-0.2, -0.25, 0.0), 
                "door_8": (0.2, -0.25, 0.0), 
                "default_door": (0.5, -0.25, 0.0),
                "default": (0.0, -0.25, 0.0)
            },
            "2": {
                "default_door": (0.5, -0.25, 0.0),
                "default": (0.0, -0.25, 0.0)
                }
        }

        # Definition of horizontal doors for each specific environment
        self.horizontal_doors_map = {
            '1': {"2", "3", "5", "6", "8"},
            '2': {}
        }

        # Retrieving the set of horizontal doors for the corresponding world (empty set by default)
        self.horiz_set = self.horizontal_doors_map.get(self.world_id, set())

        self.subscriber = rospy.Subscriber("/data", String, self.publishMarkerArrayCallback, queue_size=1)
        self.publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)
        
        self.rate=rospy.Rate(10)



    def publishMarkerArrayCallback(self, data_json_string):
        self.msg_marker_array = MarkerArray()
        
        self.data = json.loads(data_json_string.data)

        sensors_list = self.data["sensors_list_names"]
        robots_list = self.data["robots_list_names"]
        actuator_list = self.data["actuators_list_names"]
        actuator_data = self.data["actuators"]
        activated_sensors = self.data["activated_sensors"]

        models_list = sensors_list + robots_list + actuator_list
        counter = 0

        for model_name in models_list:
            # 1. Retrieving Gazebo models information 
            if model_name in sensors_list:
                model_position = self.data["sensors"][model_name]["position"]
                model_orientation = self.data["sensors"][model_name]["orientation"]
            elif model_name in robots_list:
                model_position = self.data["robots"][model_name]["current_position"]
                model_orientation = self.data["robots"][model_name]["current_orientation"]
            else:
                model_position = self.data["actuators"][model_name]["position"]

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

            if model_name in list(self.textual_offsets[self.world_id].keys()):
                text_off_x, text_off_y, text_off_z = self.textual_offsets[self.world_id][model_name]
            else:
                if "door" in model_name:
                    text_off_x, text_off_y, text_off_z = self.textual_offsets[self.world_id]["default_door"]
                else:
                    text_off_x, text_off_y, text_off_z = self.textual_offsets[self.world_id]["default"]
            
            self.msg_marker_text.pose.position.x = model_position[0] + text_off_x
            self.msg_marker_text.pose.position.y = model_position[1] + text_off_y
            self.msg_marker_text.pose.position.z = 0.0 + text_off_z

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

                self.msg_marker_position.pose.position.z = model_position[2]

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
            
            elif "door" in model_name:
                # In the world_1 we skip the first door
                if "1" in model_name and self.world_id=="1":
                    counter += 1
                    continue
                
                self.msg_marker_position.type = 1 # 1 = cube 
                self.msg_marker_position.id = counter+500 # unique identifier for door position markers 
                self.msg_marker_position.pose.position.z = 0

                

                # Check if the current door must be represented horizontally: 
                # it extracts the number from the name and check in the corresponding set
                is_horizontal = any(num in model_name for num in self.horiz_set)

                # Check if the door is closed
                is_closed = actuator_data[model_name]["state"] == "close"
                # Scale: 1.0 if closed (blu bar), 0.0 if open (no bar)
                val = 1.0 if is_closed else 0.0

                if is_horizontal:
                    self.msg_marker_position.pose.orientation.x = 0
                    self.msg_marker_position.pose.orientation.y = 0
                    self.msg_marker_position.pose.orientation.z = 0
                    self.msg_marker_position.pose.orientation.w = 1
                    
                    self.msg_marker_position.scale.x = val
                    self.msg_marker_position.scale.y = 0.1 if is_closed else 0.0
                    self.msg_marker_position.scale.z = 0.1 if is_closed else 0.0
                else:
                    # Orientamento Verticale
                    self.msg_marker_position.pose.orientation.x = 0
                    self.msg_marker_position.pose.orientation.y = 0.707
                    self.msg_marker_position.pose.orientation.z = 0
                    self.msg_marker_position.pose.orientation.w = 0.707

                    self.msg_marker_position.scale.x = 0.1 if is_closed else 0.0
                    self.msg_marker_position.scale.y = val
                    self.msg_marker_position.scale.z = 0.1 if is_closed else 0.0

                self.msg_marker_position.color.r = 0.0
                self.msg_marker_position.color.g = 0.0
                self.msg_marker_position.color.b = 1.0
                self.msg_marker_position.color.a = 1.0 if is_closed else 0.0
            elif "turtlebot" in model_name:
                counter += 1
                continue

            else:
                rospy.logwarn(f"Unknown model type: {model_name}")
            
            self.msg_marker_position.header.stamp = rospy.Time.now()

            self.msg_marker_position.pose.position.x = model_position[0]
            self.msg_marker_position.pose.position.y = model_position[1]
            
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

