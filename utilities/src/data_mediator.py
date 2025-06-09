#!/usr/bin/env python3
import rospy
import json
from OpenAI_interface.srv import triggerGpt, triggerGptRequest, retrieveSystemState, retrieveSystemStateResponse
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates


class Node():

    def __init__(self):

        rospy.init_node("Data_mediator",anonymous=True)


        ###### IT IS JUST USED TO MANUALLY SET THE BATTERY LEVEL OF EACH ROBOT ######

        self.scenario_number = rospy.get_param("/scenario")

        if self.scenario_number == 1:
            self.turtlebot_1_battery = 60
            self.turtlebot_2_battery = 60

        elif self.scenario_number == 2:
            self.turtlebot_1_battery = 60
            self.turtlebot_2_battery = 60

        elif self.scenario_number == 3:
            self.turtlebot_1_battery = 5            # with low battery the right robot should be turtlebot_2 
            self.turtlebot_2_battery = 60 

        else:
            self.activated_sensors = []

        ######################################################################################

        self.message = {
            "robots_list_names": [],
            "robots":
                {
                    "turtlebot_1":{
                        "current_position": [],
                        "current_orientation": [],
                        "battery": self.turtlebot_1_battery
                        },
                    "turtlebot_2":{
                        "current_position": [],
                        "current_orientation": [],
                        "battery": self.turtlebot_2_battery
                        }
                    },
            "sensors_list_names": [],
            "sensors":
                {
                    "Camera_1":{
                        "position": [],
                        "orientation": [],
                        },
                    "Camera_2":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_1":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_2":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_3":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_4":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_5":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_6":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_7":{
                        "position": [],
                        "orientation": [],
                        },
                    "Lidar_8":{
                        "position": [],
                        "orientation": [],
                        } 
                    },
            "actuators_list_names": [],
            "actuators":
                {
                    "door_1":{
                        "position": [],
                        "state": "",
                        },
                    "door_2":{
                        "position": [],
                        "state": "",
                        },
                    "door_3":{
                        "position": [],
                        "state": "",
                        },
                    "door_4":{
                        "position": [],
                        "state": "",
                        },
                    "door_5":{
                        "position": [],
                        "state": "",
                        },
                    "door_6":{
                        "position": [],
                        "state": "",
                        },
                    "door_7":{
                        "position": [],
                        "state": "",
                        },
                    "door_8":{
                        "position": [],
                        "state": "",
                        },
                    "door_9":{
                        "position": [],
                        "state": "",
                        }     
                    },
            "activated_sensors": []
        }

        self.previous_length = 0 # It represents the number of default activated sensors (it is used to check if new sensors are activated in order to trigger the LLM every time that one sensors is activated)

        self.msg_gazebo_models = ModelStates()

        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.publishData, queue_size=1)
        self.activated_sensors_subscriber = rospy.Subscriber("/activated_sensors", String, self.updateActivatedSensors, queue_size=1)
        self.publisher = rospy.Publisher('/data', String, queue_size=1)
        self.client_service = rospy.ServiceProxy('/alert', triggerGpt)
        self.server = rospy.Service("/retrieve_system_state", retrieveSystemState, self.retrieve_system_state)


        for i in range(1,10):
            topic = f"/door_{i}/door_state"
            subscriber = rospy.Subscriber(topic, String, self.extract_actuator_state(index = i), queue_size=1)


        
        self.rate=rospy.Rate(1)

    def updateActivatedSensors(self, activatedSensors_msg):
        activatedSensors = json.loads(activatedSensors_msg.data)
        self.message["activated_sensors"] = activatedSensors
        
        
    def triggerLLM(self):

        try:
            request = triggerGptRequest(json.dumps(self.message)) # self.message is converted as string and sent as request to the LLM
            response = self.client_service(request)
            rospy.loginfo(f"Call executed")
            rospy.loginfo(f"Server response: {response.received}")
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Error triggering the LLM by the data mediator: {e}")
            return None
        
    
    def retrieve_system_state(self, req):
        return retrieveSystemStateResponse(json.dumps(self.message)) 
    
    def extract_actuator_state(self, index):
        # To dynamically extract the state for each door and update the self.message, it creates a dynamic callback according to the considered door. To avoid this
        # mechanism it is necessary to create a custom message with fields:
        #   - model_name (string)
        #   - state (string)
        # Use this message from the gazebo plugin to continuosly publish such data and directly extract this data inside a single callback

        def callback(msg):
            state = msg.data
            actuator_name = f"door_{index}"

            self.message["actuators"][actuator_name]["state"] = state
        return callback


    def publishData(self, gazeboMsgData):
        """This function is used to continuosly extract information from Gazebo and update environmental information stored inside a predefined message.
        This message is then sent:
        - by a publisher --> it provides information on topic /data that will be used by the sensorVisualizationMarker.py
        - by the service TriggerLLM() --> in case of activated sensors, in order to inform the LLM"""
        self.msg_gazebo_models = gazeboMsgData

        

        turtlebot_indices = [index for index, model_name in enumerate(self.msg_gazebo_models.name) if "turtlebot" in model_name]
        sensors_indices = [index for index, model_name in enumerate(self.msg_gazebo_models.name) if "Lidar" in model_name or "Camera" in model_name]
        actuators_indices = [index for index, model_name in enumerate(self.msg_gazebo_models.name) if "door" in model_name]


        turtlebot_list_names = []
        sensors_list_names = []
        actuators_list_names = []
        
        for index in turtlebot_indices:
            turtlebot_list_names.append(self.msg_gazebo_models.name[index])

        for index in sensors_indices:
            sensors_list_names.append(self.msg_gazebo_models.name[index])
        
        for index in actuators_indices:
            actuators_list_names.append(self.msg_gazebo_models.name[index])

        

        self.message["robots_list_names"] = sorted(turtlebot_list_names)
        self.message["sensors_list_names"] = sorted(sensors_list_names)
        self.message["actuators_list_names"] = sorted(actuators_list_names)


        # Extraction of robots positions and orientation information
        for turtlebot_name, turtlebot_index in zip(turtlebot_list_names, turtlebot_indices):

            if turtlebot_name in ["turtlebot_1", "turtlebot_2"]:
                turtlebot_pose =  self.msg_gazebo_models.pose[turtlebot_index]

                self.message["robots"][turtlebot_name]["current_position"] = [round(turtlebot_pose.position.x, 2), round(turtlebot_pose.position.y, 2), round(turtlebot_pose.position.z,2)] 
                self.message["robots"][turtlebot_name]["current_orientation"] = [round(turtlebot_pose.orientation.x, 2), round(turtlebot_pose.orientation.y, 2), round(turtlebot_pose.orientation.z, 2), round(turtlebot_pose.orientation.w,2)] 
                
            else:
                continue

        
        # Extraction of sensors positions and orientation information
        for sensor_name, sensor_index in zip(sensors_list_names, sensors_indices):

            sensor_pose =  self.msg_gazebo_models.pose[sensor_index]

            self.message["sensors"][sensor_name]["position"] = [round(sensor_pose.position.x, 2), round(sensor_pose.position.y, 2), round(sensor_pose.position.z, 2)] 
            self.message["sensors"][sensor_name]["orientation"] = [round(sensor_pose.orientation.x, 2), round(sensor_pose.orientation.y, 2), round(sensor_pose.orientation.z, 2), round(sensor_pose.orientation.w, 2)] 
    

        # Extraction of actuators positions and states:
        for actuator_name, actuator_index in zip(actuators_list_names, actuators_indices):

            actuator_pose =  self.msg_gazebo_models.pose[actuator_index]

            self.message["actuators"][actuator_name]["position"] = [round(actuator_pose.position.x, 2), round(actuator_pose.position.y, 2), round(actuator_pose.position.z, 2)] 
        

        str_msg = json.dumps(self.message)
        try:    
            self.publisher.publish(str_msg)
        except Exception as e:
            print(e)


        # To deactivate sensors, user calls a service to the "sensor_activation_node". This means that, after the reset of all sensors, the "previous_length" variable remain at the same value of the last number of activated sensors.
        # By using this condition, we can propagate the reset from the "sensors_activation_node" to the "data_mediator". Every time the number of activated sensors is 0, also the "previous_length" will be 0 
        if len(self.message.get("activated_sensors")) == 0:
            self.previous_length = 0

        # Every time a new sensor is activated, the data mediator inform the LLM. Since only the operator can deactivate the sensors, the check is performed on the length of the list of all activated sensors 
        if len(self.message.get("activated_sensors")) != self.previous_length:
            self.previous_length = len(self.message["activated_sensors"]) # update of the actual length of the list of activated sensors
            self.triggerLLM()


if __name__=="__main__":

    data_mediator=Node()
    #data_mediator.publishData()
    rospy.spin()