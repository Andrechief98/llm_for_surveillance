#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, session
from openai import OpenAI
from utilitiesOpenAI import *
import os
from werkzeug.utils import secure_filename
import rospy
import rosgraph
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from geometry_msgs.msg import PoseStamped
from OpenAI_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from flask_socketio import SocketIO, emit
import subprocess
from gazebo_plugins.srv import doorStringCommand, doorStringCommandRequest
import re


# Hostato a http://127.0.0.1:5000/


script_dir = os.path.dirname(__file__)

session={
    "messages":[]
    }

class ChatNode():

    def __init__(self):
        rospy.init_node('ChatNode', anonymous=True)
        self.rosPublisher = rospy.Publisher('assistant_message', String, queue_size=10)

        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleAlert)
        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        self.client = OpenAI()  
        required_assistant = "Surveillance guard expert"
        assistant_instructions = """You are a surveillance guard that must monitor an indoor environment."""
        self.model_to_use = "gpt-4o"

        assistants_names_list, assistant_ids_list = extractAssistantFromJson(self.client, script_dir)

        if required_assistant not in assistants_names_list:
            self.assistant = self.client.beta.assistants.create(
                model= self.model_to_use,
                name = required_assistant,
                tools=[
                        # {
                        #     "type": "code_interpreter"
                        # },
                        {
                            "type": "file_search"
                        },
                        {
                            "type": "function", 
                            "function": {
                                "name": "retrieve_system_state",
                                "description": "Use this function to request the current state of the system from the ROS data mediator. It returns information about the positions and orientation of robots and sensors, as well as positions and current states of door actuators . This function should be called before any other function call to ensure awareness of the system’s current condition, or whenever the user asks for the system’s status.",
                                "parameters": {}
                                }
                        },
                        {
                            "type": "function", 
                            "function": {
                                "name": "reset_sensors_activation",
                                "description": "Use this function to deactivate all sensors in the environment after the user explicit request.",
                                "parameters": {}
                                }
                        },
                        {
                            "type": "function", 
                            "function": {
                                "name": "display_cameras",
                                "description": "It opens a window to visualize each camera. The function must be used every time a camera feed is required or proposed",
                                "parameters": {
                                    "type": "object",
                                    "properties": {
                                        "cameras_names_list": {
                                            "type": "array", 
                                            "description": "List of all the cameras names to considered for visualization.",
                                            "items" : {
                                                "type" : "string",
                                                "description": "Name of a single camera"
                                            }
                                            },
                                        },
                                    "required": ["cameras_names_list"]
                                    }
                                }
                        },
                        {
                            "type": "function",
                            "function": {
                                "name": "control_actuator",
                                "description": "Use this function to control a sequence of door actuators in the environment via ROS service calls defined in the info.json file.",# Each actuator can be opened or closed based on user instructions.",
                                "parameters": {    
                                    "type": "object",
                                    "properties": {
                                        "actuators_sequence": {
                                            "type": "array",
                                            "description": "Ordered list of actuator commands. Each item specifies the ROS service name (from the info.json file) and the desired action ('open' or 'close').",
                                            "items": {
                                                "type": "object",
                                                "properties": {
                                                    "ros_service_name": {
                                                        "type": "string",
                                                        "description": "Name of the ROS service corresponding to a specific door actuator, as defined in the info.json file."
                                                        },
                                                    "command": {
                                                        "type": "string",
                                                        "enum": ["open", "close"],
                                                        "description": "Action to perform on the actuator: either 'open' or 'close', based on the user's intent."
                                                        }
                                                    },
                                                "required": ["ros_service_name", "command"]
                                            }
                                        }
                                    },
                                "required": ["actuators_sequence"]
                                }
                            }
                        },
                        {
                            "type": "function", 
                            "function": {
                                "name": "send_robot_to_area",
                                "description": "Takes the message of the operator and sent the indicated robot to the indicated area. The function will return the success or failure of the execution",
                                "parameters": {
                                    "type": "object",
                                    "properties": {
                                        "robot_to_send": {
                                            "type": "string", 
                                            "description": "name of the robot to send to a given area, either indicated by the user or indicated in the plan proposed by the LLM module"
                                            },
                                        "area_to_reach": {
                                            "type": "string", 
                                            "description": "A string containing the letter of the area the robot must reach."
                                            }
                                        },
                                    "required": ["robot_to_send", "area_to_reach"]
                                    }
                                }
                        }
                        # {
                        #     "type": "function", 
                        #     "function": {
                        #         "name": "send_robot_to_area",
                        #         "description": "Takes the message of the operator and sent the indicated robot to the indicated area. The function will return the success or failure of the execution",
                        #         "parameters": {
                        #             "type": "object",
                        #             "properties": {
                        #                 "robot_to_send": {
                        #                     "type": "string", 
                        #                     "description": "name of the robot to send to a given area, either indicated by the user or indicated in the plan proposed by the LLM module"
                        #                     },
                        #                 "area_to_reach": {
                        #                     "type": "string", 
                        #                     "description": "A string containing the coordinates (x,y) of the area the robot must reach obtained from info.json file."
                        #                     },
                        #                 "ros_topic": {
                        #                     "type": "string", 
                        #                     "description": "ros topic that must be consider to control the robot obtained from the info.json file"
                        #                     },
                        #                 },
                        #             "required": ["robot_to_send", "area_to_reach", "ros_topic"]
                        #             }
                        #         }
                        # }
                    ],
                instructions= assistant_instructions,
                response_format={
                                    "type": "json_schema", 
                                    "json_schema": {
                                        "name": "output_schema",
                                        "schema": {
                                            "type": "object",
                                            "properties": {
                                                "content": {
                                                    "type": "string",
                                                    "description": "String containing the actual response of the message."
                                                    },
                                                # "final_decision": {
                                                #     "type": "string",
                                                #     "description": "The actual robot to be activated."
                                                #     },
                                            },
                                            # "required": ["content", "final_decision"],
                                            "required": ["content"],
                                            "additionalProperties": False
                                            },
                                        "strict": True
                                        }
                                    }
            )
            print("New assistant created")

            # Update Assistants json file
            assistants_names_list.append(self.assistant.name) 
            assistant_ids_list.append(self.assistant.id)
            updateAssistantJsonFile(assistants_names_list, assistant_ids_list, script_dir)

        else:
            required_assistant_id = assistant_ids_list[assistants_names_list.index(required_assistant)]
            self.assistant = self.client.beta.assistants.retrieve(
                assistant_id = required_assistant_id
                )
            print("Already created assistant considered")


        self.img_file = self.client.files.create(
            file=open(f"{script_dir}/static/uploads/building_plan.png", "rb"),
            purpose="vision"
        )
        self.info_file = self.client.files.create(
            file=open("OpenAI_interface/config/info.json", "rb"), 
            purpose="assistants"
        )

        #print(self.info_file)


        self.task_instructions = """
# Role
You are an **LLM module** in an architecture for surveillance application with the purpose of detect possible intrusions by means of different types of sensors and manage the situations acting on robots and doors actuators in an indoor environment.
The architecture is composed by:
    - a Data Mediator: responsible to continuosly retrieve data from sensors, actuators and robots;
    - an LLM module (you): responsible to manage network and environmental information, user requests and perform  real-world action on the environment (specified as callable functions).

# Environment Overview
The provided file **building_plan.png** is the map of the considered indoor environment with the following specifications:
- **8 Areas (A - H):** Defined by impassable walls (solid black lines) and conceptual boundaries (blue dotted lines).
- **Walkable Space:** Light gray regions. Each room has one or more **doors** controlled by actuators via ROS services.
- **8 Lidar Sensors:** One at each room entrance, labeled `Lidar_1` through `Lidar_8` (red squares on the map).
- **2 Cameras:** `Camera_1` and `Camera_2` (red arrows on the map), available via ROS topics.
- **2 Mobile Robots:**
  - `turtlebot3_1`
  - `turtlebot3_2`

# ROS network and areas information:
All robots, sensors (lidars, cameras) and doors actuators are integrated within a **ROS network** and can be controlled by means of your callable functions. 
The provided **info.json** file contains all the information about the ROS network and coordinates environment:
- **robot topics names**: for sending robots in a given area;
- **camera topics names**  for visualizing cameras feed;
- **doors services names** for opening/closing doors;
- **areas information**: for retrieving area coordinates for robot control, perform spatial reasoning, understanding which areas are near a given activated sensors.

# Control Logic:
In case of user requests on peforming real-world actions in the environment, you must retrieve the system state before taking any actions in the environment.

If an intrusion is detected by sensors activation, the Data Mediator will immediately provide you an updated status of the environment. Based on this environmental information, you must:
1. integrate spatial information from both info.json and building_plan.png files to:
    - determine **all** the zones near the activated sensors that must be monitored;
    - determine which doors are related to the identified areas;
2. determine which robots should be sent to a given area considering activated sensors, robots current positions and robots remain battery level
3. propose an plan to the user composed by all the needed actions to handle the situation:
    - In case of unclear situation or doubts (e.g. activation of a single sensor), it is best to send the nearest robot to check **ALL* areas near the activated sensors. In case they are present, propose before using the cameras to show the user what is happening in the area.
    - In case of multiple sensors activation, You must understand which area must be checked from the sequence of activated sensors and spatial information about the environment. 
      Once you are sure about the area to be analyzed, you can propose:
        1. Show camera feed of the considered area (if present);
        2. Deploy one or more robots for further inspection (one robot for each area);
        2. Lock the entire area by closing all doors related to it. In case of robot deployed to check the considered area, You must leave opened one door to allow the robot to actually reach the area.

The user will read the plan and he will do one of the following:
- confirm the plan: in this case you must perform all actions indicated in the plan in subsequent order. 
- correct the overall plan or just some actions: once corrected, you must execute the actions in subsequent order; 
- ask to perform just some actions of the proposed plan;


# Actions and additional info:
The plan must be reasoned, including logical and coherent actions, ensuring the system operates smoothly. To do so, you must consider the following rules when combining actions:

1. **System State Verification:** 
    Before performing any real-world function you must have an updated status of the environment (e.g., positions and orientations of robots, states of sensors and actuators, 
    and positions of doors) . You will receive that from data mediator in case of intrusion or you can retrieve it by using the `retrieve_system_state` function. 
    This ensures you have an up-to-date overview of the system's condition. Don't consider the conversation history to understand the system state.

2. **Spatial information:** 
    Before performing any real-world function you must consider the information in the info.json file.

3. **Door Control:** 
    If the action (from user request or proposed plan) involve closing or opening the doors, you must use the function 'control_actuator' with arguments obtained from info.json file.
    Before sending a robot to an area that involves passing through doors, you must understand which doors should be considered to reach that area considering the current position of the indicated robot, the coordinates of the indicated area and the positions of the doors near that area.
    Then you have to verify the status of the these doors. If doors are closed, you will have to open them using the `control_actuator` function before proceeding.

3. **Sending Robots:** 
    Once you know the state of the doors and the robots, use the `send_robot_to_area` function and coordinates obtained from info.json file to send the robot to the target area. 
    This action must be executed after the doors are confirmed to be open (if required). 
    Pay attention if you close doors in previous actions in the plan that could block the area the robot has to reach.

4. **Camera Visualization:** 
    You must open the appropriate camera windows using the `display_cameras` function. 

**ALWAYS** retrieve ROS Topic names, ROS service names and areas specifications (coordinates, sensors and actuators in each area, ecc) from **`info.json`**.


Every time you perform an action in the environment, You will receive a positive or negative feedback about the success of the action. 
If the feedback is negative, you must reason about the failure and try again with corrected information. A new action must be performed after receiving the positive feedback from the previous one. 
If you fail to perform an action two times, you can pass to the next action of the plan informing the user about the error.


# Response Structure:
In case of sensor activations:
  1. **Status**: Report which sensors are active, which area must be checked, relevant doors, and robot statuses.
  2. **Plan**: Sequence of actions to manage the situations (no explanation).
  3. **Await Confirmation**: Do not execute until operator confirms (unless fully automated mode is requested).

It is possible that multiple sensors activations happens. In that case:
  1. **Updated Status:**
  2. **Updated Plan:**
  3. **Await Confirmation:**

In case of operator requests, you can simply answer normally and perform required actions respecting rules previously mentioned.

Always make sure to only execute actions that are logically necessary for achieving the user’s requirements or the proposed plan, especially if user change your initial plan. 
Ensure all dependencies between actions of the plan are respected in the order they need to be executed.
If any detail is unclear (e.g., ambiguous area or actions to do) or you have doubts, **ask the operator for clarification** before proceeding."""


        required_thread_name = "Surveillance application"

        threads_name_list, threads_id_list = extractThreadsFromJson(self.client, script_dir)


        ######### This section allows to consider the same thread created previously #########
        # if required_thread_name not in threads_name_list:

        #     self.thread = self.client.beta.threads.create(
        #         metadata={
        #             "thread_name":required_thread_name
        #             },
        #         messages=[
        #             {
        #                 "role": "user",
        #                 "content": [
        #                     {
        #                         "type": "text",
        #                         "text": self.task_instructions
        #                     },
        #                     #   {
        #                     #     "type": "image_url",      # Immagini prese da internet
        #                     #     "image_url": {"url": "https://example.com/image.png"} 
        #                     #   },
        #                     {
        #                         "type": "image_file",       # Immagini prese dal computer
        #                         "image_file": {
        #                             "file_id": self.img_file.id,
        #                             "detail":"high" # Può essere "low" e "high" e indica la risoluzione con cui verrà analizzata l'immagine
        #                             } 
        #                     },
        #                 ],
        #                 "attachments": [
        #                     { 
        #                         "file_id": self.info_file.id, 
        #                         "tools": [
        #                             {
        #                                 "type": "file_search"
        #                                 }
        #                             ] 
        #                         }
        #                 ],
        #             }
        #         ]
        #     )
        #     print("Created a new thread")

        #     threads_name_list.append(required_thread_name)
        #     threads_id_list.append(self.thread.id)

        #     updateThreadsJsonFile(threads_name_list,threads_id_list, script_dir)

        # else:
        #     required_thread_id = threads_id_list[threads_name_list.index(required_thread_name)]
        #     self.thread = self.client.beta.threads.retrieve(required_thread_id)
        #     print("Already existing thread considered")



        ####### To start every time from a complete new conversation, we everytime delete the previous thread (Better for debugging) ######

        if required_thread_name in threads_name_list:
            required_thread_id = threads_id_list[threads_name_list.index(required_thread_name)]
            self.thread = self.client.beta.threads.delete(required_thread_id)
            
            # UPDATE JSON FILE
            # Retrive stored Threads
            with open("/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/Open AI threads.json", "r") as file:
                threads_dict = json.load(file)

            # Since we cannot delete all threads together, we create a new list considering only the threads that must remain in the JSON
            new_threads_list = []

            # Loop on all the threads stored in the json file
            for thread_dict in threads_dict["threads"]:
                # If the name of the thread we want to delete is found, we pass to the next iteration. We don't add the thread in the list
                if required_thread_name == thread_dict["thread_name"]:
                    continue
                # If the name of the thread we want to delete is not equal to the current thread of the loop, we can add it to the final list of thread to store in the JSON
                else:
                    new_threads_list.append(thread_dict)
            
            # We update the list of threads of the JSON by saving the list created by the loop
            threads_dict["threads"] = new_threads_list

            # We overwrite the JSON
            with open("/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/Open AI threads.json", "w") as file:
                json.dump(threads_dict, file, indent=2)

            print("Existing thread deleted")

            # Since we changed the JSON, we update our lists of threads names and corresponding threads IDs to obtain an ordered list
            threads_name_list, threads_id_list = extractThreadsFromJson(self.client, script_dir)



        self.thread = self.client.beta.threads.create(
                metadata={
                    "thread_name":required_thread_name
                    },
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": self.task_instructions
                            },
                            #   {
                            #     "type": "image_url",      # Immagini prese da internet
                            #     "image_url": {"url": "https://example.com/image.png"} 
                            #   },
                            {
                                "type": "image_file",       # Immagini prese dal computer
                                "image_file": {
                                    "file_id": self.img_file.id,
                                    "detail":"high" # Può essere "low" e "high" e indica la risoluzione con cui verrà analizzata l'immagine
                                    } 
                            },
                        ],
                        "attachments": [
                            { 
                                "file_id": self.info_file.id, 
                                "tools": [
                                    {
                                        "type": "file_search"
                                        }
                                    ] 
                                }
                        ],
                    }
                ]
            )
        
        print("New thread created")

        threads_name_list.append(required_thread_name)
        threads_id_list.append(self.thread.id)

        updateThreadsJsonFile(threads_name_list,threads_id_list, script_dir)



    def handleAlert(self,req):
    ## gestire l'esecuzione della run dove si aggiunge all contesto dell'LLM il messaggio ricevuto e si esegue la funzione per l'ottenimento delle strategie

        if req:
            # Estrai e decodifica il messaggio ricevuto via ros service
            print("in esecuzione")
            request_info = json.loads(req.alert_info)
            
            # Eliminiamo "robot_list_name", "sensors_list_names", "current_orientation" di ogni robot e di ogni sensore

            robots_list_names = request_info.pop("robots_list_names", [])
            sensors_list_names = request_info.pop("sensors_list_names", [])

            for robot in robots_list_names:
                if robot in ["turtlebot_1","turtlebot_2"]:
                    request_info["robots"][robot].pop("current_orientation", None)

            for sensor in sensors_list_names:
                request_info["sensors"][sensor].pop("orientation", None)

            print(request_info)
            self.alert_info = request_info
            
            # Inserimento del messaggio ricevuto nella cronologia della chat come se provenisse dall’operatore (OpenAI non prevede messaggi associati a figure diverse da operatore o assistant)
            session["messages"].append({"role": "user", "content": f"{json.dumps(self.alert_info)}"})
            
            # Stampa l'allert nella chat da parte dell'USER:
            active_sensors = self.alert_info["activated_sensors"]
            socketio.emit('new_message', {"role": "data_mediator", "content": f"ALERT: \n Activated sensors: {active_sensors}"})

            # Registrazione del messaggio nel thread della conversazione
            self.client.beta.threads.messages.create(
                thread_id=self.thread.id,
                role="user",
                content=json.dumps(self.alert_info)
            )
            
            # run per ottenere la risposta dell’LLM
            run = self.client.beta.threads.runs.create_and_poll(
                thread_id=self.thread.id,
                assistant_id=self.assistant.id,
                model=self.model_to_use,
            )
            
            if run.status == 'completed':
                messages = self.client.beta.threads.messages.list(
                    thread_id=self.thread.id
                )
                # 
                message_dictionary = json.loads(messages.data[0].content[0].text.value)
                assistant_reply = message_dictionary["content"]
                
                # Aggiorna la cronologia della chat con la risposta dell’LLM
                session["messages"].append({"role": "assistant", "content": assistant_reply})
                
                # (Opzionale) Pubblica la decisione finale su un topic ROS
                #final_decision = message_dictionary.get("final_decision", "")
                #self.rosPublisher.publish(final_decision)
                
                socketio.emit('new_message', {"role": "assistant", "content": assistant_reply})
            else:
                print("Errore durante la generazione della risposta:", run.status)

            return triggerGptResponse("Success")
        else:
            return triggerGptResponse("Failed")
        

    def reset_sensors_activation(self):
        response = self.resetSensorsActivationClient()
        return f"All sensors deactivated"

    # Funzione che fa ricavare sia robot name, che area coordinates che topic names all'LLM (poco robusta agli errori)
    # def send_robot_to_area(self, robot, area, topic):

    #     topics_info_list = rospy.get_published_topics()
    #     topics_names_list = [topic_info[0] for topic_info in topics_info_list]

    #     if topic in topics_names_list and "goal" in topic:

    #         # area = area.strip("()")
    #         # area = tuple(map(float, area.split(',')))
    #         # print(area)  # Output: (1.5, 4.5)
    #         # area_x, area_y = tuple(area)

    #         numbers = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", area)

    #         # Converti in float e crea la tupla
    #         area_x, area_y = tuple(map(float, numbers))


    #         # Create a temporary client to call the clear_costmap service for all robots:
    #         match = re.search(r"_(\d+)$", robot)
    #         number = str(int(match.group(1)))
    #         service_name = "turtlebot3_" + str(number) + "/move_base/clear_costmaps"
    #         clear_costmap_client = rospy.ServiceProxy(service_name, Empty)

    #         clear_costmap_client()

            
    #         # Create a temporary publisher with the given topic
    #         temp_pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
            
    #         # Wait to register the publisher
    #         rospy.sleep(0.5)
            
    #         goal_msg = PoseStamped()

            
    #         goal_msg.pose.position.x = float(area_x)
    #         goal_msg.pose.position.y = float(area_y)
    #         goal_msg.pose.position.z = 0.0
    #         goal_msg.pose.orientation.x = 0.0
    #         goal_msg.pose.orientation.y = 0.0
    #         goal_msg.pose.orientation.z = 0.0
    #         goal_msg.pose.orientation.w = 1.0 
    #         goal_msg.header.stamp = rospy.Time.now()
    #         goal_msg.header.frame_id = "map"

    #         # Publish the goal message
    #         temp_pub.publish(goal_msg)
            
    #         # Wait to transmit the message
    #         rospy.sleep(0.5)
    #         return f"{robot} sent to area {area}"
        
    #     else:
    #         return f"wrong topic considered"


    # Funzione più robusta
    def send_robot_to_area(self, robot, area):

        with open("OpenAI_interface/config/info.json", "r") as f:
            info = json.load(f)
        
        robots_dict = info["ros_publishers"]["robots"]
        robots_list = robots_dict.keys()

        areas_dict = info["areas"]
        areas_list = areas_dict.keys()

        if robot in robots_list:    
            if area in areas_list:

              
                topic = robots_dict[robot]["ros_topic"]
                area_x = areas_dict[area]["coordinates"]["x"]
                area_y = areas_dict[area]["coordinates"]["y"]

                # Create a temporary client to call the clear_costmap service for all robots:
                match = re.search(r"_(\d+)$", robot)
                number = str(int(match.group(1)))
                service_name = "turtlebot3_" + str(number) + "/move_base/clear_costmaps"
                clear_costmap_client = rospy.ServiceProxy(service_name, Empty)

                clear_costmap_client()

                
                # Create a temporary publisher with the given topic
                temp_pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
                
                # Wait to register the publisher
                rospy.sleep(0.5)
                
                goal_msg = PoseStamped()

                
                goal_msg.pose.position.x = float(area_x)
                goal_msg.pose.position.y = float(area_y)
                goal_msg.pose.position.z = 0.0
                goal_msg.pose.orientation.x = 0.0
                goal_msg.pose.orientation.y = 0.0
                goal_msg.pose.orientation.z = 0.0
                goal_msg.pose.orientation.w = 1.0 
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = "map"

                # Publish the goal message
                temp_pub.publish(goal_msg)
                
                # Wait to transmit the message
                rospy.sleep(0.5)
                return f"{robot} sent to area {area}"
            
            else:
                return "Wrong area letter considered"
        else:
            return "Wrong robot name considered"



    def display_cameras(self, cameras_names_list):
        
        with open("OpenAI_interface/config/info.json", "r") as f:
            info = json.load(f)
        
        sensors_dict = info["ros_subscribers"]["sensors"]
        cameras_list = sensors_dict.keys()

        camera_names_correctness = {}
        topics_list = []

        for camera_name in cameras_names_list:
            camera_name = camera_name.lower()

            if camera_name in cameras_list:
                topics_list.append(sensors_dict[camera_name]["ros_topic"])
                camera_names_correctness[camera_name] = True
            
            else:
                camera_names_correctness[camera_name] = False    
        
        subprocess.Popen(["python3", "/home/andrea/ros_packages_aggiuntivi/src/OpenAI_interface/src/display_camera.py"] + topics_list)

        return f"Summary of cameras names correctness: {json.dumps(camera_names_correctness)}"
        
    

    def control_actuator(self, actuators_sequence):

        master = rosgraph.Master('/rospy')
        services_info = master.getSystemState()[2]  # [publishers, subscribers, services]

        # Estrai solo i nomi dei servizi
        ros_service_names_list = [service_info[0] for service_info in services_info]


        responses_list = []

        for actuator in actuators_sequence:

            ros_service_name = actuator.get("ros_service_name")
            command = actuator.get("command")

            if ros_service_name in ros_service_names_list:
                try:
                    rospy.wait_for_service(ros_service_name, timeout=5)
                except rospy.ROSException as e:
                    rospy.logerr(f"Service {ros_service_name} not available: {e}")
                    response = {
                        "ros_service_name" : ros_service_name,
                        "success" : "false",
                        "additional_info" : e
                    }
                    responses_list.append(response)
                    continue
                
                try:
                    service_client = rospy.ServiceProxy(ros_service_name, doorStringCommand)
                    
                    req = doorStringCommandRequest()
                    req.command = command
                    
                    # Effettua la chiamata sincrona al servizio e ne gestisci la risposta
                    resp = service_client(req)

                    response = {
                        "ros_service_name" : ros_service_name,
                        "success" : "true",
                        "additional_info" : ""
                    }
                    responses_list.append(response)
                
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed call to service {ros_service_name}: {e}")
                    response = {
                        "ros_service_name" : ros_service_name,
                        "success" : "false",
                        "additional_info" : e
                    }
                    responses_list.append(response)
            else:
                response = {
                        "ros_service_name" : ros_service_name,
                        "success" : "false",
                        "additional_info" : "Wrong ros service name considered. Check again in the info.json file"
                    }
                responses_list.append(response)
        
        return str(responses_list)

    def retrieve_system_state(self):
        response = self.retrieveSystemStateClient()
        system_state = response.system_state
        return system_state

chatNode = ChatNode()


app = Flask(__name__)
app.secret_key = "supersegreta"

socketio = SocketIO(app)



DEFAULT_MESSAGE = chatNode.task_instructions
DEFAULT_IMAGE = "static/uploads/building_plan.png"  


@app.route("/")
def index():
    # Inizializza la sessione con il messaggio di default
    return render_template("index.html", default_message=DEFAULT_MESSAGE, default_image=DEFAULT_IMAGE)
    

@app.route("/chat", methods=["POST"])
def chat():
    data = request.json
    user_message = data.get("message", "").strip()
    print(f"Messaggio dell'utente: {user_message}")

    if not user_message:
        return jsonify({"error": "Messaggio vuoto"}), 400
    
    session["messages"].append({"role": "user", "content": user_message})

    try:
        chatNode.client.beta.threads.messages.create(
            thread_id=chatNode.thread.id,
            role="user",
            content=user_message
        )

        run = chatNode.client.beta.threads.runs.create_and_poll(
            thread_id=chatNode.thread.id,
            assistant_id=chatNode.assistant.id,
            model = chatNode.model_to_use,
        )

        if run.status == 'completed':
            print("Run completed")
            messages = chatNode.client.beta.threads.messages.list(
                thread_id=chatNode.thread.id
            )
            
            message_dictionary = json.loads(messages.data[0].content[0].text.value)
            print(message_dictionary)
            
            
            #print(message_dictionary["final_decision"])
            
            assistant_reply = message_dictionary["content"]
            #final_decision = message_dictionary["final_decision"]


            session["messages"].append({"role": "assistant", "content": assistant_reply})


            response = {
                "reply": assistant_reply,
                #"final_decision": final_decision,
            }
            return jsonify(response)
        
        elif run.status == "requires_action":
            
            while run.status == 'requires_action':
                # The while loop allows to perform sequential actions (multiple action steps) where the next action requires parameters proposed by the LLM based on the output of previous actions. 
                # It allows to perform a single action, send the response to the LLM and obtain the parameters for the new action

                print("List of all function calls:")
                print(run.required_action.submit_tool_outputs.tool_calls)

                tool_outputs = []

                # Loop through each tool required by a single action step (no information are returned to the LLM before finishing this sequence of actions)
                for tool in run.required_action.submit_tool_outputs.tool_calls:
                    # print("Considered tool")
                    # print(tool)

                    # Extraction of the function arguments:
                    args = json.loads(tool.function.arguments)
                    

                    if tool.function.name == "send_robot_to_area":
                        
                        # Function execution
                        # result = chatNode.send_robot_to_area(args["robot_to_send"], args["area_to_reach"], args["ros_topic"])
                        result = chatNode.send_robot_to_area(args["robot_to_send"], args["area_to_reach"])

                        # Serve per dire all'LLM che la funzione è stata eseguita
                        tool_outputs.append({
                        "tool_call_id": tool.id,
                        "output": result
                        })
                    
                    elif tool.function.name == "display_cameras":

                        # Function execution
                        result = chatNode.display_cameras(args["cameras_names_list"])

                        # Serve per dire all'LLM che la funzione è stata eseguita
                        tool_outputs.append({
                        "tool_call_id": tool.id,
                        "output": result
                        })
                    
                    elif tool.function.name == "reset_sensors_activation":

                        # Function execution
                        result = chatNode.reset_sensors_activation()

                        # Serve per dire all'LLM che la funzione è stata eseguita
                        tool_outputs.append({
                        "tool_call_id": tool.id,
                        "output": result
                        })

                    elif tool.function.name == "control_actuator":

                        # Function execution
                        result = chatNode.control_actuator(args["actuators_sequence"])

                        # Serve per dire all'LLM che la funzione è stata eseguita
                        tool_outputs.append({
                        "tool_call_id": tool.id,
                        "output": result
                        })
                    
                    elif tool.function.name == "retrieve_system_state":

                        # Function execution
                        result = chatNode.retrieve_system_state()

                        # Serve per dire all'LLM che la funzione è stata eseguita
                        tool_outputs.append({
                        "tool_call_id": tool.id,
                        "output": result
                        })
                    
                    else:
                        print("No valid function call")
                    

                if tool_outputs:
                    try:
                        run = chatNode.client.beta.threads.runs.submit_tool_outputs_and_poll(
                        thread_id=chatNode.thread.id,
                        run_id=run.id,
                        tool_outputs=tool_outputs
                        )
                        print("Tool outputs submitted successfully.")
                        print("Run status after submission of the output:")
                        print(run.status)
                    except Exception as e:
                        print("Failed to submit tool outputs:", e)
                else:
                    print("No tool outputs to submit.")
                
                
                

            if run.status == 'completed':
                messages = chatNode.client.beta.threads.messages.list(
                    thread_id=chatNode.thread.id
                )
            
                message_dictionary = json.loads(messages.data[0].content[0].text.value)
                print(message_dictionary)

                assistant_reply = message_dictionary["content"]
                #final_decision = message_dictionary["final_decision"]

                #rosPublisher.publish(final_decision)

                session["messages"].append({"role": "assistant", "content": assistant_reply})


                response = {
                    "reply": assistant_reply,
                    #"final_decision": final_decision,
                }
                return jsonify(response)


            else:
                print("Run not completed. Current status:")
                print(run.status)

                print(run.required_action.submit_tool_outputs.tool_calls)
                return jsonify({"error": "Run not completed", "status": run.status})

        else:
            print("Error in the response generation. Current status:")
            print(run.status)
            return jsonify({"error": "Error in the response generation.", "status": run.status})

    except Exception as e:
        print(e)
        return jsonify({"error": str(e)}), 500
    


@app.route("/upload", methods=["POST"])
def upload_image():
    if "image" not in request.files:
        return jsonify({"error": "Nessun file caricato"}), 400

    file = request.files["image"]

    if file.filename == "":
        return jsonify({"error": "Nome file non valido"}), 400

    filename = secure_filename(file.filename)
    file_path = os.path.join(app.config["UPLOAD_FOLDER"], filename)
    file.save(file_path)

    return jsonify({"image_url": f"{script_dir}/static/uploads/{filename}"})



@app.route("/reset", methods=["POST"])
def reset_chat():
    session.pop("messages", None)
    return jsonify({"message": "Chat resettata!"})



@app.route('/activate_robot', methods=['POST'])
def activate_robot():
    # Ottieni i dati dalla richiesta JSON
    data = request.get_json()
    response = data.get('response')

    chatNode.rosPublisher.publish(response)

    return jsonify({"status": "success", "response": response})


if __name__ == "__main__":
    app.run(debug=True)
