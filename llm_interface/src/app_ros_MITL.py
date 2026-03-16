#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, session
from openai import OpenAI
import actionlib.simple_action_server
from utilitiesOpenAI import *
import os
from werkzeug.utils import secure_filename
import rospy
import rosgraph
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from geometry_msgs.msg import PoseStamped
from llm_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback
from flask_socketio import SocketIO, emit
import subprocess
from gazebo_plugins.srv import doorStringCommand, doorStringCommandRequest 
import re
import time
import actionlib
import yaml



script_dir = os.path.dirname(__file__) #/home/[...]/llm_for_surveillance/llm_interface/src
print("\n" + script_dir)
if rospy.has_param("/markerVisualizationNode/mode"):
    mode = rospy.get_param("/markerVisualizationNode/mode")
    #print("parameter /markerVisualizationNode/mode found")
    #print(mode)
else:
    mode = "original"




if mode == "original":
    file_config = "info.json"
    file_prompt = "prompts.yaml"
    file_building_plan = "building_plan.png"
else:
    file_config = "info"+mode+".json"
    file_prompt = "prompts"+mode+".yaml"
    file_building_plan = "building_plan"+mode+".png"

print(file_config)

session={
    "messages":[]
    }

class ChatNode():

    def __init__(self):
        rospy.init_node('ChatNode', anonymous=True)
        self.rosPublisher = rospy.Publisher('assistant_message', String, queue_size=10)

        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleAlert)
        # self.robotGoalCheckerServer = rospy.Service('/robot_goal_checker', triggerGpt, self.handleRobotGoalChecker)
        self.robotGoalCheckerServer = actionlib.SimpleActionServer('/robot_goal_checker', goalCheckerLLMAction, self.handleRobotGoalChecker, auto_start = False)
        self.robotGoalCheckerServer.start()

        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        self.client = OpenAI()  
        required_assistant = "Surveillance guard expert"
        assistant_instructions = """You are a surveillance guard that must monitor an indoor environment."""
        self.model_to_use = "gpt-4o"

        assistants_names_list, assistant_ids_list = extractAssistantFromJson(self.client, script_dir)

        self.last_run = None

        file_paths = [f"{script_dir}/../config/{file_config}"]
        file_streams = [open(path, "rb") for path in file_paths]
        print("file_path: "+ f"{script_dir}/../config/{file_config}"+"\nlenght file_streams: "+ str(len(file_streams)))

        if required_assistant not in assistants_names_list:
            # We need to create a new assistant                 
            
            # We update the JSON file 
            files_name_list, files_id_list, vector_store_id_list = extractFilesFromJson(self.client, script_dir)
            found = False
            print("file looked for in Open AI files.json")

            if file_config in files_name_list:
                # the configuration file is already uploaded. We search for the correct vector_store_id
                for vector_store_id in vector_store_id_list:
                    for file in self.client.vector_stores.files.list(vector_store_id = vector_store_id):
                        retrieved_file = self.client.files.retrieve(file_id=file.id)       
                        if retrieved_file.filename == file_config:
                            required_vector_store_id =  vector_store_id
                            found = True
                            break

                    if found:
                        break
                print("already created file info.config considered")
            else:
                # Vector store is needed to upload files to the assistant.
                vector_store = self.client.vector_stores.create(name="Environmental and ROS information for surveillance")
                required_vector_store_id = vector_store.id

                # We upload the file
                file_batch = self.client.vector_stores.file_batches.upload_and_poll(
                    vector_store_id = required_vector_store_id, files=file_streams
                )
                # print(file_batch.status)
                # print(file_batch.file_counts)
                

                for file in self.client.vector_stores.files.list(vector_store_id = required_vector_store_id):
                    retrieved_file = self.client.files.retrieve(file_id=file.id)       
                    if retrieved_file.filename == file_config:
                        files_name_list.append(file_config)
                        files_id_list.append(file.id)
                        vector_store_id_list.append(required_vector_store_id)


                updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)
                print("New info.config file uploaded")


            self.assistant = self.client.beta.assistants.create(
                model= self.model_to_use,
                temperature=0.0,
                name = required_assistant,
                tools=[
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
                                "description": "It opens a window to visualize each camera. The function must be used every time a camera feed is required or proposed. It takes as input the name of the cameras.",
                                "parameters": {
                                    "type": "object",
                                    "properties": {
                                        "cameras_names_list": {
                                            "type": "array", 
                                            "description": "List of all the cameras names to considered for visualization.",
                                            "items" : {
                                                "type" : "string",
                                                "description": "Name of a single camera."
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
                                "description": f"Use this function to control a sequence of door actuators in the environment via ROS service calls defined in the {file_config} file.",# Each actuator can be opened or closed based on user instructions.",
                                "parameters": {    
                                    "type": "object",
                                    "properties": {
                                        "actuators_sequence": {
                                            "type": "array",
                                            "description": f"Ordered list of actuator commands. Each item specifies the ROS service name (from the {file_config} file) and the desired action ('open' or 'close').",
                                            "items": {
                                                "type": "object",
                                                "properties": {
                                                    "ros_service_name": {
                                                        "type": "string",
                                                        "description": f"Name of the ROS service corresponding to a specific door actuator, as defined in the {file_config} file."
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
                                "name": "send_robots_to_area",
                                "description": "Use this function to deploy a sequence of robots to corresponding areas. The function will return the success or failure of each robot deployment.",
                                "parameters": {
                                    "type": "object",
                                    "properties": {
                                        "robots_sequence": {
                                            "type": "array",
                                            "description": "Ordered list of robots to deploy. Each item specifies the robot name and the area where the robot must be deployed.",
                                            "items": {
                                                "type": "object",
                                                "properties": {
                                                    "robot_to_send": {
                                                        "type": "string",
                                                        "description": "Name of the considered robot to deploy."
                                                        },
                                                    "area_to_reach": {
                                                        "type": "string",
                                                        "description": "Name of the area where the corresponding robot must be deploy."
                                                        }
                                                    },
                                                "required": ["robot_to_send", "area_to_reach"]
                                            }
                                        },                                    
                                        },
                                    "required": ["robots_sequence"]
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
                        #                     "description": "A string containing the letter of the area the robot must reach."
                        #                     }
                        #                 },
                        #             "required": ["robot_to_send", "area_to_reach"]
                        #             }
                        #         }
                        # }
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
                        #                     "description": f"A string containing the coordinates (x,y) of the area the robot must reach obtained from {file_config} file."
                        #                     },
                        #                 "ros_topic": {
                        #                     "type": "string", 
                        #                     "description": f"ros topic that must be consider to control the robot obtained from the {file_config} file"
                        #                     },
                        #                 },
                        #             "required": ["robot_to_send", "area_to_reach", "ros_topic"]
                        #             }
                        #         }
                        # }
                    ],
                tool_resources={"file_search": {"vector_store_ids": [required_vector_store_id]}},
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
                                            },
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
            # The assistant is already created. We can retrieve it from the JSON file.
            required_assistant_id = assistant_ids_list[assistants_names_list.index(required_assistant)]
            self.assistant = self.client.beta.assistants.retrieve(
                assistant_id = required_assistant_id
                )
            print("Already created assistant considered")


        # We extract the correct prompt from the yaml file

        with open(f"{script_dir}/../config/{file_prompt}") as f:
            prompts_dict = yaml.load(f, Loader=yaml.SafeLoader)
            print("path for prompts_dict: "+f"{script_dir}/../config/{file_prompt}"+ "\nlenght promts_dict: "+ str(len(prompts_dict)))

        prompt_type = "man_in_the_loop"
        #prompt_type = "autonomous"

        self.task_instructions = prompts_dict[prompt_type]

        print(f"{prompt_type} prompt considered")


        # We retrieve the thread
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



        ####### To start everytime from a complete new conversation, we everytime delete the previous thread (Better for debugging) ######

        if required_thread_name in threads_name_list:
            required_thread_id = threads_id_list[threads_name_list.index(required_thread_name)]
            self.thread = self.client.beta.threads.delete(required_thread_id)
            
            # UPDATE THREAD JSON FILE
            # Retrive stored Threads
            with open(f"{script_dir}/Open AI threads.json", "r") as file:
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
            with open(f"{script_dir}/Open AI threads.json", "w") as file:
                json.dump(threads_dict, file, indent=2)

            print("Existing thread deleted")

            # Since we changed the JSON, we update our lists of threads names and corresponding threads IDs to obtain an ordered list
            threads_name_list, threads_id_list = extractThreadsFromJson(self.client, script_dir)
  

        if self.model_to_use == "gpt-4o":
            # it can process both files and images

            # If we want to upload images in the thread we can create images in case of new one or retrieve already created images:
            required_files_names_list = [f"{file_building_plan}"]
            files_name_list, files_id_list, vector_store_id_list = extractFilesFromJson(self.client, script_dir)
            print(files_name_list)
            
            for required_file_name in required_files_names_list:
                if required_file_name in files_name_list:
                    # If the image is already created, we retrieve the image
                    required_file_id = files_id_list[files_name_list.index(required_file_name)]
                    self.img_file = self.client.files.retrieve(required_file_id)
                    print("Already created file considered")
                else:
                    # Otherwise, we create the image and we update the JSON file
                    self.img_file = self.client.files.create(
                            file=open(f"{script_dir}/static/uploads/{required_file_name}", "rb"),
                            purpose="vision"
                        )
                    required_file_id = self.img_file.id

                    files_name_list.append(required_file_name)
                    files_id_list.append(required_file_id)
                    vector_store_id_list.append("/")

                    updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)
                    print("Files uploaded")
                    

            
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
                                {
                                    "type": "image_file",       
                                    "image_file": {
                                        "file_id": self.img_file.id,
                                        "detail":"high" # Image resolution: "low" or "high"
                                        } 
                                },
                            ],
                            # "attachments": [
                            #     { 
                            #         "file_id": self.info_file.id, 
                            #         "tools": [
                            #             {
                            #                 "type": "file_search"
                            #                 }
                            #             ] 
                            #         }
                            # ],
                        }
                    ]
                )
        # elif self.model_to_use == "o3-mini":
        #     # It cannot process images
        #     self.thread = self.client.beta.threads.create(
        #             metadata={
        #                 "thread_name":required_thread_name
        #                 },
        #             messages=[
        #                 {
        #                     "role": "user",
        #                     "content": [
        #                         {
        #                             "type": "text",
        #                             "text": self.task_instructions
        #                         },
        #                     ],
        #                     # "attachments": [
        #                     #     { 
        #                     #         "file_id": self.info_file.id, 
        #                     #         "tools": [
        #                     #             {
        #                     #                 "type": "file_search"
        #                     #                 }
        #                     #             ] 
        #                     #         }
        #                     # ],
        #                 }
        #             ]
        #         )

        
        print("New thread created")

        threads_name_list.append(required_thread_name)
        threads_id_list.append(self.thread.id)

        updateThreadsJsonFile(threads_name_list,threads_id_list, script_dir)

    
    def handleRobotGoalChecker(self,goal):
        if goal:
            
            feedback = goalCheckerLLMFeedback()
            result = goalCheckerLLMResult()

            # Inserimento del messaggio ricevuto nella cronologia della chat come se provenisse dall’operatore (OpenAI non prevede messaggi associati a figure diverse da operatore o assistant)
            session["messages"].append({"role": "user", "content": f"{goal.message_for_LLM}"})

            while True:
                feedback.status_LLM = "Sending robot goal success to LLM..."
                self.robotGoalCheckerServer.publish_feedback(feedback)

                if self.last_run == None or self.last_run.status == 'completed':

                    # Registrazione del messaggio nel thread della conversazione
                    self.client.beta.threads.messages.create(
                        thread_id=self.thread.id,
                        role="user",
                        content=goal.message_for_LLM
                    )

                    # run per ottenere la risposta dell’LLM
                    self.last_run = self.client.beta.threads.runs.create_and_poll(
                        thread_id=self.thread.id,
                        assistant_id=self.assistant.id,
                        model=self.model_to_use,
                        tool_choice={"type": "file_search"}
                    )
                    
                    if self.last_run.status == 'completed':
                        messages = self.client.beta.threads.messages.list(
                            thread_id=self.thread.id
                        )

                        message_dictionary = json.loads(messages.data[0].content[0].text.value)
                        assistant_reply = message_dictionary["content"]

                        # print("\nAI:") 
                        # print(f"\t{assistant_reply}")
                        
                        # Aggiorna la cronologia della chat con la risposta dell’LLM
                        session["messages"].append({"role": "assistant", "content": assistant_reply})
                        
                        socketio.emit('new_message', {"role": "assistant", "content": assistant_reply})


                        result.success = f"Message from robot correctly processed by the LLM"

                    elif self.last_run.status == 'requires_action':
                        tool_outputs = []

                        print("Total number of function calls:")
                        print(f"\t{len(self.last_run.required_action.submit_tool_outputs.tool_calls)}")

                        # Loop through each tool required by a single action step (no information are returned to the LLM before finishing this sequence of actions)
                        for tool in self.last_run.required_action.submit_tool_outputs.tool_calls:

                            tool_outputs.append(
                                {
                                    "tool_call_id": tool.id,
                                    "output": "Don't call functions. Just report the robot's report to the user."
                                }
                            )
                        if tool_outputs:
                            try:
                                self.last_run = self.client.beta.threads.runs.submit_tool_outputs_and_poll(
                                    thread_id=self.thread.id,
                                    run_id=self.last_run.id,
                                    tool_outputs=tool_outputs
                                )

                            except Exception as e:
                                print("Function handleRobotGoalChecker: failed to submit tool outputs:", e)
                        else:
                            print("Function handleRobotGoalChecker: No tool outputs to submit.")

                    else:
                        print("Error in the response generation inside the function handleRobotGoalChecker:", self.last_run.status)
                        result.success = f"Message from robot NOT processed by the LLM"
                    
                    #rospy.loginfo(result.success)
                    self.robotGoalCheckerServer.set_succeeded(result)
                    return 
                else:
                    continue
        else:
            result.success = f"Function handleRobotGoalChecker: No message received from robot"
            rospy.loginfo(result.success)
            self.robotGoalCheckerServer.set_succeeded(result)

            return 

    
    
    def handleAlert(self,req):
    ## gestire l'esecuzione della run dove si aggiunge all contesto dell'LLM il messaggio ricevuto e si esegue la funzione per l'ottenimento delle strategie

        if req:
            # Estrai e decodifica il messaggio ricevuto via ros service
            request_info = json.loads(req.alert_info)
            
            # Eliminiamo "robot_list_name", "sensors_list_names", "current_orientation" di ogni robot e di ogni sensore
            
            with open(f"{script_dir}/../config/{file_config}", "r") as f:
                info = json.load(f)
    
            areas_dict = info["areas"]
        
            
            del request_info["robots_list_names"]
            del request_info["sensors_list_names"]
            del request_info["actuators_list_names"]

            robots_name = request_info["robots"].keys()

            for robot_name in robots_name:
                
                x, y, z = request_info["robots"][robot_name]["current_position"]
    
                for area_name, area_info in areas_dict.items():
                    x_range = area_info["coordinate_ranges"]["x"]
                    y_range = area_info["coordinate_ranges"]["y"]

                    if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                        request_info["robots"][robot_name]["current_area"] = area_name
                        break

                del request_info["robots"][robot_name]["current_orientation"]
            
            del request_info["sensors"]


            doors_name = request_info["actuators"].keys()

            for door_name in doors_name:
                del request_info["actuators"][door_name]["position"]
            
            #print(request_info)
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
            
            start_response_time = time.time()

            # run per ottenere la risposta dell’LLM
            self.last_run = self.client.beta.threads.runs.create_and_poll(
                thread_id=self.thread.id,
                assistant_id=self.assistant.id,
                model=self.model_to_use,
                tool_choice={"type": "file_search"}
            )
            
            if self.last_run.status == 'completed':
                end_response_time = time.time()
                # inference_plan_time = end_response_time - start_response_time
                # print(f"Inference plan time: {inference_plan_time}")

                messages = self.client.beta.threads.messages.list(
                    thread_id=self.thread.id
                )
                # 
                message_dictionary = json.loads(messages.data[0].content[0].text.value)
                assistant_reply = message_dictionary["content"]
                # print("\nAI:") 
                # print(f"\t{assistant_reply}")
                
                # Aggiorna la cronologia della chat con la risposta dell’LLM
                session["messages"].append({"role": "assistant", "content": assistant_reply})
            
                socketio.emit('new_message', {"role": "assistant", "content": assistant_reply})
            
            else:
                print("Function handleAlert: error in the response generation:", self.last_run.status)

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
    # def send_robot_to_area(self, robot, area):

    #     with open(f"llm_interface/config/{file_config}", "r") as f:
    #         info = json.load(f)
        
    #     robots_dict = info["ros_publishers"]["robots"]
    #     robots_list = robots_dict.keys()

    #     areas_dict = info["areas"]
    #     areas_list = areas_dict.keys()

    #     if robot in robots_list:    
    #         if area in areas_list:

              
    #             topic = robots_dict[robot]["ros_topic"]
    #             area_x = areas_dict[area]["coordinates"]["x"]
    #             area_y = areas_dict[area]["coordinates"]["y"]

    #             # Create a temporary client to call the clear_costmap service for all robots:
    #             match = re.search(r"_(\d+)$", robot)
    #             number = str(int(match.group(1)))
    #             service_name = "turtlebot3_" + str(number) + "/move_base/clear_costmaps"
    #             clear_costmap_client = rospy.ServiceProxy(service_name, Empty)

    #             clear_costmap_client()

                
    #             # Create a temporary publisher with the given topic
    #             temp_pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
                
    #             # Wait to register the publisher
    #             rospy.sleep(0.5)
                
    #             goal_msg = PoseStamped()

                
    #             goal_msg.pose.position.x = float(area_x)
    #             goal_msg.pose.position.y = float(area_y)
    #             goal_msg.pose.position.z = 0.0
    #             goal_msg.pose.orientation.x = 0.0
    #             goal_msg.pose.orientation.y = 0.0
    #             goal_msg.pose.orientation.z = 0.0
    #             goal_msg.pose.orientation.w = 1.0 
    #             goal_msg.header.stamp = rospy.Time.now()
    #             goal_msg.header.frame_id = "map"

    #             # Publish the goal message
    #             temp_pub.publish(goal_msg)
                
    #             # Wait to transmit the message
    #             rospy.sleep(0.5)
    #             return f"{robot} sent to area {area}"
            
    #         else:
    #             return "Wrong area letter considered"
    #     else:
    #         return "Wrong robot name considered"

    def send_robots_to_area(self, robots_sequence):

        with open(f"{script_dir}/../config/{file_config}", "r") as f:
            info = json.load(f)

        robots_dict = info["ros_publishers"]["robots"]
        robots_list = robots_dict.keys()

        areas_dict = info["areas"]
        areas_list = areas_dict.keys()

        robot_deployment_correctness = {}
        
        for robot_to_deploy in robots_sequence:
            robot = robot_to_deploy.get("robot_to_send")
            area = robot_to_deploy.get("area_to_reach")

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
                    goal_msg.header.frame_id = f"map"

                    # Publish the goal message
                    temp_pub.publish(goal_msg)
                    
                    # Wait to transmit the message
                    rospy.sleep(0.5)

                    # return f"{robot} sent to area {area}"
                    robot_deployment_correctness[robot] = {
                        "deployment_success": True,
                        "additional_info": None
                    }
                
                else:
                    robot_deployment_correctness[robot] = {
                        "deployment_success": False,
                        "additional_info": "Wrong area letter considered"
                    }
            else:
                robot_deployment_correctness[robot] = {
                        "deployment_success": False,
                        "additional_info": "Wrong robot name considered"
                    }
                

        return f"Summary of robots deployments correctness: {json.dumps(robot_deployment_correctness)}" 



    def display_cameras(self, cameras_names_list):
        
        with open(f"{script_dir}/../config/{file_config}", "r") as f:
            info = json.load(f)
        
        sensors_dict = info["ros_subscribers"]["sensors"]
        cameras_list = sensors_dict.keys()

        camera_names_correctness = {}
        topics_list = []

        for camera_name in cameras_names_list:

            camera_name_lower = camera_name.lower()

            if camera_name_lower in cameras_list:
                topics_list.append(sensors_dict[camera_name_lower]["ros_topic"])
                camera_names_correctness[camera_name_lower] = {
                    "correctness": True,
                    "reason": ""
                }  
            
            else:
                camera_names_correctness[camera_name_lower] = {
                    "correctness": False,
                    "reason": "wrong name"
                }    
                print(camera_name)
                print(camera_name_lower)
                print(cameras_list)
        
        subprocess.Popen(["python3", f"{script_dir}/display_camera.py"] + topics_list)

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
                        "additional_info" : f"Wrong ros service name considered. Check again in the {file_config} file"
                    }
                responses_list.append(response)
        
        return str(responses_list)

    def retrieve_system_state(self):
        response = self.retrieveSystemStateClient()
        system_state = json.loads(response.system_state)

        with open(f"{script_dir}/../config/{file_config}", "r") as f:
            info = json.load(f)

        areas_dict = info["areas"]
    
        
        del system_state["robots_list_names"]
        del system_state["sensors_list_names"]
        del system_state["actuators_list_names"]

        robots_name = system_state["robots"].keys()

        for robot_name in robots_name:
            
            x, y, z = system_state["robots"][robot_name]["current_position"]
   
            for area_name, area_info in areas_dict.items():
                x_range = area_info["coordinate_ranges"]["x"]
                y_range = area_info["coordinate_ranges"]["y"]

                if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                    system_state["robots"][robot_name]["current_area"] = area_name
                    break

            del system_state["robots"][robot_name]["current_orientation"]
        
        del system_state["sensors"]


        doors_name = system_state["actuators"].keys()

        for door_name in doors_name:
            del system_state["actuators"][door_name]["position"]
        

        return json.dumps(system_state)

chatNode = ChatNode()


app = Flask(__name__)
app.secret_key = "supersegreta"

socketio = SocketIO(app)



DEFAULT_MESSAGE = chatNode.task_instructions
DEFAULT_IMAGE = f"static/uploads/{file_building_plan}"  


@app.route("/")
def index():
    # Inizializza la sessione con il messaggio di default
    return render_template("index.html", default_message=DEFAULT_MESSAGE, default_image=DEFAULT_IMAGE)
    

@app.route("/chat", methods=["POST"])
def chat():
    data = request.json
    user_message = data.get("message", "").strip()

    # print("\nUSER:")
    # print(f"\t{user_message}")

    if not user_message:
        return jsonify({"error": "Messaggio vuoto"}), 400
    
    session["messages"].append({"role": "user", "content": user_message})

    try:
        chatNode.client.beta.threads.messages.create(
            thread_id=chatNode.thread.id,
            role="user",
            content=user_message
        )

        start_response_time = time.time()
        print("break 1")
        chatNode.last_run = chatNode.client.beta.threads.runs.create_and_poll(
            thread_id=chatNode.thread.id,
            assistant_id=chatNode.assistant.id,
            model = chatNode.model_to_use,
            tool_choice={"type": "file_search"},
            reasoning_effort = None
        )
        print("break 2")

        if chatNode.last_run.status == 'completed':
            end_response_time = time.time()
            inference_plan_time = end_response_time - start_response_time
            # print("Run completed")
            # print(f"Inference plan time: {inference_plan_time}")

            messages = chatNode.client.beta.threads.messages.list(
                thread_id=chatNode.thread.id
            )
            
            message_dictionary = json.loads(messages.data[0].content[0].text.value)        
            assistant_reply = message_dictionary["content"]

            if isinstance(assistant_reply,dict):
                assistant_reply = assistant_reply["content"]
            
            # print("\nAI:") 
            # print(f"\t{assistant_reply}")


            session["messages"].append({"role": "assistant", "content": assistant_reply})


            response = {
                "reply": assistant_reply,
            }
            return jsonify(response)
        
        elif chatNode.last_run.status == "requires_action":
            end_response_time = time.time()
            inference_action_time = end_response_time - start_response_time
            #print(f"Inference action time: {inference_action_time}")

            while chatNode.last_run.status == 'requires_action':
                # The while loop allows to perform sequential actions (multiple action steps) where the next action requires parameters proposed by the LLM based on the output of previous actions. 
                # It allows to perform a single action, send the response to the LLM and obtain the parameters for the new action

                # print("List of all function calls:")
                # print(chatNode.last_run.required_action.submit_tool_outputs.tool_calls)

                tool_outputs = []

                print("Total number of function calls:")
                print(f"\t{len(chatNode.last_run.required_action.submit_tool_outputs.tool_calls)}")

                # Loop through each tool required by a single action step (no information are returned to the LLM before finishing this sequence of actions)
                for tool in chatNode.last_run.required_action.submit_tool_outputs.tool_calls:
                    print("Considered function:")
                    print(f"\t{tool.function.name}")
                    print("Function arguments:")
                    print(f"\t{tool.function.arguments}\n")

                    # Extraction of the function arguments:
                    args = json.loads(tool.function.arguments)
                    

                    # if tool.function.name == "send_robot_to_area":
                        
                    #     # Function execution
                    #     # result = chatNode.send_robot_to_area(args["robot_to_send"], args["area_to_reach"], args["ros_topic"])
                    #     result = chatNode.send_robot_to_area(args["robot_to_send"], args["area_to_reach"])

                    #     # Serve per dire all'LLM che la funzione è stata eseguita
                    #     tool_outputs.append({
                    #     "tool_call_id": tool.id,
                    #     "output": result
                    #     })
                    
                    if tool.function.name == "send_robots_to_area":

                        # Function execution
                        # result = chatNode.send_robot_to_area(args["robot_to_send"], args["area_to_reach"], args["ros_topic"])
                        result = chatNode.send_robots_to_area(args["robots_sequence"])

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
                    
                    elif tool is None:
                        continue

                    else:
                        print("No valid function call")
                    

                if tool_outputs:
                    try:
                        chatNode.last_run = chatNode.client.beta.threads.runs.submit_tool_outputs_and_poll(
                            thread_id=chatNode.thread.id,
                            run_id=chatNode.last_run.id,
                            tool_outputs=tool_outputs
                        )
                        # print("Tool outputs submitted successfully.")
                        # print("Run status after submission of the output:")
                        # print(chatNode.last_run.status)
                    except Exception as e:
                        print("Failed to submit tool outputs:", e)
                else:
                    print("No tool outputs to submit.")
                

                

            if chatNode.last_run.status == 'completed':
                messages = chatNode.client.beta.threads.messages.list(
                    thread_id=chatNode.thread.id
                )
            
                message_dictionary = json.loads(messages.data[0].content[0].text.value)
                #print(message_dictionary)

                assistant_reply = message_dictionary["content"]
                
                if isinstance(assistant_reply,dict):
                    assistant_reply = assistant_reply["content"]

                # print("\nAI:") 
                # print(f"\t{assistant_reply}")

                session["messages"].append({"role": "assistant", "content": assistant_reply})


                response = {
                    "reply": assistant_reply,
                }
                return jsonify(response)


            else:
                print("Run not completed. Current status:")
                print(chatNode.last_run.status)

                print(chatNode.last_run.required_action.submit_tool_outputs.tool_calls)
                return jsonify({"error": "Run not completed", "status": chatNode.last_run.status})

        else:
            print("Error in the response generation. Current status:")
            print(chatNode.last_run.status)
            print(chatNode.last_run.last_error)
            return jsonify({"error": "Error in the response generation.", "status": chatNode.last_run.status})

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



# @app.route('/activate_robot', methods=['POST'])
# def activate_robot():
#     # Ottieni i dati dalla richiesta JSON
#     data = request.get_json()
#     response = data.get('response')

#     chatNode.rosPublisher.publish(response)

#     return jsonify({"status": "success", "response": response})


if __name__ == "__main__":
    app.run(debug=False)
