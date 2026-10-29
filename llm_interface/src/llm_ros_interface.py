import chainlit as cl
from chainlit.input_widget import Select
from chainlit.context import get_context, init_ws_context
from chainlit.session import WebsocketSession
from openai import AsyncOpenAI
import rospy
import rosgraph
from utilitiesOpenAI import *
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from llm_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback
from gazebo_plugins.srv import doorStringCommand, doorStringCommandRequest 
import actionlib
import subprocess
import yaml
import time
import re
import json
import os
import asyncio
from queue import Queue
import base64
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
import math



# Retrieve file paths
script_dir = os.path.dirname(__file__) #/home/[...]/llm_for_surveillance/llm_interface/src

# Retrieve ROS parameters
experiment_type=rospy.get_param("experiment_type", "")
world_id = str(rospy.get_param("world_id", 1))
llm_modality = rospy.get_param("modality", "")
print_rag_search = rospy.get_param("print_rag_search", False)
teleport_robots = rospy.get_param("teleport_robots",True)
sensor_sequence = rospy.get_param("sensor_sequence", "").split(",")
trial_id = rospy.get_param("trial_id", "0")
llm_model = rospy.get_param("llm_model", "")




# if llm_modality in ["rag_with_semantics", "image_and_rag_with_semantics"]:
#     semantic_RAG = True
#     function_RAG = False
    
# elif llm_modality in ["rag_with_functions", "image_and_rag_with_functions"]:
#     semantic_RAG = False
#     function_RAG = True

# elif llm_modality in ["image"]:
#     semantic_RAG = False
#     function_RAG = False

# if llm_modality in ["image", "image_and_rag_with_functions", "image_and_rag_with_semantics"]:
#     image_info = True
# elif llm_modality in ["rag_with_semantics", "rag_with_functions"]:
#     image_info = False

# Creating corresponding file names
file_RAG = f"RAG_map{world_id}.json"
file_prompt = f"prompts_map{world_id}.yaml"
file_building_plan = f"building_plan_map{world_id}.png"
file_ROS_info = f"info_map{world_id}.json"
file_experiments = "experiments.json"

# Preparing correct building plan image
path_building_plan = os.path.join(script_dir, "../config/uploads", file_building_plan)

# Preparing the RAG files for the vector database
file_RAG_paths = [os.path.join(script_dir, "../config/RAG", file_RAG)]
file_RAG_streams = [open(path, "rb") for path in file_RAG_paths]

# Preparing the ROS file configuration
file_ROS_path = os.path.join(script_dir, "../config/ROS", file_ROS_info)

# Preparing experiments file and modality
spatial_reasoning_modalities = ["rag_with_functions", "rag_with_semantics", "image", "image_and_rag_with_functions", "image_and_rag_with_semantics"]
intruder_detection_modalities = ["HITL", "AUTO"]

if llm_modality in spatial_reasoning_modalities:
    experiment_folder = "spatial_reasoning"

    if llm_modality in ["rag_with_semantics", "image_and_rag_with_semantics"]:
        semantic_RAG = True
        function_RAG = False
        
    elif llm_modality in ["rag_with_functions", "image_and_rag_with_functions"]:
        semantic_RAG = False
        function_RAG = True

    elif llm_modality in ["image"]:
        semantic_RAG = False
        function_RAG = False

    if llm_modality in ["image", "image_and_rag_with_functions", "image_and_rag_with_semantics"]:
        image_info = True
    elif llm_modality in ["rag_with_semantics", "rag_with_functions"]:
        image_info = False



elif llm_modality in intruder_detection_modalities:
    experiment_folder = "intruder_detection"

    # We choose the best modality obtained from ablation study
    semantic_RAG = True
    function_RAG = False
    image_info = False
else:
    rospy.logerr("Wrong experiment folder considered")


print("semantic_RAG:", semantic_RAG)
print("function_RAG:", function_RAG)
print("image_info", image_info)


file_experiments_path = os.path.join(script_dir, "../../data_analysis/", experiment_folder, file_experiments)

# Loading the correct prompt based on the specified LLM modality
with open(f"{script_dir}/../config/prompts/{file_prompt}") as f:
    prompts_dict = yaml.load(f, Loader=yaml.SafeLoader)

prompt = prompts_dict[llm_modality]


# Creating the Robot Assistant
class LLM_cognitive_module:
    def __init__(self):
        rospy.init_node('LLM_cognitive_module', anonymous=True)
        self.client = AsyncOpenAI()
        self.llm_model = llm_model # gpt-4o , gpt-6-luna
        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleDataMediatorAlert)

        self.robotGoalCheckerServer = actionlib.SimpleActionServer('/robot_goal_checker', goalCheckerLLMAction, self.handleRobotGoalChecker, auto_start = False)
        self.robotGoalCheckerServer.start()

        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        # Necessario per evitare che si perdano messaggi dai robot quando deploiati insieme
        self.lock = asyncio.Lock()

        # Definiamo una coda, necessaria per gestire gli alert da parte del datamediator 
        self.alert_queue = Queue()

        # Setting of configuration file
        self.required_vector_store_id = None
        self.available_tools = None

        # Extracting experiments data from json file
        with open(file_experiments_path, "r") as f:
            self.experiments_data = json.load(f) 

        # Preparing single run experiment dictionary 
        self.data_experiment_run = {
            "semantic_RAG":semantic_RAG,
            "function_RAG":function_RAG,
            "image_info":image_info,
            "sensor_sequence":sensor_sequence,
            "conversation":[]
        }

        if llm_modality in intruder_detection_modalities:
            self.data_experiment_run["success"] = 0

        
        # ROS network information from file
        with open(file_ROS_path, "r") as f:
            ros_info = json.load(f)

        self.robots_dict = ros_info["ros_publishers"]["robots"]
        self.robots_list = list(self.robots_dict.keys())

        self.areas_dict = ros_info["areas"]
        self.areas_list = list(self.areas_dict.keys())

        self.cameras_dict = ros_info["ros_subscribers"]["sensors"]
        self.cameras_list = list(self.cameras_dict.keys())

        self.actuators_dict = ros_info["ros_services"]["actuators"]
        self.actuators_list = list(self.actuators_dict.keys())


        with open(file_RAG_paths[0], 'r') as f:
            self.RAG_info = json.load(f)

        self.sensors_list = list(self.RAG_info["sensors"].keys())

        self.robot_current_positions = {}
        self.alert_received = False


    async def initialize_assistant(self):
        files_name_list, files_id_list, vector_store_id_list = extractFilesFromJson(self.client, script_dir)
            
        found = False

        if not semantic_RAG:
            # Modalities don't need RAG files for semantic search
            result = subprocess.run(["python3", f"{script_dir}/delete_file_openAI.py"], capture_output=True, text=True)
            print("Output dello script:", result.stdout)
        else:

            if file_RAG in files_name_list:
                # the configuration file is already uploaded. We search for the correct vector_store_id
                for vector_store_id in vector_store_id_list:
                    async for file in self.client.vector_stores.files.list(vector_store_id = vector_store_id):
                        retrieved_file = await self.client.files.retrieve(file_id=file.id)       
                        if retrieved_file.filename == file_RAG:
                            self.required_vector_store_id =  vector_store_id
                            found = True
                            break

                    if found:
                        break
                print(f"Already created file {file_RAG} considered")
            else:
                # Vector store is needed to upload files to the assistant.
                vector_store = await self.client.vector_stores.create(name="Environmental and ROS information for surveillance")
                self.required_vector_store_id = vector_store.id

                # We upload the file
                file_batch = await self.client.vector_stores.file_batches.upload_and_poll(
                    vector_store_id = self.required_vector_store_id, files=file_RAG_streams
                )
                # print(file_batch.status)
                # print(file_batch.file_counts)

                async for file in self.client.vector_stores.files.list(vector_store_id = self.required_vector_store_id):
                    retrieved_file = await self.client.files.retrieve(file_id=file.id)       
                    if retrieved_file.filename == file_RAG:
                        files_name_list.append(file_RAG)
                        files_id_list.append(file.id)
                        vector_store_id_list.append(self.required_vector_store_id)


                updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)
                print(f"New {file_RAG} file uploaded")

        self.available_tools = [
                    # {
                    #     "type": "function", 
                    #     "name": "retrieve_system_state",  
                    #     "description": "Use this function to request the current state of the system from the data mediator. It returns information robot locations, sensors and door actuator states.",
                    #     "parameters": {}
                    # },
                    # {
                    #     "type": "function", 
                    #     "name": "reset_sensors_activation",
                    #     "description": "Use this function to deactivate all sensors in the environment after the user explicit request.",
                    #     "parameters": {} 
                    # },
                    # {
                    #     "type": "function", 
                    #     "name": "show_camera_to_user",
                    #     "description": "It opens a window to allow the user to visualize each camera. Must be used if the plan include allowing user visualization of a camera feed.",
                    #     "parameters": {
                    #         "type": "object",
                    #         "properties": {
                    #             "cameras_names_list": {
                    #                 "type": "array", 
                    #                 "description": "List of all the cameras names to considered for visualization.",
                    #                 "items" : {
                    #                     "type" : "string",
                    #                     "description": "Name of a single camera.",
                    #                     "enum": self.cameras_list
                    #                 }
                    #                 },
                    #             },
                    #         "required": ["cameras_names_list"]
                    #         }
                    # },
                    {
                        "type": "function", 
                        "name": "get_camera_feed",
                        "description": "It returns the image representing the current feed of the sequence of specified cameras. The function must be used to analyze the area related to the considered camera.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "cameras_names_list": {
                                    "type": "array", 
                                    "description": "List of all the cameras names to considered for visualization.",
                                    "items" : {
                                        "type" : "string",
                                        "description": "Name of a single camera.",
                                        "enum": self.cameras_list
                                    }
                                    },
                                },
                            "required": ["cameras_names_list"]
                            }
                    },
                    {
                        "type": "function",
                        "name": "control_actuators",
                        "description": f"It allows to control a sequence of door actuators in the environment.",
                        "parameters": {    
                            "type": "object",
                            "properties": {
                                "actuators_sequence": {
                                    "type": "array",
                                    "description": f"Ordered list of actuator commands.",
                                    "items": {
                                        "type": "object",
                                        "properties": {

                                            "actuator_name": {
                                                "type": "string",
                                                "description": f"Name of the door actuator.",
                                                "enum":self.actuators_list
                                                },
                                            "command": {
                                                "type": "string",
                                                "description": "Action to perform on the actuator.",
                                                "enum": ["open", "close"]
                                                }
                                            },
                                        "required": ["ros_service_name", "command"]
                                    }
                                }
                            },
                        "required": ["actuators_sequence"]
                        }
                    },
                    {
                        "type":"function",
                        "name":"compute_robot_area_distance",
                        "description":"It provides the optimal robot-area allocation based on distance to the areas to inspect. It must be used after the area to inspect are decided and before sending the robots. ",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "areas_to_inspect": {
                                    "type": "array", 
                                    "description": "List of all the area names where the robot should be deployed.",
                                    "items" : {
                                        "type" : "string",
                                        "description": "Name of a single area to inspect.",
                                        "enum": self.areas_list
                                    }
                                    },
                                },
                            "required": ["areas_to_inspect"]
                        }
                    },
                    {
                        "type": "function", 
                        "name": "send_robots_to_area",
                        "description": "Use this function to deploy a sequence of robots to corresponding areas.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "robots_sequence": {
                                    "type": "array",
                                    "description": "Ordered list of robots to deploy.",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "robot_to_send": {
                                                "type": "string",
                                                "description": "Name of the considered robot to deploy.",
                                                "enum": self.robots_list
                                                },
                                            "area_to_reach": {
                                                "type": "string",
                                                "description": "Letter of the area where the corresponding robot must be deploy.",
                                                "enum": self.areas_list
                                                }
                                            },
                                        "required": ["robot_to_send", "area_to_reach"]
                                    }
                                },                                    
                                },
                            "required": ["robots_sequence"]
                            }
                    }
                ]
        
        if llm_modality == "HITL":
            camera_functions = [
                    {
                        "type": "function", 
                        "name": "show_camera_to_user",
                        "description": "It opens a window to allow the user to visualize each camera. Must be used if the plan include allowing user visualization of a camera feed.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "cameras_names_list": {
                                    "type": "array", 
                                    "description": "List of all the cameras names to considered for visualization.",
                                    "items" : {
                                        "type" : "string",
                                        "description": "Name of a single camera.",
                                        "enum": self.cameras_list
                                    }
                                    },
                                },
                            "required": ["cameras_names_list"]
                            }
                    },
            ]

        if semantic_RAG:
            RAG_functions = [
                {
                    "type":"file_search",
                    "vector_store_ids": [self.required_vector_store_id],
                    "max_num_results": 20,
                }
            ]
        elif function_RAG:
            RAG_functions = [
                {
                    "type": "function", 
                    "name": "get_sensors_info",  
                    "description": "Use this function to retrieve the information related to a given sequence of sensors. It returns the area covered by cameras and associated to lidars.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "sensor_names_list": {
                                "type": "array", 
                                "description": "List of all the sensor names for which you want to retrieve information.",
                                "items" : {
                                    "type" : "string",
                                    "description": "Name of a single sensor.",
                                    "enum": self.sensors_list
                                }
                                },
                            },
                        "required": ["cameras_names_list"]
                    }
                },
                {
                    "type": "function", 
                    "name": "get_areas_info",  
                    "description": "Use this function to retrieve the information related to a given sequence of areas. It returns information about all the sensors, actuators and other areas connection for a given area.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "area_names_list": {
                                "type": "array", 
                                "description": "List of all the area names for which you want to retrieve information.",
                                "items" : {
                                    "type" : "string",
                                    "description": "Name of a single area.",
                                    "enum": self.areas_list
                                }
                                },
                            },
                        "required": ["cameras_names_list"]
                    }
                },
            ]
        else:
            RAG_functions = []

        for function in RAG_functions:
            self.available_tools.append(function)

        for function in camera_functions:
            self.available_tools.append(function)
            
    def log_conversation_message(self, log, authority, resp_time=None, gen_time=None):
        data = {authority:log}

        if resp_time is not None:
            data["resp_time"] = round(resp_time,2)

        if gen_time is not None:
            data["gen_time"] = round(gen_time,2)

        self.data_experiment_run["conversation"].append(data)

        return

    def save_experiment_run(self):
        trial_name = f"trial_{trial_id}"
        world_name = f"world_{world_id}"

        # Select the correct root of the JSON structure
        if experiment_folder == "intruder_detection":
            root_data = self.experiments_data.setdefault(experiment_type, {})
        else:
            root_data = self.experiments_data

        
        # Check dictionary keys
        if llm_modality not in root_data:
            rospy.logerr(f"Key '{llm_modality}' not present in {file_experiments}")

            # We add the new llm modality
            root_data.update({llm_modality:{}})
        
        if world_name not in root_data[llm_modality]:
            rospy.logerr(f"Key '{world_name}' not present in experiments_data['{llm_modality}']")

            # We add the new world name
            root_data[llm_modality].update({world_name:{}})

        if self.llm_model not in root_data[llm_modality][world_name]:
            rospy.logerr(f"Key '{self.llm_model}' not present in experiments_data['{llm_modality}']['{world_name}']")

            # We add the new llm model
            root_data[llm_modality][world_name].update({self.llm_model:{}})

        if trial_name not in root_data[llm_modality][world_name][self.llm_model]:
            rospy.logerr(f"Key '{trial_name}' not present in experiments_data['{llm_modality}']['{world_name}']['{self.llm_model}']")
            
            # We add the new trial
            root_data[llm_modality][world_name][self.llm_model].update({trial_name:{}})


        # Saving run in the correct trial
        trial_runs = root_data[llm_modality][world_name][self.llm_model][trial_name]

        new_run = True

        for existing_run_name, run_data in trial_runs.items():
            if sensor_sequence == run_data["sensor_sequence"]:
                new_run = False
                run_name = existing_run_name
                break

        if new_run:
            run_id = len(trial_runs) + 1
            run_name = f"run_{run_id}"

        # Save experiment run
        trial_runs[run_name] = self.data_experiment_run

        text = json.dumps(self.experiments_data, indent=2, ensure_ascii=False)
        text = re.sub(r'\[\s*((?:"[^"]*"(?:,\s*)?)*)\s*\]',
                    lambda m: '[' + re.sub(r',\s*', ', ', m.group(1)) + ']',
                    text)
        text = re.sub(r'\{\s*("(?:[^"\\]|\\.)*":\s*(?:"(?:[^"\\]|\\.)*"|\d+|true|false|null)(?:,\s*"(?:[^"\\]|\\.)*":\s*(?:"(?:[^"\\]|\\.)*"|\d+|true|false|null))*)\s*\}',
                    lambda m: '{' + m.group(1).replace('\n', '') + '}',
                    text)

        with open(file_experiments_path, "w", encoding="utf-8") as f:
            f.write(text)
        
        return 
    
    async def handle_message(self, msg, authority):
        async with self.lock:
            # Riprendiamo lo storico della conversazione a cui aggiungeremo il nuovo messaggio
            
            history = cl.user_session.get("history")
            history.append({"role": authority, "content": msg})

            if self.alert_received:
                self.log_conversation_message(log=msg, authority="data_orchestrator")
                self.alert_received = False
            else:
                self.log_conversation_message(log=msg, authority=authority)

            # Creiamo un messaggio vuoto che "riempiremo" con lo streaming
            msg = cl.Message(content="")

            requires_action = True

            rag_start_times = {}
            tool_start_times = {}

            while requires_action:
                llm_request_start = time.perf_counter()
                first_text_time = None
                llm_response_time = None
                llm_generation_time = None

                requires_action = False # Di default pensiamo di finire. Se esegue un tool è necessario continuare

                full_ai_response = ""
                final_tool_calls = {}
                active_steps = {} # Teniamo traccia degli step attivi per indice
                tool_outputs = []

                try:
                    response_stream = await self.client.responses.create(
                        model=self.llm_model,
                        input=history,
                        tools = self.available_tools,
                        include=["file_search_call.results"],
                        stream=True
                    )                
                    async with response_stream as stream:
                        async for event in stream:
                            # 1. GESTIONE TESTO
                            if event.type == "response.created":
                                pass
                            
                            elif event.type == "response.failed":
                                # Per avvisare direttamente in chat che c'è stato un errore
                                await cl.Message(content=f"Error: {event.error.message}").send()
                                break

                            # --- CASI DEL TESTO (OUTPUT) ---
                            elif event.type == "response.output_text.delta":
                                if first_text_time is None:
                                    first_text_time = time.perf_counter()

                                if not msg.id: 
                                    await msg.send()
                                full_ai_response += event.delta
                                await msg.stream_token(event.delta)

                            elif event.type == "response.text.done":
                                # Testo terminato, possiamo aggiornare lo stato finale
                                await msg.update()

                            # TOOLS
                            elif event.type == "response.output_item.added":
                                # Viene aggiunto un nuovo item (testo o funzione)
                                if event.item.type == "function_call":
                                    idx = event.output_index
                                    tool_name = event.item.name 

                                    tool_start_times[idx] = time.perf_counter()
                                    
                                    # Step for chainlit visualization 
                                    step = cl.Step(name=tool_name, type="tool")
                                    step.language = "json" # Se vuoi mostrare i parametri
                                    await step.send()

                                    final_tool_calls[idx] = event.item # passiamo l'intero item 
                                    active_steps[idx] = step

                                elif event.item.type == "file_search_call":
                                    rag_start_times[event.item.id] = time.perf_counter()
                            
                            elif event.type == "response.output_item.done":
                                
                                if event.item.type == "file_search_call":
                                    rag_end = time.perf_counter()
                                    rag_start = rag_start_times.pop(event.item.id, None)

                                    if rag_start is not None:
                                        rag_elapsed = rag_end - rag_start

                                    results = event.item.results or [] 
                                    complete_file_search_call = ""
                                    number_of_chunks = len(results) + 1

                                    for i, result in enumerate(results, start=1):
                                        complete_file_search_call =  complete_file_search_call + result.text 

                                    rag_log = {
                                        "number_of_chunks":number_of_chunks,
                                        "retrieved_info":complete_file_search_call
                                    }

                                    if llm_modality in intruder_detection_modalities:
                                        self.log_conversation_message(log=rag_log, authority="file_search_call", resp_time=rag_elapsed)
                                    else:
                                        self.log_conversation_message(log=rag_log, authority="file_search_call")

                                    # To visualize the obtained text from RAG
                                    if print_rag_search:  
                                        print("\n\n=== FILE SEARCH RESULTS ===")
                                        print(f"\n--- Chunk {i} ---")
                                        for i, result in enumerate(results, start=1):
                                            print(f"\nFile: {result.filename}")
                                            print(f"File ID: {result.file_id}")
                                            print(f"Retrieved text:\n{result.text}")
                                            print(f"Score: {result.score}")
                                            print(f"Attributes: {result.attributes}")

                            elif event.type == "response.function_call_arguments.delta":
                                idx = event.output_index
                                if idx in final_tool_calls:
                                    # Accumulo asincrono dei parametri JSON
                                    if final_tool_calls[idx].arguments is None:
                                        final_tool_calls[idx].arguments = ""
                                    final_tool_calls[idx].arguments += event.delta

                                if idx in active_steps:
                                    if active_steps[idx].input is None:
                                        active_steps[idx].input = ""
                                    
                                    await active_steps[idx].stream_token(event.delta, is_input=True)

                            elif event.type == "response.function_call_arguments.done":
                                # Parametri pronti!
                                idx = event.output_index

                                start_time = tool_start_times.pop(idx, None)
                                if start_time is not None:
                                    tool_call_time = time.perf_counter() - start_time

                                tool_call = final_tool_calls[idx]
                                step = active_steps[idx] # Recuperiamo lo step creato in .added
                                args = event.arguments # event.arguments è la stringa completa
                                
                                # Esecuzione ROS effettiva
                                result = await cl.make_async(self.execute_ros_command)(tool_call.name, json.loads(args))

                                # Aggiungi la chiamata "completata" alla cronologia
                                history.append({
                                    "type": "function_call",
                                    "call_id": tool_call.call_id,
                                    "name": tool_name,
                                    "arguments": args
                                })

                                log = f"name - {tool_name} \n\targuments - {args}"
                                if llm_modality in intruder_detection_modalities:
                                    self.log_conversation_message(log=log, authority="function_call", resp_time=tool_call_time)
                                else:
                                    self.log_conversation_message(log=log, authority="function_call")

                                
                                # Add ROS output
                                if tool_call.name == "get_camera_feed": 
                                    # We must manage the dictionary received by the function containing both the image and the feedback
                                    camera_data = json.loads(result)
                                    images = []

                                    for camera_name in camera_data.keys():
                                        # Extraction of the image from the dictionary
                                        image_base64 = camera_data[camera_name].pop("retrieved_image_base64", "")
                                        info = camera_data[camera_name]

                                        if (info["name"]["correctness"] and info["image_conversion"]["success"] and image_base64):
                                            images.append({
                                                "type": "input_text",
                                                "text": f"Frame from camera: {camera_name}"
                                            })

                                            images.append({
                                                "type": "input_image",
                                                "image_url": (
                                                    f"data:image/jpeg;base64,{image_base64}"
                                                ),
                                                "detail": "high"
                                            })

                                    output = [{
                                            "type": "input_text",
                                            "text": json.dumps(camera_data, ensure_ascii=False)
                                        },
                                        *images
                                    ]
                                    
                                    log = json.dumps(camera_data, ensure_ascii=False)

                                else:
                                    output = str(result)
                                    log = str(result)

                                # Chiude lo step legato all'output
                                active_steps[idx].output = log
                                await active_steps[idx].update()
                                                                

                                history.append({
                                    "type": "function_call_output",
                                    "call_id": tool_call.call_id,
                                    "output": output
                                })

                                
                                self.log_conversation_message(log=log, authority="ROS_output")

                                # Aggiunta ai tool chiamati in modo che sia possibile verificare la presenza di tool
                                tool_outputs.append({
                                    "type": "function_call_output",
                                    "call_id": tool_call.call_id,
                                    "output": output
                                })

                                # Se abbiamo eseguito un tool, dobbiamo fare un altro giro di loop in modo che LLM riceva l'output del tool
                                requires_action = True

                                

                            # CASI DI RAGIONAMENTO (REASONING)
                            elif event.type == "response.reasoning.delta":
                                # Se usi modelli o1/o3, qui arriva il "pensiero" del modello.
                                # Puoi scegliere se mostrarlo o ignorarlo.
                                pass

                            elif event.type == "response.completed":
                                response_end_time = time.perf_counter()

                                # Tempo totale: invio richiesta -> completamento risposta
                                llm_response_time = response_end_time - llm_request_start

                                # Tempo generazione testo: primo token -> completamento risposta
                                if first_text_time is not None:
                                    llm_generation_time = response_end_time - first_text_time


                                if msg.id:
                                    await msg.update()
                                break # Esci dal ciclo per sicurezza
                    
                except Exception as e:
                    rospy.logerr(f"Error in the LLM response generation: \n {e}")
                

                if full_ai_response:
                        history.append({"role": "assistant", "content": full_ai_response})

                        if llm_modality in intruder_detection_modalities:
                            self.log_conversation_message(log=full_ai_response, authority="assistant", resp_time=llm_response_time, gen_time=llm_generation_time)
                        else:
                            self.log_conversation_message(log=full_ai_response, authority="assistant")
            
            
            cl.user_session.set("history", history)
            rospy.set_param("/chainlit_done", True)


    def execute_ros_command(self, tool_name, args):
        """
        Dispatcher che smista le chiamate ai metodi ROS specifici.
        Viene chiamata tramite cl.make_async per non bloccare la UI.
        """
        try:
            # 1. Verifica se il metodo esiste nella classe
            if hasattr(self, tool_name):
                method = getattr(self, tool_name)
                
                # 2. Log per il terminale ROS
                rospy.loginfo(f"Esecuzione tool: {tool_name} con argomenti: {args}")
                
                # 3. Chiamata dinamica al metodo 
                result = method(**args)
                
                return result
            else:
                error_msg = f"Errore: Il comando '{tool_name}' non è implementato nel nodo ROS."
                rospy.logerr(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Eccezione durante l'esecuzione di {tool_name}: {str(e)}"
            rospy.logerr(error_msg)
            return error_msg
    
    
    def clear_environment_state(self, environment_state):
        """Clear the information obtained by the data mediator"""
        with open(f"{script_dir}/../config/ROS/{file_ROS_info}", "r") as f:
            ROS_info = json.load(f)

        areas_dict = ROS_info["areas"]
    
        del environment_state["robots_list_names"]
        del environment_state["sensors_list_names"]
        del environment_state["actuators_list_names"]

        robots_name = environment_state["robots"].keys()

        for robot_name in robots_name:
            
            x, y, z = environment_state["robots"][robot_name]["current_position"]
   
            for area_name, area_info in areas_dict.items():
                x_range = area_info["coordinate_ranges"]["x"]
                y_range = area_info["coordinate_ranges"]["y"]

                if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                    environment_state["robots"][robot_name]["current_area"] = area_name
                    break
            
            del environment_state["robots"][robot_name]["current_position"]
            del environment_state["robots"][robot_name]["current_orientation"]
        
        del environment_state["sensors"]


        doors_name = environment_state["actuators"].keys()

        for door_name in doors_name:
            del environment_state["actuators"][door_name]["position"]
        
        return environment_state


    def retrieve_system_state(self):
        response = self.retrieveSystemStateClient()
        system_state = self.clear_environment_state(json.loads(response.system_state))    
        return json.dumps(system_state)


    def handleDataMediatorAlert(self, req):
        if req:        
            try:
                self.alert_queue.put(
                    {
                        "type": "ALERT_SERVICE",
                        "content": req,
                        }
                    )
                self.alert_received = True
                rospy.loginfo("Service added to queue")
                return triggerGptResponse("Success")
            except Exception as e:
                return triggerGptResponse(f"Error: {str(e)}")
        return triggerGptResponse("Failed")
    

    async def async_process_queue(self, session_id):
            try:
                # 1. Recupera la sessione tramite l'ID
                session = WebsocketSession.get_by_id(session_id)
                # 2. Inizializza il contesto usando quella sessione
                init_ws_context(session)
            except Exception as e:
                print(f"Error in setting the session id: {e}")

            while True:
                if not self.alert_queue.empty():
                    try:
                        item = self.alert_queue.get()
                        item_type = item.get("type")
                        
                        if item_type == "ALERT_SERVICE":
                            # La task nella coda è il processing di un servizio ROS
                            req = item.get("content")
                            # Estrai e pulisci il messaggio ricevuto via ros service dal data mediator
                            alert_info = self.clear_environment_state(json.loads(req.alert_info))

                            # Estraiamo i sensori attivi
                            active_sensors = alert_info["activated_sensors"]

                            # Trasformiamo la lista in una stringa di elementi avvolti da backtick `
                            sensors_formatted = ", ".join([f"`{sensor}`" for sensor in active_sensors])

                            await cl.Message(
                                author="Data Mediator",
                                content=f"### 🚨 **DATA MEDIATOR ALERT**\n\n**Activated sensors:** {sensors_formatted}",
                            ).send()

                            # Messaggio da inviare al'LLM
                            message = f"Environmental alert detected. Current environment state: {json.dumps(alert_info)}. Propose a strategy to inspect the environment."


                        elif item_type == "ROBOT_ACTION":
                            # La task nella coda è il processing di un' action ROS
                            goal = item.get("content")

                            # Messaggio da inviare al'LLM
                            message = goal.message_for_LLM

                        # Chiamata al modello per rispondere
                        
                        await self.handle_message(msg=message, authority="user")
                        await show_experiment_controls()
                        
                    except Exception as e:
                        rospy.logerr(f"Error in handling the queue: \n {e}")
            
                await asyncio.sleep(0.1)
        
    def reset_sensors_activation(self):
        response = self.resetSensorsActivationClient()
        return f"All sensors deactivated"

    def compute_robot_area_distance(self,areas_to_inspect):

        robot_area_allocation = {}

        for area in areas_to_inspect:
            area_to_inspect_coordinates =  self.areas_dict[area]["coordinates"]

            robot_area_allocation[area] = {}

            for robot in self.robots_list:
                # Create the client to retrieve the robot position from gazebo
                rospy.wait_for_service('/gazebo/get_model_state')
            
                get_model_state_client = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

                current_position = get_model_state_client(model_name=robot, relative_entity_name='map')

                # We use the same message received by Gazebo as starting point to generate the path towards the goal
                start = PoseStamped()
                start.header.frame_id = 'map'
                start.header.stamp = rospy.Time(0)

                # Defining the initial pose of the path
                start.pose = current_position.pose

                # Defining the final pose to reach
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = rospy.Time(0)

                goal.pose.position.x = area_to_inspect_coordinates["x"]
                goal.pose.position.y = area_to_inspect_coordinates["y"]
                goal.pose.position.z = 0.0

                goal.pose.orientation.x = 0.0
                goal.pose.orientation.y = 0.0
                goal.pose.orientation.z = 0.0
                goal.pose.orientation.w = 1.0

                # Create the client to call the make_plan service
                rospy.wait_for_service(f'{robot}/move_base/make_plan')
                make_plan_client = rospy.ServiceProxy(f'{robot}/move_base/make_plan', GetPlan)

                # Generate the path
                response = make_plan_client(start=start, goal=goal, tolerance=0.2)

                # Computing the total distance
                poses = response.plan.poses
                total_distance = 0.0

                for i in range(1, len(poses)):
                    dx = (poses[i].pose.position.x - poses[i - 1].pose.position.x)
                    dy = (poses[i].pose.position.y - poses[i - 1].pose.position.y)
                    total_distance += math.hypot(dx, dy)

                robot_area_allocation[area][robot] = round(total_distance,2)

        return json.dumps(robot_area_allocation)

    def send_robots_to_area(self, robots_sequence):

        robot_deployment_correctness = {}
        
        for robot_to_deploy in robots_sequence:
            robot = robot_to_deploy.get("robot_to_send")
            area = robot_to_deploy.get("area_to_reach")

            if robot in self.robots_list:    
                if area in self.areas_list:
                
                    area_x = self.areas_dict[area]["coordinates"]["x"]
                    area_y = self.areas_dict[area]["coordinates"]["y"]


                    if teleport_robots:
                        # Create a temporary client to call the gazebo service to teleport a given model
                        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

                        model_state = ModelState()
                        model_state.model_name = robot  # name of robot model in Gazebo
    
                        model_state.pose.position.x = area_x-0.3 # small offset to allow the robot to actually reach the goal triggering the move_base
                        model_state.pose.position.y = area_y-0.3 # small offset to allow the robot to actually reach the goal triggering the move_base
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
    
                        response = set_model_state(model_state)


                    # We publish the final goal to reach for each robot in order to trigger the check robot goal
                    topic = self.robots_dict[robot]["ros_topic"]

                    # Create a temporary client to call the clear_costmap service for all robots:
                    service_name = robot + "/move_base/clear_costmaps"
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
    

    def control_actuators(self, actuators_sequence):
        """Function to control actuators"""
        master = rosgraph.Master('/rospy')
        services_info = master.getSystemState()[2]  # [publishers, subscribers, services]

        # Estrai solo i nomi dei servizi
        ros_service_names_list = [service_info[0] for service_info in services_info]

        responses_list = []

        for actuator in actuators_sequence:

            actuator_name = actuator.get("actuator_name")
            command = actuator.get("command")
            ros_service_name = self.actuators_dict[actuator_name]["ros_service_name"]

            if ros_service_name in ros_service_names_list:
                try:
                    rospy.wait_for_service(ros_service_name, timeout=5)
                except rospy.ROSException as e:
                    rospy.logerr(f"Service {ros_service_name} not available: {e}")
                    response = {
                        "actuator": actuator_name,
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
                        "additional_info" : f"Wrong ros service name considered. Check again in the {file_RAG} file"
                    }
                responses_list.append(response)
        
        return str(responses_list)


    def handleRobotGoalChecker(self,goal):

        if not goal:
            rospy.logerr(f"No message received by robot")
            result.success = f"Function handleRobotGoalChecker: No message received from robot"
            self.robotGoalCheckerServer.set_succeeded(result)
            return 

        try:
            feedback = goalCheckerLLMFeedback()
            result = goalCheckerLLMResult()

            # Invio feedback a Robot verification module
            feedback.status_LLM = "Sending robot goal success to LLM..."
            self.robotGoalCheckerServer.publish_feedback(feedback)
            
            self.alert_queue.put(
                {
                    "type": "ROBOT_ACTION",
                    "content": goal,
                }
            )
            rospy.loginfo("Action added to queue")

            result.success = f"Message from robot correctly processed by the LLM"

        except Exception as e:        
            rospy.logerr(f"Error in the response generation inside the function handleRobotGoalChecker: \n {e}")
            result.success = f"Message from robot NOT processed by the LLM"
                    
        self.robotGoalCheckerServer.set_succeeded(result)
        return 

    def get_sensors_info(self, sensor_names_list):

        sensor_information = {}

        for sensor in sensor_names_list:
            sensor_information[sensor] = self.RAG_info["sensors"][sensor]

        return json.dumps(sensor_information)

    def get_areas_info(self, area_names_list):
        areas_information = {}
        
        for area in area_names_list:
            areas_information[area] = self.RAG_info["areas"][area]

        return json.dumps(areas_information)

    def get_camera_feed(self, cameras_names_list):
        """Acquire one frame from each provided camera in the list and convert it in a suitable format for the LLM."""


        if llm_modality == "HITL":
            self.show_camera_to_user(cameras_names_list)

        response = {}
        bridge = CvBridge()

        for camera_name in cameras_names_list:
            camera_name_lower = camera_name.lower()

            # Initialize response structure
            response[camera_name_lower] = {
                "name": {
                    "correctness": False,
                    "reason": ""
                },
                "image_conversion": {
                    "success": False,
                    "reason": ""
                },
                "retrieved_image_base64": ""
            }

            # Check camera name correctness
            if camera_name_lower not in self.cameras_list:
                response[camera_name_lower]["name"]["reason"] = "wrong name"
                continue

            response[camera_name_lower]["name"]["correctness"] = True

            topic = self.cameras_dict[camera_name_lower]["ros_topic"]

            try:
                # Acquire one frame
                msg = rospy.wait_for_message(
                    topic,
                    Image,
                    timeout=5.0
                )

                # Convert ROS Image to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8"
                )

                # Encode image as JPEG
                success, buffer = cv2.imencode(".jpg", cv_image)

                if not success:
                    raise RuntimeError("JPEG encoding failed")

                # Store result
                response[camera_name_lower]["image_conversion"]["success"] = True
                response[camera_name_lower]["retrieved_image_base64"] = (
                    base64.b64encode(buffer.tobytes()).decode("utf-8")
                )

            except Exception as e:
                response[camera_name_lower]["image_conversion"]["reason"] = str(e)

        return f"{json.dumps(response)}"
        
    
    def show_camera_to_user(self, cameras_names_list):        
        """Function to open camera feed"""
        camera_names_correctness = {}
        topics_list = []

        for camera_name in cameras_names_list:

            camera_name_lower = camera_name.lower()

            if camera_name_lower in self.cameras_list:
                topics_list.append(self.cameras_dict[camera_name_lower]["ros_topic"])
                camera_names_correctness[camera_name_lower] = {
                    "correctness": True,
                    "reason": ""
                }  
            
            else:
                camera_names_correctness[camera_name_lower] = {
                    "correctness": False,
                    "reason": "wrong name"
                }    
                # print(camera_name)
                # print(camera_name_lower)
        
        subprocess.Popen(["python3", f"{script_dir}/display_camera.py"] + topics_list)

        return f"Summary of cameras names correctness: {json.dumps(camera_names_correctness)}"
            
        


agent = LLM_cognitive_module()

# --- Integrazione Chainlit ---
@cl.on_chat_start
async def start():
    await agent.initialize_assistant()

    try:
        print("Contesto Chainlit catturato con successo!")
    except Exception as e:
        print(f"Errore critico nel catturare il contesto: {e}")
    

    settings = await cl.ChatSettings([
        Select(
            id="model",
            label="OpenAI - modello",
            values=["gpt-4o", "gpt-6-luna"], 
            initial_index=0,
        )
    ]).send()

    history = []
     
    # Uploading the prompt
    history.append({"role": "user", "content": str(prompt)})

    # Uploading the image
    with open(path_building_plan, "rb") as f:
        img_b64 = base64.b64encode(f.read()).decode("utf-8")

    #if llm_modality in ["HITL", "AUTO","image", "image_and_rag_with_functions", "image_and_rag_with_semantics"]:
    if image_info:
        history.append(
            {
                "role": "user", 
                "content": [
                        {
                            "type": "input_image",
                            "image_url": f"data:image/png;base64,{img_b64}",
                        },
                    ],
                }
            )
        agent.log_conversation_message(log="Added building_plan image",authority="image")
        


    cl.user_session.set("history", history)
    cl.user_session.set("model", settings["model"])
    cl.user_session.set("agent", agent)

    context = get_context()
    session_id = context.session.id

    # Per gestire gli alert da parte del data mediator creiamo una coda in cui il servizio ros
    # va ad inserire la richiesta da processare da parte dell'alert. In seguito dentro il 
    # servizio ros si processa la coda tramite la funzine "async_process_queue"
    asyncio.create_task(agent.async_process_queue(session_id))
    

async def show_experiment_controls():
    await cl.Message(
        content="### 🎮 Experiment commands",
        actions=[
            cl.Action(
                name="next_sequence",
                label="⏭️ Next sensor sequence",
                value="next_sequence",
                payload={},
            ),
            cl.Action(
                name="stop_experiment",
                label="🛑 Stop experiment",
                value="stop_experiment",
                payload={},
            ),
            cl.Action(
                name="false_positive",
                label="⚠️ False positive experiment",
                value="false_positive",
                payload={},
            ),
            cl.Action(
                name="intruder_detected",
                label="✅ Intruder detected",
                value="intruder_detected",
                payload={},
            ),
        ],
    ).send()

@cl.action_callback("next_sequence")
async def next_sequence(action: cl.Action):
    try:
        rospy.set_param("/next_sequence", True)
        await cl.Message(
            content="⏭️ Next sensor sequence request sent."
        ).send()
    except Exception as exc:
        await cl.Message(content=f"ROS error: {exc}").send()


@cl.action_callback("stop_experiment")
async def stop_experiment(action: cl.Action):
    try:
        rospy.set_param("/stop_experiment", True)
        await cl.Message(
            content="🛑 Stop experiment request sent."
        ).send()
    except Exception as exc:
        await cl.Message(content=f"ROS error: {exc}").send()

@cl.action_callback("intruder_detected")
async def intruder_detected(action: cl.Action):
    try:
        rospy.set_param("/intruder_detected", True)
        await cl.Message(
            content="✅ Detection success set."
        ).send()

        intruder_final_area = rospy.get_param("/intruder_final_area")
        agent.data_experiment_run["intruder_final_area"] = intruder_final_area
        agent.data_experiment_run["success"] = 1

    except Exception as exc:
        await cl.Message(content=f"ROS error: {exc}").send()

@cl.action_callback("false_positive")
async def false_positive(action: cl.Action):
    try:
        await cl.Message(
            content="⚠️ Experiment considered as false positive."
        ).send()

        intruder_final_area = rospy.get_param("/intruder_final_area", "")
        agent.data_experiment_run["intruder_final_area"] = intruder_final_area

        if intruder_final_area == "FP":
            agent.data_experiment_run["success"] = 1
        else:
            agent.data_experiment_run["success"] = 0

    except Exception as exc:
        await cl.Message(content=f"ROS error: {exc}").send()
        

# Setta comandi di test che si possono usare per inserire prompt di default
@cl.set_starters
async def set_starters():
    return [
        cl.Starter(
            label="Retrieve system state",
            message="Provide me the system state of the environment",
            icon="/public/info.svg",
            ),
        cl.Starter(
            label="Test robot deployment",
            message="Send robot 1 in area A",
            icon="/public/forward.svg",
            ),
        cl.Starter(
            label="Close all doors",
            message="Close all the doors",
            icon="/public/door-closed.svg",
            ),
        ]

@cl.on_settings_update
async def update_settings(settings):
    cl.user_session.set("model", settings["model"])

@cl.on_message
async def main(message: cl.Message):
    """Eseguito ogni volta che l'utente invia un messaggio"""
    
    agent = cl.user_session.get("agent")
    await agent.handle_message(msg=message.content, authority="user")
    await show_experiment_controls()


@cl.on_chat_end
def on_chat_end():
    print("Shutting down Chainlit: saving conversation ...")
    valid_run = rospy.get_param("valid_run", False)
    if valid_run:
        agent.save_experiment_run()