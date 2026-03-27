import chainlit as cl
from chainlit.input_widget import Select
from openai import AsyncOpenAI
import rospy
import rosgraph
from utilitiesOpenAI import *
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from geometry_msgs.msg import PoseStamped
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



#retirve file paths
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

file_paths = [os.path.join(script_dir, "..", "config", file_config)]
file_streams = [open(path, "rb") for path in file_paths]

with open(f"{script_dir}/../config/{file_prompt}") as f:
    prompts_dict = yaml.load(f, Loader=yaml.SafeLoader)
prompt_type = "man_in_the_loop"
prompt = prompts_dict[prompt_type]

class RobotAssistant:
    def __init__(self):
        rospy.init_node('RobotAssistant', anonymous=True)
        self.client = AsyncOpenAI()

        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleDataMediatorAlert)

        self.robotGoalCheckerServer = actionlib.SimpleActionServer('/robot_goal_checker', goalCheckerLLMAction, self.handleRobotGoalChecker, auto_start = False)
        self.robotGoalCheckerServer.start()

        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        # Necessario per evitare che si perdano messaggi dai robot quando deploiati insieme
        self.lock = asyncio.Lock()

        # Definiamo una coda, necessaria per gestire gli alert da parte del datamediator (in quanto i servizi ros non sono )
        self.alert_queue = Queue()

        # Setting of configuration file
        self.required_vector_store_id = None
        self.available_tools = None
        
        # Setting for log the history
        self.history_log_file = "history.txt"

        # When we start the script we clear the file
        with open(self.history_log_file, 'w') as f:
            pass
        
        # ROS network information from file
        with open(file_paths[0], "r") as f:
            info = json.load(f)

        self.robots_dict = info["ros_publishers"]["robots"]
        self.robots_list = list(self.robots_dict.keys())

        self.areas_dict = info["areas"]
        self.areas_list = list(self.areas_dict.keys())

        self.sensors_dict = info["ros_subscribers"]["sensors"]
        self.cameras_list = list(self.sensors_dict.keys())

        self.actuators_dict = info["ros_services"]["actuators"]
        self.actuators_list = list(self.actuators_dict.keys())


    async def initialize_assistant(self):
        files_name_list, files_id_list, vector_store_id_list = extractFilesFromJson(self.client, script_dir)
            
        found = False

        if file_config in files_name_list:
            # the configuration file is already uploaded. We search for the correct vector_store_id
            for vector_store_id in vector_store_id_list:
                async for file in self.client.vector_stores.files.list(vector_store_id = vector_store_id):
                    retrieved_file = await self.client.files.retrieve(file_id=file.id)       
                    if retrieved_file.filename == file_config:
                        self.required_vector_store_id =  vector_store_id
                        found = True
                        break

                if found:
                    break
            print(f"Already created file {file_config} considered")
        else:
            # Vector store is needed to upload files to the assistant.
            vector_store = await self.client.vector_stores.create(name="Environmental and ROS information for surveillance")
            self.required_vector_store_id = vector_store.id

            # We upload the file
            file_batch = await self.client.vector_stores.file_batches.upload_and_poll(
                vector_store_id = self.required_vector_store_id, files=file_streams
            )
            # print(file_batch.status)
            # print(file_batch.file_counts)

            async for file in self.client.vector_stores.files.list(vector_store_id = self.required_vector_store_id):
                retrieved_file = await self.client.files.retrieve(file_id=file.id)       
                if retrieved_file.filename == file_config:
                    files_name_list.append(file_config)
                    files_id_list.append(file.id)
                    vector_store_id_list.append(self.required_vector_store_id)


            updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)
            print(f"New {file_config} file uploaded")

        self.available_tools = [
                    {
                        "type":"file_search",
                        "vector_store_ids": [self.required_vector_store_id]
                    },
                    {
                        "type": "function", 
                        "name": "retrieve_system_state",  
                        "description": "Use this function to request the current state of the system from the data mediator. It returns information robot locations, sensors and door actuator states.",
                        "parameters": {}
                    },
                    {
                        "type": "function", 
                        "name": "reset_sensors_activation",
                        "description": "Use this function to deactivate all sensors in the environment after the user explicit request.",
                        "parameters": {} 
                    },
                    {
                        "type": "function", 
                        "name": "display_cameras",
                        "description": "It opens a window to visualize each camera. The function must be used every time a camera feed is required or proposed.",
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
                        "name": "control_actuator",
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
    
    def log_message_in_history_file(self, log, authority):
        with open(self.history_log_file, "a") as f:
            f.write(f"{authority}: \n\t{log}\n\n")
        
        return
    

    async def handle_message(self, msg, authority):
        async with self.lock:
            # Riprendiamo lo storico della conversazione a cui aggiungeremo il nuovo messaggio
            
            history = cl.user_session.get("history")
            history.append({"role": authority, "content": msg})

            self.log_message_in_history_file(log=msg, authority=authority)

            # Creiamo un messaggio vuoto che "riempiremo" con lo streaming
            msg = cl.Message(content="")

            requires_action = True

            while requires_action:
                requires_action = False # Di default pensiamo di finire. Se esegue un tool è necessario continuare

                full_ai_response = ""
                final_tool_calls = {}
                active_steps = {} # Teniamo traccia degli step attivi per indice
                tool_outputs = []
                
                try:
                    response_stream = await self.client.responses.create(
                        model="gpt-4o",
                        input=history,
                        tools = self.available_tools,
                        stream=True
                    )                
                    async with response_stream as stream:
                        async for event in stream:
                            # 1. GESTIONE TESTO
                            if event.type == "response.created":
                                pass # Ottimo per loggare l'ID della risposta
                            
                            elif event.type == "response.failed":
                                # Per avvisare direttamente in chat che c'è stato un errore
                                await cl.Message(content=f"Error: {event.error.message}").send()
                                break

                            # --- CASI DEL TESTO (OUTPUT) ---
                            elif event.type == "response.output_text.delta":
                                if not msg.id: 
                                    await msg.send()
                                full_ai_response += event.delta
                                await msg.stream_token(event.delta)

                            elif event.type == "response.text.done":
                                # Testo terminato, possiamo aggiornare lo stato finale
                                await msg.update()

                            # CASI DEI TOOL (FUNZIONI ROS)
                            elif event.type == "response.output_item.added":
                                # Viene aggiunto un nuovo item (testo o funzione)
                                if event.item.type == "function_call":
                                    idx = event.output_index
                                    tool_name = event.item.name 
                                    
                                    # Creiamo uno step per la visualizzazione
                                    step = cl.Step(name=tool_name, type="tool")
                                    step.language = "json" # Se vuoi mostrare i parametri
                                    await step.send()

                                    final_tool_calls[idx] = event.item # passiamo l'intero item 
                                    active_steps[idx] = step

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
                                tool_call = final_tool_calls[idx]
                                step = active_steps[idx] # Recuperiamo lo step creato in .added
                                args = event.arguments # event.arguments è la stringa completa
                                
                                # Esecuzione ROS effettiva
                                result = await cl.make_async(self.execute_ros_command)(tool_call.name, json.loads(args))

                                # Chiude lo step legato all'output
                                active_steps[idx].output = result
                                await active_steps[idx].update()


                                # Aggiungi la chiamata "completata"
                                history.append({
                                    "type": "function_call",
                                    "call_id": tool_call.call_id,
                                    "name": tool_name,
                                    "arguments": args
                                })

                                log = f"name - {tool_name} \n\targuments - {args}"
                                self.log_message_in_history_file(log=log, authority="function_call")

                                # Aggiungi l'output di ROS
                                history.append({
                                    "type": "function_call_output",
                                    "call_id": tool_call.call_id,
                                    "output": str(result)
                                })

                                log = str(result)
                                self.log_message_in_history_file(log=log, authority="ROS_output")

                                # Aggiunta ai tool chiamati in modo che sia possibile verificare la presenza di tool
                                tool_outputs.append({
                                    "type": "function_call_output",
                                    "call_id": tool_call.call_id,
                                    "output": json.dumps({
                                            "system_state": result
                                        })
                                })

                                # Se abbiamo eseguito un tool, dobbiamo fare un altro giro di loop in modo che LLM riceva l'output del tool
                                requires_action = True

                                

                            # CASI DI RAGIONAMENTO (REASONING)
                            elif event.type == "response.reasoning.delta":
                                # Se usi modelli o1/o3, qui arriva il "pensiero" del modello.
                                # Puoi scegliere se mostrarlo o ignorarlo.
                                pass

                            elif event.type == "response.completed":
                                if msg.id:
                                    await msg.update()
                                break # Esci dal ciclo per sicurezza
                    
                except Exception as e:
                    rospy.logerr(f"Error in the LLM response generation: \n {e}")
                

                if full_ai_response:
                        history.append({"role": "assistant", "content": full_ai_response})
                        self.log_message_in_history_file(log=full_ai_response, authority="assistant")
            
            
            cl.user_session.set("history", history)


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
        with open(f"{script_dir}/../config/{file_config}", "r") as f:
            info = json.load(f)

        areas_dict = info["areas"]
    
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
                rospy.loginfo("Service added to queue")
                return triggerGptResponse("Success")
            except Exception as e:
                return triggerGptResponse(f"Error: {str(e)}")
        return triggerGptResponse("Failed")
    

    async def async_process_queue(self):
            
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

                            await cl.Message(
                                author="Data Mediator",
                                elements=[
                                    cl.Text(
                                        name="🚨 **DATA MEDIATOR**", 
                                        content=f"\tActivated sensors: {active_sensors}", 
                                        display="inline"
                                    )
                                ],
                                type="status" # Lo rende visivamente distinto
                            ).send()

                            # Messaggio da inviare al'LLM
                            message = f"Environmental alert detected. Current environment state: {json.dumps(alert_info)}. Propose a strategy to inspect the environment."


                        elif item_type == "ROBOT_ACTION":
                            # La task nella coda è il processing di un' action ROS
                            goal = item.get("content")

                            # Messaggio da inviare al'LLM
                            message = goal.message_for_LLM


                        # Chiamata al modello per rispondere
                        start_response_time = time.time()
                        await self.handle_message(msg=message, authority="system")
                        end_response_time = time.time()

                        response_time = round(end_response_time - start_response_time, 4)
                        
                    except Exception as e:
                        rospy.logerr(f"Error in handling the queue: \n {e}")
            
                await asyncio.sleep(0.1)
        
    def reset_sensors_activation(self):
        response = self.resetSensorsActivationClient()
        return f"All sensors deactivated"


    def send_robots_to_area(self, robots_sequence):

        robot_deployment_correctness = {}
        
        for robot_to_deploy in robots_sequence:
            robot = robot_to_deploy.get("robot_to_send")
            area = robot_to_deploy.get("area_to_reach")

            if robot in self.robots_list:    
                if area in self.areas_list:

                
                    topic = self.robots_dict[robot]["ros_topic"]
                    area_x = self.areas_dict[area]["coordinates"]["x"]
                    area_y = self.areas_dict[area]["coordinates"]["y"]

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
    

    def control_actuator(self, actuators_sequence):
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
                        "additional_info" : f"Wrong ros service name considered. Check again in the {file_config} file"
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

    
    def display_cameras(self, cameras_names_list):        
        """Function to open camera feed"""
        camera_names_correctness = {}
        topics_list = []

        for camera_name in cameras_names_list:

            camera_name_lower = camera_name.lower()

            if camera_name_lower in self.cameras_list:
                topics_list.append(self.sensors_dict[camera_name_lower]["ros_topic"])
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
        
        subprocess.Popen(["python3", f"{script_dir}/display_camera.py"] + topics_list)

        return f"Summary of cameras names correctness: {json.dumps(camera_names_correctness)}"
            
        


agent = RobotAssistant()

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
            id="Model",
            label="OpenAI - Modello",
            values=["gpt-4o", "gpt-4-turbo", "gpt-5"], 
            initial_index=0,
        )
    ]).send()
    history = []
    history.append({"role": "system", "content": str(prompt)})
    cl.user_session.set("history", history)
    cl.user_session.set("model", settings["Model"])
    cl.user_session.set("agent", agent)

    # Per gestire gli alert da parte del data mediator creiamo una coda in cui il servizio ros
    # va ad inserire la richiesta da processare da parte dell'alert. In seguito dentro il 
    # servizio ros si processa la coda tramite la funzine "async_process_queue"
    asyncio.create_task(agent.async_process_queue())
    

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
    cl.user_session.set("model", settings["Model"])

@cl.on_message
async def main(message: cl.Message):
    """Eseguito ogni volta che l'utente invia un messaggio"""
    
    agent = cl.user_session.get("agent")
    await agent.handle_message(msg=message.content, authority="user")