import chainlit as cl
from chainlit.input_widget import Select
from openai import AsyncOpenAI
import rospy
import rosgraph
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from geometry_msgs.msg import PoseStamped
from llm_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback
from gazebo_plugins.srv import doorStringCommand, doorStringCommandRequest 
import actionlib
import yaml
import time
import json
import os
import asyncio
from queue import Queue



script_dir = os.path.dirname(__file__)


class RobotAssistant:
    def __init__(self):
        rospy.init_node('RobotAssistant', anonymous=True)
        self.client = AsyncOpenAI()

        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleAlert)

        # self.robotGoalCheckerServer = actionlib.SimpleActionServer('/robot_goal_checker', goalCheckerLLMAction, self.handleRobotGoalChecker, auto_start = False)
        # self.robotGoalCheckerServer.start()

        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        # Definiamo una coda, necessaria per gestire gli alert da parte del datamediator (in quanto i servizi ros non sono )
        self.alert_queue = Queue()
    
    async def handle_user_message(self, user_input):

        # Riprendiamo lo storico della conversazione a cui aggiungeremo il nuovo messaggio
        history = cl.user_session.get("history")
        history.append({"role": "user", "content": user_input})

        # Creiamo un messaggio vuoto che "riempiremo" con lo streaming
        msg = cl.Message(content="")
        
        stream = await self.client.responses.create(
            model="gpt-4", # O gpt-4o
            input=history,
            tools = [
                {
                    "type": "function",  # Specifica sempre il tipo
                    "name": "send_robots_to_area",  # <--- Il parametro mancante era qui
                    "description": "Invia una sequenza di robot in un'area specifica",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "robots_sequence": {
                                "type": "string",
                                "description": "Lista dei robot e destinazioni"
                            }
                        },
                        "required": ["robots_sequence"]
                    }
                },
                {
                    "type": "function",  # Specifica sempre il tipo
                    "name": "retrieve_system_state",  # <--- Il parametro mancante era qui
                    "description": "Fornisce lo stato del sistema",
                    "parameters": {}
                },
                {
                    "type": "function",
                    "name": "control_actuator",
                    "description": "It allows to control a sequence of door actuators in the environment via ROS service calls defined in the info.json file.",
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
                },
            ],
            stream=True
        )


        full_ai_response = ""
        final_tool_calls = {}
        active_steps = {} # Teniamo traccia degli step attivi per indice
        tool_outputs = []

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
                    
                    # Creiamo uno step professionale
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

                # Aggiungi l'output di ROS
                history.append({
                    "type": "function_call_output",
                    "call_id": tool_call.call_id,
                    "output": str(result)
                })


                # Aggiunta ai tool chiamati in modo che sia possibile verificare la presenza di tool
                tool_outputs.append({
                    "type": "function_call_output",
                    "call_id": tool_call.call_id,
                    "output": json.dumps({
                            "system_state": result
                        })
                })

                

            # CASI DI RAGIONAMENTO (REASONING)
            elif event.type == "response.reasoning.delta":
                # Se usi modelli o1/o3, qui arriva il "pensiero" del modello.
                # Puoi scegliere se mostrarlo o ignorarlo.
                pass

            elif event.type == "response.completed":
                if msg.id:
                    await msg.update()
                break # Esci dal ciclo per sicurezza
        
        if full_ai_response:
            history.append({"role": "assistant", "content": full_ai_response})
        
        if tool_outputs:
                
            follow_up_stream = await self.client.responses.create(
                model="gpt-4o",
                input=history, # La storia ora include l'esito dell'azione ROS
                tools = [],
                stream=True
            )
    
            # Continuiamo lo streaming nel messaggio originale 'msg'
            tool_function_result_msg = ""
            async for event in follow_up_stream:
                if event.type == "response.created":
                    pass # Ottimo per loggare l'ID della risposta
                
                elif event.type == "response.failed":
                    # Per avvisare direttamente in chat che c'è stato un errore
                    await cl.Message(content=f"Error: {event.error.message}").send()
                    break

                elif event.type == "response.output_text.delta":
                    if not msg.id: 
                        await msg.send()
                    tool_function_result_msg += event.delta
                    await msg.stream_token(event.delta)

                elif event.type == "response.completed":
                    if msg.id:
                        await msg.update()
                    break # Esci dal ciclo per sicurezza
            
            if tool_function_result_msg:
                history.append({"role": "assistant", "content": full_ai_response})
        
        cl.user_session.set("history", history)


    async def monitor_queue(self):
        """
        Metodo asincrono che Chainlit userà per 'ascoltare' la classe.
        """
        while True:
            if not self.alert_queue.empty():
                alert_data = self.alert_queue.get()
                
                # Logica di streaming Chainlit
                msg = cl.Message(content=f"**Notifica ROS:** {alert_data}\n\nGenerazione piano...")
                await msg.send()
                
                # Esempio Streaming LLM
                # Qui chiameresti il tuo modello (es. langchain o openai)
                await self.stream_llm_plan(msg)
                
            # Fondamentale: rilascia il controllo per permettere ad altri task di girare
            await asyncio.sleep(0.1)


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
                
                # 3. Chiamata dinamica al metodo (es. self.send_robots_to_area(**args))
                # L'operatore ** scompatta il dizionario args nei parametri della funzione
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
        with open(f"{script_dir}/../config/info.json", "r") as f:
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

    def handleAlert(self, req):
        # ROS chiama questa funzione in modo sincrono
        if req:        
            try:
                self.alert_queue.put(req)
                return triggerGptResponse("Success")
            except Exception as e:
                return triggerGptResponse(f"Error: {str(e)}")
        return triggerGptResponse("Failed")
    

    async def async_process_alert(self):
            
            while True:
                if not self.alert_queue.empty():
                    try:
                        req = self.alert_queue.get()

                        # Estrai e pulisci il messaggio ricevuto via ros service dal data mediator
                        alert_info = self.clear_environment_state(json.loads(req.alert_info))

                        # Estraiamo i sensori attivi
                        active_sensors = alert_info["activated_sensors"]

                        await cl.Message(
                            content=f"🚨 **SISTEMA DI ALLARME ATTIVATO**\nSensori: {active_sensors}",
                            author="Data Mediator",
                            type="status" # Lo rende visivamente distinto
                        ).send()

                        # Recuperiamo la sessione di messaggi da chainlit
                        history = cl.user_session.get("history")

                        # Inserimento del messaggio ricevuto nella cronologia della chat come se provenisse dall’operatore (OpenAI non prevede messaggi associati a figure diverse da operatore o assistant)
                        prompt_alert = f"Environmental alert detected. Current environment state: {json.dumps(alert_info)}. Propose a strategy to inspect the environment."

                        history.append({"role": "system", "content": prompt_alert})
                        
                        start_response_time = time.time()

                        # Creiamo il messaggio che conterrà la strategia
                        msg = cl.Message(content="", author="Assistant")
                        await msg.send()

                        # Chiamata OpenAI (usando il modello selezionato)
                        stream = await self.client.responses.create(
                            model=cl.user_session.get("model"),
                            input=history,
                            tools=[],
                            stream=True
                        )

                        async for event in stream:
                            if event.type == "response.output_text.delta":
                                await msg.stream_token(event.delta)
                        
                        await msg.update()
                        end_response_time = time.time()

                        response_time = round(end_response_time - start_response_time, 4)

                        # Salviamo la risposta finale in history
                        history.append({"role": "assistant","content": msg.content})
                    except Exception as e:
                        print(e)
            
                await asyncio.sleep(0.1)
        
    def reset_sensors_activation(self):
        response = self.resetSensorsActivationClient()
        return f"All sensors deactivated"

    def send_robots_to_area(self, robots_sequence):
        """Esempio di comando di movimento"""
        # Qui inserisci la tua logica ROS Noetic (es. pubblicazione su un topic o chiamata service)
        # Esempio: self.my_publisher.publish(msg)
        return f"Sequenza '{robots_sequence}' inviata correttamente ai robot."
    
    def control_actuator(self, actuators_sequence):
        """Function to control actuators"""
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




agent = RobotAssistant()

# --- Integrazione Chainlit ---
@cl.on_chat_start
async def start():
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
    cl.user_session.set("history", [])
    cl.user_session.set("model", settings["Model"])
    cl.user_session.set("agent", agent)

    # Per gestire gli alert da parte del data mediator creiamo una coda in cui il servizio ros
    # va ad inserire la richiesta da processare da parte dell'alert. In seguito dentro il 
    # servizio ros si processa la coda tramite la funzine "async_process_alert"
    asyncio.create_task(agent.async_process_alert())
    

@cl.on_settings_update
async def update_settings(settings):
    cl.user_session.set("model", settings["Model"])

@cl.on_message
async def main(message: cl.Message):
    """Eseguito ogni volta che l'utente invia un messaggio"""
    
    agent = cl.user_session.get("agent")
    await agent.handle_user_message(message.content)