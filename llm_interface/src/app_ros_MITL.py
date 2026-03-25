#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, session
from openai import OpenAI
import os
from werkzeug.utils import secure_filename
import rospy
import actionlib.simple_action_server
from utilitiesOpenAI import *
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from llm_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback
from flask_socketio import SocketIO
import json
import time
import logging
import yaml
import subprocess
import re
import rosgraph
from gazebo_plugins.srv import doorStringCommand, doorStringCommandRequest
import actionlib
from utilitiesOpenAI import extractFilesFromJson, updateFilesJsonFile

script_dir = os.path.dirname(__file__)

# Log conversation feedback to the terminal
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# In-memory chat history (simple session)
session = {"messages": []}

# Mode-based config
if rospy.has_param("/markerVisualizationNode/mode"):
    mode = rospy.get_param("/markerVisualizationNode/mode")
else:
    mode = "original"

if mode == "original":
    file_config = "info.json"
    file_prompt = "prompts.yaml"
    file_building_plan = "building_plan.png"
else:
    file_config = "info" + mode + ".json"
    file_prompt = "prompts" + mode + ".yaml"
    file_building_plan = "building_plan" + mode + ".png"

print(file_config)

# Definition of the chat node
class ChatNode:
    def __init__(self):
        rospy.init_node("ChatNode", anonymous=True)
        self.rosPublisher = rospy.Publisher("assistant_message", String, queue_size=10)

        self.rosServer = rospy.Service("/alert", triggerGpt, self.handleAlert)
        self.robotGoalCheckerServer = actionlib.SimpleActionServer(
            "/robot_goal_checker",
            goalCheckerLLMAction,
            self.handleRobotGoalChecker,
            auto_start=False,
        )
        self.robotGoalCheckerServer.start()

        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        self.client = OpenAI()
        self.model_to_use = "gpt-4o"

        # Load task instructions from yaml
        with open(f"{script_dir}/../config/{file_prompt}") as f:
            prompts_dict = yaml.load(f, Loader=yaml.SafeLoader)
        prompt_type = "man_in_the_loop"
        self.task_instructions = prompts_dict[prompt_type]

        # Ensure the info.json file is available in a vector store for file_search.
        file_paths = [os.path.join(script_dir, "..", "config", file_config)]
        file_streams = [open(path, "rb") for path in file_paths]

        required_vector_store_id = None
        files_name_list, files_id_list, vector_store_id_list = extractFilesFromJson(self.client, script_dir)
        if file_config in files_name_list:
            for vector_store_id in vector_store_id_list:
                if not vector_store_id.startswith("vs_"):
                    continue
                for file in self.client.vector_stores.files.list(vector_store_id=vector_store_id):
                    retrieved_file = self.client.files.retrieve(file_id=file.id)
                    if retrieved_file.filename == file_config:
                        required_vector_store_id = vector_store_id
                        break
                if required_vector_store_id:
                    break

        if required_vector_store_id is None:
            vector_store = self.client.vector_stores.create(
                name="Environmental and ROS information for surveillance"
            )
            required_vector_store_id = vector_store.id
            self.client.vector_stores.file_batches.upload_and_poll(
                vector_store_id=required_vector_store_id, files=file_streams
            )
            for file in self.client.vector_stores.files.list(vector_store_id=required_vector_store_id):
                retrieved_file = self.client.files.retrieve(file_id=file.id)
                if retrieved_file.filename == file_config and file_config not in files_name_list:
                    files_name_list.append(file_config)
                    files_id_list.append(file.id)
                    vector_store_id_list.append(required_vector_store_id)
            updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)

        # Always close streams
        for fs in file_streams:
            try:
                fs.close()
            except Exception:
                pass

        if required_vector_store_id is None:
            raise RuntimeError("Unable to determine required_vector_store_id for file_search tool")

        # Define tools for Responses API
        self.tools = [
            {
                "type": "file_search",
                "vector_store_ids": [required_vector_store_id],
            },
            {
                "type": "function",
                "name": "retrieve_system_state",
                "description": "Request the current system state from the ROS data mediator.",
                "parameters": {},
            },
            {
                "type": "function",
                "name": "reset_sensors_activation",
                "description": "Deactivate all sensors in the environment.",
                "parameters": {},
            },
            {
                "type": "function",
                "name": "display_cameras",
                "description": "Open a window to visualize each camera.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "cameras_names_list": {
                            "type": "array",
                            "description": "List of camera names.",
                            "items": {"type": "string"},
                        }
                    },
                    "required": ["cameras_names_list"],
                },
            },
            {
                "type": "function",
                "name": "control_actuator",
                "description": f"Use this function to control a sequence of door actuators in the environment via ROS service calls defined in the {file_config} file.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "actuators_sequence": {
                            "type": "array",
                            "description": f"Ordered list of actuator commands. Each item specifies the ROS service name (from the {file_config} file) and the desired action ('open' or 'close').",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "ros_service_name": {"type": "string"},
                                    "command": {"type": "string", "enum": ["open", "close"]},
                                },
                                "required": ["ros_service_name", "command"],
                            },
                        }
                    },
                    "required": ["actuators_sequence"],
                },
            },
            {
                "type": "function",
                "name": "send_robots_to_area",
                "description": "Deploy a sequence of robots to specified areas.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "robots_sequence": {
                            "type": "array",
                            "description": "Ordered list of robots and target areas.",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "robot_to_send": {"type": "string"},
                                    "area_to_reach": {"type": "string"},
                                },
                                "required": ["robot_to_send", "area_to_reach"],
                            },
                        }
                    },
                    "required": ["robots_sequence"],
                },
            },
        ]

        # Initialize conversation memory
        # Start with a system-style instruction so the model understands its role.
        session["messages"] = [
            {"role": "system", "content": self.task_instructions}
        ]

        # Ensure the building plan image is accessible
        required_files_names_list = [file_building_plan]
        files_name_list, files_id_list, vector_store_id_list = extractFilesFromJson(self.client, script_dir)
        for required_file_name in required_files_names_list:
            if required_file_name in files_name_list:
                required_file_id = files_id_list[files_name_list.index(required_file_name)]
                try:
                    self.img_file = self.client.files.retrieve(required_file_id)
                except Exception as e:
                    # File not found, remove from list and upload anew
                    idx = files_name_list.index(required_file_name)
                    files_name_list.pop(idx)
                    files_id_list.pop(idx)
                    vector_store_id_list.pop(idx)
                    self.img_file = self.client.files.create(
                        file=open(f"{script_dir}/static/uploads/{required_file_name}", "rb"),
                        purpose="vision",
                    )
                    files_name_list.append(required_file_name)
                    files_id_list.append(self.img_file.id)
                    vector_store_id_list.append("/")
                    updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)
            else:
                self.img_file = self.client.files.create(
                    file=open(f"{script_dir}/static/uploads/{required_file_name}", "rb"),
                    purpose="vision",
                )
                required_file_id = self.img_file.id
                files_name_list.append(required_file_name)
                files_id_list.append(required_file_id)
                vector_store_id_list.append("/")
                updateFilesJsonFile(files_name_list, files_id_list, vector_store_id_list, script_dir)

        print("ChatNode initialized")

    def _build_response_input(self, user_message: str) -> list:
        """Builds the input conversation state for the Responses API."""
        session["messages"].append({"role": "user", "content": user_message})
        logging.info(f"[Chat] User: {user_message}")
        input_messages = []
        for msg in session["messages"]:
            role = msg.get("role", "user")
            content_type = "input_text"
            if role == "assistant":
                content_type = "output_text"
            input_messages.append(
                {
                    "type": "message",
                    "role": role,
                    "content": [{"type": content_type, "text": msg.get("content", "")}],
                }
            )
        return input_messages

    def _extract_assistant_response(self, response):
        if hasattr(response, "output_text") and response.output_text:
            return response.output_text
        if hasattr(response, "output") and response.output:
            for item in response.output:
                if getattr(item, "type", None) == "message":
                    content_list = getattr(item, "content", [])
                    for c in content_list:
                        if getattr(c, "type", None) == "output_text":
                            return getattr(c, "text", "")
        return ""

    def _extract_tool_calls(self, response):
        """Return list of tool call dicts from a Responses API response."""
        tool_calls = []
        if not hasattr(response, "output") or not response.output:
            return tool_calls

        for item in response.output:
            if getattr(item, "type", None) == "function_call":
                tool_calls.append({
                    "name": getattr(item, "name", None),
                    "arguments": getattr(item, "arguments", None),
                    "call_id": getattr(item, "call_id", None),
                })
        return tool_calls

    def _execute_tool_call(self, call):
        """Execute a local tool implementation and append its result to history."""
        name = call.get("name")
        arguments = call.get("arguments")
        logging.info(f"[Tool] Calling: {name} with args: {arguments}")
        tool_fn = getattr(self, name, None)
        if tool_fn is None:
            output = f"Tool '{name}' not implemented."
        else:
            try:
                args = {}
                if arguments:
                    args = json.loads(arguments)
                result = tool_fn(**args) if isinstance(args, dict) else tool_fn(args)
                output = json.dumps(result) if not isinstance(result, str) else result
            except Exception as e:
                output = f"Tool '{name}' execution error: {e}"

        logging.info(f"[Tool] {name} output: {output}")

        # Add tool output back into the session so subsequent responses can use it
        session["messages"].append({
            "role": "system",
            "content": f"Tool {name} output: {output}",
        })
        return output

    def chat(self, user_message: str) -> str:
        # Keep looping if model decides to call a tool; we re-run after executing it.
        for _ in range(3):
            response = self.client.responses.create(
                model=self.model_to_use,
                input=self._build_response_input(user_message),
                instructions=self.task_instructions,
                tools=self.tools,
                tool_choice="auto",
            )

            tool_calls = self._extract_tool_calls(response)
            if tool_calls:
                for call in tool_calls:
                    self._execute_tool_call(call)
                # After running tools, re-run the model to get the final assistant response.
                continue

            assistant_reply = self._extract_assistant_response(response)
            if assistant_reply:
                session["messages"].append({"role": "assistant", "content": assistant_reply})
                logging.info(f"[Chat] Assistant: {assistant_reply}")
                return assistant_reply

        # If the code reached here, it didn't get a final assistant text reply.
        logging.warning("[Chat] No assistant response received after tool calls")
        return ""

    def handleRobotGoalChecker(self, goal):
        result = goalCheckerLLMResult()
        if not goal or not getattr(goal, "message_for_LLM", None):
            result.success = "Function handleRobotGoalChecker: No message received from robot"
            self.robotGoalCheckerServer.set_succeeded(result)
            return

        assistant_reply = self.chat(goal.message_for_LLM)
        socketio.emit("new_message", {"role": "assistant", "content": assistant_reply})

        result.success = "Message from robot correctly processed by the LLM"
        self.robotGoalCheckerServer.set_succeeded(result)

    def handleAlert(self, req):
        if not req or not getattr(req, "alert_info", None):
            return triggerGptResponse("Failed")

        try:
            request_info = json.loads(req.alert_info)
        except Exception:
            return triggerGptResponse("Failed")

        with open(f"{script_dir}/../config/{file_config}", "r") as f:
            info = json.load(f)

        areas_dict = info.get("areas", {})
        request_info.pop("robots_list_names", None)
        request_info.pop("sensors_list_names", None)
        request_info.pop("actuators_list_names", None)

        for robot_name, robot_info in request_info.get("robots", {}).items():
            x, y, _ = robot_info.get("current_position", (0, 0, 0))
            for area_name, area_info in areas_dict.items():
                x_range = area_info["coordinate_ranges"]["x"]
                y_range = area_info["coordinate_ranges"]["y"]
                if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                    request_info["robots"][robot_name]["current_area"] = area_name
                    break
            request_info["robots"][robot_name].pop("current_orientation", None)

        request_info.pop("sensors", None)
        for door_info in request_info.get("actuators", {}).values():
            door_info.pop("position", None)

        assistant_reply = self.chat(json.dumps(request_info))
        socketio.emit("new_message", {"role": "assistant", "content": assistant_reply})
        return triggerGptResponse("Success")

    def reset_sensors_activation(self):
        self.resetSensorsActivationClient()
        return "All sensors deactivated"

    def retrieve_system_state(self):
        response = self.retrieveSystemStateClient()
        system_state = json.loads(response.system_state)
        with open(f"{script_dir}/../config/{file_config}", "r") as f:
            info = json.load(f)
        areas_dict = info.get("areas", {})

        system_state.pop("robots_list_names", None)
        system_state.pop("sensors_list_names", None)
        system_state.pop("actuators_list_names", None)

        for robot_name, robot_data in system_state.get("robots", {}).items():
            x, y, z = robot_data.get("current_position", (0, 0, 0))
            for area_name, area_info in areas_dict.items():
                x_range = area_info["coordinate_ranges"]["x"]
                y_range = area_info["coordinate_ranges"]["y"]
                if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                    system_state["robots"][robot_name]["current_area"] = area_name
                    break
            system_state["robots"][robot_name].pop("current_orientation", None)

        system_state.pop("sensors", None)
        for door_name, door_info in system_state.get("actuators", {}).items():
            door_info.pop("position", None)

        return json.dumps(system_state)

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
                    goal_msg.header.frame_id = "map"

                    # Publish the goal message
                    temp_pub.publish(goal_msg)

                    # Wait to transmit the message
                    rospy.sleep(0.5)

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
                    response = {
                        "ros_service_name": ros_service_name,
                        "success": "false",
                        "additional_info": str(e)
                    }
                    responses_list.append(response)
                    continue

                try:
                    service_client = rospy.ServiceProxy(ros_service_name, doorStringCommand)

                    req = doorStringCommandRequest()
                    req.command = command

                    resp = service_client(req)

                    response = {
                        "ros_service_name": ros_service_name,
                        "success": "true",
                        "additional_info": ""
                    }
                    responses_list.append(response)

                except rospy.ServiceException as e:
                    response = {
                        "ros_service_name": ros_service_name,
                        "success": "false",
                        "additional_info": str(e)
                    }
                    responses_list.append(response)
            else:
                response = {
                    "ros_service_name": ros_service_name,
                    "success": "false",
                    "additional_info": f"Wrong ros service name considered. Check again in the {file_config} file"
                }
                responses_list.append(response)

        return str(responses_list)


chatNode = ChatNode()

app = Flask(__name__)
app.secret_key = "supersegreta"

socketio = SocketIO(app)

DEFAULT_MESSAGE = chatNode.task_instructions
DEFAULT_IMAGE = f"static/uploads/{file_building_plan}"


@app.route("/")
def index():
    return render_template("index.html", default_message=DEFAULT_MESSAGE, default_image=DEFAULT_IMAGE)


@app.route("/chat", methods=["POST"])
def chat():
    data = request.json or {}
    user_message = (data.get("message") or "").strip()
    if not user_message:
        return jsonify({"error": "Messaggio vuoto"}), 400

    print(f"User message: {user_message}")

    try:
        assistant_reply = chatNode.chat(user_message)
        if not assistant_reply:
            return jsonify({"error": "Nessuna risposta dall'LLM (controlla i log)."}), 500
        print(f"LLM response: {assistant_reply}")
        return jsonify({"reply": assistant_reply})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/upload", methods=["POST"])
def upload_image():
    if "image" not in request.files:
        return jsonify({"error": "Nessun file caricato"}), 400
    file = request.files["image"]
    if file.filename == "":
        return jsonify({"error": "Nome file non valido"}), 400
    filename = secure_filename(file.filename)
    file_path = os.path.join(script_dir, "static", "uploads", filename)
    file.save(file_path)
    return jsonify({"image_url": f"{script_dir}/static/uploads/{filename}"})


@app.route("/reset", methods=["POST"])
def reset_chat():
    session.pop("messages", None)
    return jsonify({"message": "Chat resettata!"})


if __name__ == "__main__":
    app.run(debug=False)
