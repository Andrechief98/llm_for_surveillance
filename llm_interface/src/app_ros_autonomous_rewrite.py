#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, session
from openai import OpenAI
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
import json


script_dir = os.path.dirname(__file__)

session = {
    "messages": []
}

class ChatNodeRewrite():

    def __init__(self):
        rospy.init_node('ChatNodeRewrite', anonymous=True)
        self.rosPublisher = rospy.Publisher('assistant_message', String, queue_size=10)

        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleAlert)
        self.robotGoalCheckerServer = actionlib.SimpleActionServer('/robot_goal_checker', goalCheckerLLMAction, self.handleRobotGoalChecker, auto_start=False)
        self.robotGoalCheckerServer.start()

        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)

        self.client = OpenAI()
        self.model_to_use = "gpt-4o"
        self.mode = rospy.get_param('~mode', 'original')

        # Load config based on mode
        if self.mode == 'original':
            cfg = "info.json"
        elif self.mode == 'j1':
            cfg = "infoj1.json"
        elif self.mode == 'j2':
            cfg = "infoj2.json"
        elif self.mode == 'j3':
            cfg = "infoj3.json"
        else:
            rospy.logwarn(f"Unknown mode '{self.mode}', defaulting to original config")
            self.mode = 'original'
            cfg = "info.json"

        self.config_path = f"{script_dir}/../config/{cfg}"

    
        with open(f"{script_dir}/../config/prompts.yaml") as f:
            prompts_dict = yaml.load(f, Loader=yaml.SafeLoader)
        self.task_instructions = prompts_dict["autonomous"]
        
        rospy.loginfo(f"ChatNodeRewrite initialized with mode: {self.mode}, using {cfg}")

    def responses_api_call(self, user_input, tool_handlers):
        """
        Use Responses API to process user input with tool calling support.
        Returns the final response from the LLM.
        """
        try:
            
            full_input = f"{self.task_instructions}\n\nUser input: {user_input}"
            
            resp = self.client.responses.create(
                model=self.model_to_use,
                input=full_input
            )

            if resp.status == "completed":
                return resp.output_text if hasattr(resp, 'output_text') else str(resp.output)

            elif resp.status == "requires_action":
                tool_calls = resp.required_action.submit_tool_outputs.tool_calls
                tool_outputs = []

                for call in tool_calls:
                    tool_name = call.function.name if hasattr(call, 'function') else call.name
                    args = json.loads(call.function.arguments if hasattr(call, 'function') else call.arguments)
                    handler = tool_handlers.get(tool_name)

                    if handler:
                        try:
                            result = handler(args)
                        except Exception as e:
                            result = f"EXCEPTION: {e}"
                    else:
                        result = f"NO_HANDLER_FOR_{tool_name}"

                    tool_outputs.append({"tool_call_id": call.id, "output": result})

                final_resp = self.client.responses.submit_tool_outputs(
                    run_id=resp.id,
                    tool_outputs=tool_outputs
                )
                return final_resp.output_text if hasattr(final_resp, 'output_text') else str(final_resp.output)

            else:
                return f"Error: Unknown run status {resp.status}"

        except Exception as e:
            rospy.logerr(f"Error in responses_api_call: {e}")
            return f"Error: {e}"

    def handleAlert(self, req):
        """Handle alert from data mediator using Responses API."""
        if req:
            try:
                request_info = json.loads(req.alert_info)
                
                with open(self.config_path, "r") as f:
                    info = json.load(f)

                areas_dict = info["areas"]
                
                del request_info["robots_list_names"]
                del request_info["sensors_list_names"]
                del request_info["actuators_list_names"]

                for robot_name in request_info["robots"].keys():
                    x, y, z = request_info["robots"][robot_name]["current_position"]
                    for area_name, area_info in areas_dict.items():
                        x_range = area_info["coordinate_ranges"]["x"]
                        y_range = area_info["coordinate_ranges"]["y"]
                        if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                            request_info["robots"][robot_name]["current_area"] = area_name
                            break
                    del request_info["robots"][robot_name]["current_orientation"]
                
                del request_info["sensors"]

                for door_name in request_info["actuators"].keys():
                    del request_info["actuators"][door_name]["position"]

                session["messages"].append({"role": "user", "content": json.dumps(request_info)})
                active_sensors = request_info["activated_sensors"]
                socketio.emit('new_message', {"role": "data_mediator", "content": f"ALERT: \nActivated sensors: {active_sensors}"})

                # Define tool handlers
                tool_handlers = {
                    "send_robots_to_area": self.send_robots_to_area,
                    "control_actuator": self.control_actuator,
                    "display_cameras": self.display_cameras,
                    "reset_sensors_activation": self.reset_sensors_activation,
                    "retrieve_system_state": self.retrieve_system_state
                }

                alert_message = json.dumps(request_info)
                output = self.responses_api_call(alert_message, tool_handlers)

                session["messages"].append({"role": "assistant", "content": output})
                socketio.emit('new_message', {"role": "assistant", "content": output})

                return triggerGptResponse("Success")
            except Exception as e:
                rospy.logerr(f"Error in handleAlert: {e}")
                return triggerGptResponse("Failed")
        else:
            return triggerGptResponse("Failed")

    def handleRobotGoalChecker(self, goal):
        """Handle robot goal report using Responses API."""
        if goal:
            try:
                feedback = goalCheckerLLMFeedback()
                result = goalCheckerLLMResult()

                session["messages"].append({"role": "user", "content": goal.message_for_LLM})

                feedback.status_LLM = "Sending robot goal report to LLM..."
                self.robotGoalCheckerServer.publish_feedback(feedback)

                tool_handlers = {
                    "send_robots_to_area": self.send_robots_to_area,
                    "control_actuator": self.control_actuator,
                    "display_cameras": self.display_cameras,
                    "reset_sensors_activation": self.reset_sensors_activation,
                    "retrieve_system_state": self.retrieve_system_state
                }

                output = self.responses_api_call(goal.message_for_LLM, tool_handlers)

                session["messages"].append({"role": "assistant", "content": output})
                socketio.emit('new_message', {"role": "assistant", "content": output})

                result.success = output
                self.robotGoalCheckerServer.set_succeeded(result)
                return

            except Exception as e:
                rospy.logerr(f"Error in handleRobotGoalChecker: {e}")
                result.success = f"Error: {e}"
                self.robotGoalCheckerServer.set_succeeded(result)
        else:
            result = goalCheckerLLMResult()
            result.success = "No message received from robot"
            self.robotGoalCheckerServer.set_succeeded(result)

    def reset_sensors_activation(self, args=None):
        """Reset all sensor activations."""
        try:
            self.resetSensorsActivationClient()
            return "All sensors deactivated"
        except Exception as e:
            return f"Error resetting sensors: {e}"

    def send_robots_to_area(self, robots_sequence):
        """Send robots to specified areas."""
        with open(self.config_path, "r") as f:
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

                    match = re.search(r"_(\d+)$", robot)
                    number = str(int(match.group(1)))
                    service_name = "turtlebot3_" + str(number) + "/move_base/clear_costmaps"
                    
                    try:
                        clear_costmap_client = rospy.ServiceProxy(service_name, Empty)
                        clear_costmap_client()
                    except Exception as e:
                        rospy.logwarn(f"Could not clear costmaps: {e}")

                    temp_pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
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
                    goal_msg.header.frame_id = f"{robot}/map"

                    temp_pub.publish(goal_msg)
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

    def display_cameras(self, args):
        """Display camera feeds."""
        cameras_names_list = args.get("cameras_names_list", [])
        
        with open(self.config_path, "r") as f:
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

        subprocess.Popen(["python3", f"{script_dir}/display_camera.py"] + topics_list)

        return f"Summary of cameras names correctness: {json.dumps(camera_names_correctness)}"

    def control_actuator(self, args):
        """Control door actuators."""
        actuators_sequence = args.get("actuators_sequence", [])
        
        master = rosgraph.Master('/rospy')
        services_info = master.getSystemState()[2]
        ros_service_names_list = [service_info[0] for service_info in services_info]

        responses_list = []

        for actuator in actuators_sequence:
            ros_service_name = actuator.get("ros_service_name")
            command = actuator.get("command")

            if ros_service_name in ros_service_names_list:
                try:
                    rospy.wait_for_service(ros_service_name, timeout=5)
                except rospy.ROSException as e:
                    responses_list.append({
                        "ros_service_name": ros_service_name,
                        "success": "false",
                        "additional_info": str(e)
                    })
                    continue

                try:
                    service_client = rospy.ServiceProxy(ros_service_name, doorStringCommand)
                    req = doorStringCommandRequest()
                    req.command = command
                    resp = service_client(req)

                    responses_list.append({
                        "ros_service_name": ros_service_name,
                        "success": "true",
                        "additional_info": ""
                    })
                except rospy.ServiceException as e:
                    responses_list.append({
                        "ros_service_name": ros_service_name,
                        "success": "false",
                        "additional_info": str(e)
                    })
            else:
                responses_list.append({
                    "ros_service_name": ros_service_name,
                    "success": "false",
                    "additional_info": "Wrong ros service name considered. Check again in the config file"
                })

        return str(responses_list)

    def retrieve_system_state(self, args=None):
        """Retrieve current system state."""
        try:
            response = self.retrieveSystemStateClient()
            system_state = json.loads(response.system_state)

            with open(self.config_path, "r") as f:
                info = json.load(f)

            areas_dict = info["areas"]

            del system_state["robots_list_names"]
            del system_state["sensors_list_names"]
            del system_state["actuators_list_names"]

            for robot_name in system_state["robots"].keys():
                x, y, z = system_state["robots"][robot_name]["current_position"]
                for area_name, area_info in areas_dict.items():
                    x_range = area_info["coordinate_ranges"]["x"]
                    y_range = area_info["coordinate_ranges"]["y"]
                    if x_range["min"] <= x <= x_range["max"] and y_range["min"] <= y <= y_range["max"]:
                        system_state["robots"][robot_name]["current_area"] = area_name
                        break
                del system_state["robots"][robot_name]["current_orientation"]

            del system_state["sensors"]

            for door_name in system_state["actuators"].keys():
                del system_state["actuators"][door_name]["position"]

            return json.dumps(system_state)
        except Exception as e:
            return f"Error retrieving system state: {e}"


chatNodeRewrite = ChatNodeRewrite()

app = Flask(__name__)
app.secret_key = "supersegreta"

socketio = SocketIO(app)

DEFAULT_MESSAGE = chatNodeRewrite.task_instructions
DEFAULT_IMAGE = "static/uploads/building_plan.png"


@app.route("/")
def index():
    return render_template("index.html", default_message=DEFAULT_MESSAGE, default_image=DEFAULT_IMAGE)


@app.route("/chat", methods=["POST"])
def chat():
    data = request.json
    user_message = data.get("message", "").strip()

    print("User:")
    print(f"{user_message}")

    if not user_message:
        return jsonify({"error": "Messaggio vuoto"}), 400

    session["messages"].append({"role": "user", "content": user_message})

    try:
        tool_handlers = {
            "send_robots_to_area": chatNodeRewrite.send_robots_to_area,
            "control_actuator": chatNodeRewrite.control_actuator,
            "display_cameras": chatNodeRewrite.display_cameras,
            "reset_sensors_activation": chatNodeRewrite.reset_sensors_activation,
            "retrieve_system_state": chatNodeRewrite.retrieve_system_state
        }

        start_response_time = time.time()
        output = chatNodeRewrite.responses_api_call(user_message, tool_handlers)
        end_response_time = time.time()

        inference_time = end_response_time - start_response_time
        print(f"Inference time: {inference_time}")
        print("AI:")
        print(output)

        session["messages"].append({"role": "assistant", "content": output})

        response = {
            "reply": output,
        }
        return jsonify(response)

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


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
