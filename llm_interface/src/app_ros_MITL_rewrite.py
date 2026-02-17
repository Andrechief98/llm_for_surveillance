#!/usr/bin/env python3
"""
app_ros_MITL_rewrite.py

This is a rewrite of app_ros_MITL.py using the OpenAI Responses API pattern.
"""

from flask import Flask, render_template, request, jsonify, session
from openai import OpenAI
import os
from werkzeug.utils import secure_filename
import rospy
from std_msgs.msg import String 
from std_srvs.srv import Empty 
from geometry_msgs.msg import PoseStamped
from llm_interface.srv import triggerGpt, triggerGptResponse, retrieveSystemState
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback
from flask_socketio import SocketIO, emit
import subprocess
import rosgraph
import re
import time
import actionlib
import yaml
import json

script_dir = os.path.dirname(__file__)

session = {"messages": []}

class ChatNodeRewrite():
    def __init__(self):
        rospy.init_node('ChatNodeRewrite', anonymous=True)
        self.rosPublisher = rospy.Publisher('assistant_message', String, queue_size=10)
        self.rosServer = rospy.Service('/alert', triggerGpt, self.handleAlert)
        self.robotGoalCheckerServer = actionlib.SimpleActionServer('/robot_goal_checker', goalCheckerLLMAction, self.handleRobotGoalChecker, auto_start = False)
        self.robotGoalCheckerServer.start()
        self.retrieveSystemStateClient = rospy.ServiceProxy("/retrieve_system_state", retrieveSystemState)
        self.resetSensorsActivationClient = rospy.ServiceProxy("/resetSensorActivation", Empty)
        self.client = OpenAI()  # Replace with your OpenAI() client
        self.model_to_use = "gpt-4o"
        # ...existing code for assistant and thread setup...
        # For brevity, you can copy your assistant/thread setup from the original file
        # and adapt as needed for the Responses API.

    def responses_api_run(self, user_input, tool_handlers):
        resp = self.client.responses.create(model=self.model_to_use, input=user_input)
        if resp.status == "completed":
            return resp.output_text
        elif resp.status == "requires_action":
            tool_calls = resp.required_action.submit_tool_outputs.tool_calls
            tool_outputs = []
            for call in tool_calls:
                tool_name = call.name
                args = json.loads(call.arguments)
                handler = tool_handlers.get(tool_name)
                if handler:
                    try:
                        result = handler(args)
                    except Exception as e:
                        result = f"EXCEPTION: {e}"
                else:
                    result = f"NO_HANDLER_FOR_{tool_name}"
                tool_outputs.append({"tool_call_id": call.id, "output": result})
            final_resp = self.client.responses.submit_tool_outputs(run_id=resp.id, tool_outputs=tool_outputs)
            return final_resp.output_text
        else:
            return "Error: Unknown run status"

    # Tool handler for sending robots to area
    def send_robots_to_area(self, args):
        area = args.get('area', 'unknown')
        robot = args.get('robot', 'turtlebot3_1')
        # Example: publish a goal to the robot's topic
        topic = self.get_robot_topic(robot)
        if topic:
            pose = PoseStamped()
            pose.pose.position.x = args.get('x', 0)
            pose.pose.position.y = args.get('y', 0)
            pub = rospy.Publisher(topic, PoseStamped, queue_size=1)
            pub.publish(pose)
            self.rosPublisher.publish(String(f"Sent {robot} to area {area} at ({pose.pose.position.x}, {pose.pose.position.y})"))
            return f"Robot {robot} sent to area {area}"
        return f"Failed to send {robot} to area {area}"

    # Tool handler for actuator control
    def control_actuator(self, args):
        actuator = args.get('actuator', 'door_1')
        action = args.get('action', 'open')
        service_name = self.get_actuator_service(actuator)
        if service_name:
            try:
                rospy.wait_for_service(service_name, timeout=2)
                srv = rospy.ServiceProxy(service_name, Empty)
                srv()
                self.rosPublisher.publish(String(f"Actuator {actuator} {action}ed"))
                return f"Actuator {actuator} {action}ed"
            except Exception as e:
                return f"Actuator {actuator} failed: {e}"
        return f"Actuator {actuator} service not found"

    # Tool handler for robot status
    def robot_status(self, args):
        robot = args.get('robot', 'turtlebot3_1')
        # Example: return dummy status
        return f"Status of {robot}: OK"

    # Helper to get robot topic
    def get_robot_topic(self, robot):
        # Example: hardcoded topics, replace with config lookup
        topics = {
            'turtlebot3_1': '/turtlebot3_1/move_base_simple/goal',
            'turtlebot3_2': '/turtlebot3_2/move_base_simple/goal'
        }
        return topics.get(robot)

    # Helper to get actuator service
    def get_actuator_service(self, actuator):
        services = {
            'door_1': '/door_1_control_service',
            'door_2': '/door_2_control_service',
            'door_3': '/door_3_control_service',
            'door_4': '/door_4_control_service',
            'door_5': '/door_5_control_service',
            'door_6': '/door_6_control_service',
            'door_7': '/door_7_control_service',
            'door_8': '/door_8_control_service',
            'door_9': '/door_9_control_service'
        }
        return services.get(actuator)

    def handleAlert(self, req):
        # Example: use responses_api_run for alert handling
        tool_handlers = {
            "send_robots_to_area": self.send_robots_to_area,
            "control_actuator": self.control_actuator,
            # ...add other handlers...
        }
        user_input = "Alert received: " + str(req.alert_info)
        output = self.responses_api_run(user_input, tool_handlers)
        return triggerGptResponse(output)

    def handleRobotGoalChecker(self, goal):
        tool_handlers = {
            "send_robots_to_area": self.send_robots_to_area,
            "control_actuator": self.control_actuator,
            "robot_status": self.robot_status
        }
        user_input = goal.message_for_LLM
        output = self.responses_api_run(user_input, tool_handlers)
        result = goalCheckerLLMResult()
        result.success = output
        self.robotGoalCheckerServer.set_succeeded(result)
        # Emit feedback via SocketIO and ROS topic
        try:
            from flask_socketio import emit
            emit('robot_goal_feedback', {'feedback': output}, broadcast=True)
        except Exception:
            pass
        self.rosPublisher.publish(String(f"RobotGoalChecker result: {output}"))

# ...existing code for Flask app, endpoints, and SocketIO setup...
# You can copy your Flask app setup and endpoints from the original file,
# and in the /chat endpoint, use chatNodeRewrite.responses_api_run instead of the old run/submit loop.

app = Flask(__name__)
socketio = SocketIO(app)

# Instantiate ChatNodeRewrite
chatNodeRewrite = ChatNodeRewrite()


# SocketIO event for real-time chat
@socketio.on('chat_message')
def handle_chat_message(data):
    user_input = data.get('user_input', '')
    tool_handlers = {
        "send_robots_to_area": chatNodeRewrite.send_robots_to_area,
        "control_actuator": chatNodeRewrite.control_actuator,
        # ...add other handlers as needed...
    }
    output = chatNodeRewrite.responses_api_run(user_input, tool_handlers)
    emit('chat_response', {'response': output})

# Example /chat endpoint using Responses API (still available for HTTP POST)
@app.route("/chat", methods=["POST"])
def chat():
    data = request.get_json()
    user_input = data.get("user_input", "")
    tool_handlers = {
        "send_robots_to_area": chatNodeRewrite.send_robots_to_area,
        "control_actuator": chatNodeRewrite.control_actuator,
        # ...add other handlers as needed...
    }
    output = chatNodeRewrite.responses_api_run(user_input, tool_handlers)
    return jsonify({"response": output})

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
