#!/usr/bin/env python3

import json, re
import threading
import subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_plugins.srv import doorStringCommand, doorStringCommandRequest
import actionlib
from llm_interface.msg import goalCheckerLLMAction, goalCheckerLLMResult, goalCheckerLLMFeedback



BASE_DIR = Path(__file__).resolve().parent

JSON_DIRECTORY = BASE_DIR / "../../llm_interface/config/ROS"

MAX_CONCURRENT_TASKS = 16
MAX_VIDEO_STREAMS = 6

CAMERA_SCRIPT = BASE_DIR / "../../llm_interface/src/display_camera.py"


# Retrieve ROS parameters
world_id = str(rospy.get_param("world_id", 1))
llm_modality = rospy.get_param("llm_modality", "HITL")
teleport_robots = rospy.get_param("teleport_robots",True)
sensor_sequence = rospy.get_param("sensor_sequence", "").split(",")
# semantic_RAG = rospy.get_param("semantic_RAG", False)
trial_id = rospy.get_param("trial_id", "0")
llm_model = rospy.get_param("llm_model", "")



class SupervisorBackend:

    def __init__(self):
        self.lock = threading.RLock()

        self.executor = ThreadPoolExecutor(
            max_workers=MAX_CONCURRENT_TASKS
        )

        self.tasks = {}
        self.robot_publishers = {}
        self.camera_process = None
        self.camera_topics = []

        # Store the latest verification result for each robot.
        self.robot_goal_events = {}

        # Store the history of robot verification events.
        self.robot_goal_history = []

        self.teleport_robots = teleport_robots
        self.world_id = world_id
        self.config = None

        self.load_world_config()

        # Start the action server after completing its initialization.
        self.robotGoalCheckerServer.start()

    # ========================================================
    # CONFIGURAZIONE DEL MONDO
    # ========================================================

    def load_world_config(self):


        json_path = (
            JSON_DIRECTORY / f"info_map{self.world_id}.json"
        ).resolve()



        with json_path.open("r", encoding="utf-8") as f:
            config = json.load(f)

        
        self.config = config

        self.robots_dict = (config.get("ros_publishers", {}).get("robots", {}))
        self.cameras_dict = (config.get("ros_subscribers", {}).get("sensors", {}))
        self.actuators_dict = (config.get("ros_services", {}).get("actuators", {}))

        self.areas_dict = config.get("areas", {})

        self.robots_list = list(self.robots_dict.keys())
        self.cameras_list = list(self.cameras_dict.keys())
        self.actuators_list = list(self.actuators_dict.keys())
        self.areas_list = list(self.areas_dict.keys())

        self.robot_publishers = {
            robot: rospy.Publisher(entry["ros_topic"],PoseStamped,queue_size=10) for robot, entry in self.robots_dict.items()
        }

        self.robotGoalCheckerServer = actionlib.SimpleActionServer(
            "/robot_goal_checker",
            goalCheckerLLMAction,
            execute_cb=self.handleRobotGoalChecker,
            auto_start=False,
        )

    # ========================================================
    # REGISTRO DELLE OPERAZIONI
    # ========================================================

    def submit_task(self, description, function, *args):
        task_id = (
            datetime.now().strftime("%Y%m%d%H%M%S%f")
        )

        with self.lock:
            self.tasks[task_id] = {
                "description": description,
                "status": "queued",
                "result": None,
                "error": None,
            }

        def run():
            with self.lock:
                self.tasks[task_id]["status"] = "running"

            try:
                result = function(*args)

                with self.lock:
                    self.tasks[task_id]["result"] = result
                    self.tasks[task_id]["status"] = "succeeded"

            except Exception as exc:
                rospy.logerr(
                    "Operazione %s fallita: %s",
                    task_id,
                    exc
                )

                with self.lock:
                    self.tasks[task_id]["error"] = str(exc)
                    self.tasks[task_id]["status"] = "failed"

            return task_id

        future = self.executor.submit(run)

        return task_id
    
    def get_tasks(self):
        with self.lock:
            return {
                task_id: dict(task)
                for task_id, task in self.tasks.items()
            }

    # ========================================================
    # DEPLOYMENT ROBOT
    # ========================================================

    def deploy_robot(self, robot, area):
        if robot not in self.robots_dict:
            raise ValueError(f"Unknown robot: {robot}")

        if area not in self.areas_dict:
            raise ValueError(f"Unknown area: {area}")

        area_x = self.areas_dict[area]["coordinates"]["x"]
        area_y = self.areas_dict[area]["coordinates"]["y"]

        if self.teleport_robots:
            proxy = rospy.ServiceProxy(
                "/gazebo/set_model_state",
                SetModelState
            )

            state = ModelState()
            state.model_name = robot
            state.reference_frame = "map"

            state.pose.position.x = area_x-0.3
            state.pose.position.y = area_y-0.3
            state.pose.position.z = 0.0
            state.pose.orientation.w = 1.0

            state.twist.linear.x = 0.0
            state.twist.linear.y = 0.0
            state.twist.angular.z = 0.0

            response = proxy(state)

            if not response.success:
                raise RuntimeError(response.status_message)

        clear_costmaps = rospy.ServiceProxy(
            f"/{robot}/move_base/clear_costmaps",
            Empty
        )
        clear_costmaps()

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = area_x
        goal.pose.position.y = area_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.robot_publishers[robot].publish(goal)

        return {
            "robot": robot,
            "area": area,
            "goal_x": area_x,
            "goal_y": area_y,
            "status": "goal_published",
        }

    def deploy_batch(self, deployments):
        task_ids = []

        for deployment in deployments:
            robot = deployment["robot_to_send"]
            area = deployment["area_to_reach"]

            task_id = self.submit_task(
                f"Deploy {robot} to {area}",
                self.deploy_robot,
                robot,
                area
            )

            task_ids.append(task_id)

        return task_ids



    def handleRobotGoalChecker(self, goal):
        """Process a robot navigation result and its intruder detection report."""

        result = goalCheckerLLMResult()

        try:
            # Validate the incoming action goal.
            if goal is None:
                raise ValueError("No action goal received")

            message = goal.message_for_LLM.strip()

            if not message:
                raise ValueError("The received message is empty")

            rospy.loginfo(
                "Received robot verification message: %s",
                message
            )

            # Extract the robot name and the report content.
            if ":" in message:
                robot, report = message.split(":", 1)
                robot = robot.strip()
                report = report.strip()
            else:
                robot = "unknown"
                report = message

            # Determine whether navigation succeeded.
            report_lower = report.lower()

            goal_reached = (
                "intruder identified" in report_lower
                or "no intruders identified" in report_lower
                or "no intruder identified" in report_lower
                or "goal reached" in report_lower
                or "succeeded" in report_lower
            )

            goal_failed = (
                "aborted" in report_lower
                or "rejected" in report_lower
                or "failed" in report_lower
                or "preempted" in report_lower
                or "recalled" in report_lower
            )

            # Extract the destination area from the report.
            area = "unknown"
            area_match = re.search(
                r"Area\s+(.+?):",
                report,
                re.IGNORECASE
            )

            if area_match:
                area = area_match.group(1).strip()
            else:
                clear_match = re.search(
                    r"Area\s+(.+?)\s+clear",
                    report,
                    re.IGNORECASE
                )

                if clear_match:
                    area = clear_match.group(1).strip()

            # Determine the intruder detection result.
            if "intruder identified" in report_lower:
                intruder_detected = True
                detection_status = "Intruder detected"

            elif (
                "no intruders identified" in report_lower
                or "no intruder identified" in report_lower
                or "area " in report_lower and " clear" in report_lower
            ):
                intruder_detected = False
                detection_status = "No intruder detected"

            else:
                intruder_detected = None
                detection_status = "Detection unavailable"

            # Override the navigation status when failure is explicit.
            if goal_failed:
                goal_reached = False
                intruder_detected = None
                detection_status = "Detection not completed"

            elif not goal_reached:
                raise ValueError(
                    f"Unable to determine navigation status from: {report}"
                )

            # Build the event for the supervisory interface.
            event = {
                "timestamp": datetime.now().isoformat(
                    timespec="seconds"
                ),
                "robot": robot,
                "area": area,
                "goal_reached": goal_reached,
                "intruder_detected": intruder_detected,
                "goal_status": (
                    "Goal reached"
                    if goal_reached
                    else "Goal not reached"
                ),
                "detection_status": detection_status,
                "message": report,
                "status": (
                    "completed"
                    if goal_reached
                    else "failed"
                ),
            }

            # Update the latest event and the event history.
            with self.lock:
                self.robot_goal_events[robot] = event.copy()

                self.robot_goal_history.insert(
                    0,
                    event.copy()
                )

                self.robot_goal_history = (
                    self.robot_goal_history[:100]
                )

            # Publish feedback about navigation completion.
            feedback = goalCheckerLLMFeedback()
            feedback.status_LLM = (
                f"Robot {robot}, area {area}: "
                f"{event['goal_status']}"
            )
            self.robotGoalCheckerServer.publish_feedback(feedback)

            # Publish feedback about the intruder detection result.
            feedback.status_LLM = (
                f"Robot {robot}: {detection_status}"
            )
            self.robotGoalCheckerServer.publish_feedback(feedback)

            # Set the final action result.
            result.success = (
                f"{event['goal_status']}; {detection_status}"
            )

            # Finish the action according to the navigation outcome.
            if goal_reached:
                self.robotGoalCheckerServer.set_succeeded(result)
            else:
                self.robotGoalCheckerServer.set_aborted(result)

        except Exception as exc:
            rospy.logerr(
                "Error in handleRobotGoalChecker: %s",
                exc
            )

            result.success = f"Verification failed: {exc}"

            # Record the error so that it can be displayed in the frontend.
            error_event = {
                "timestamp": datetime.now().isoformat(
                    timespec="seconds"
                ),
                "robot": "unknown",
                "area": "unknown",
                "goal_reached": None,
                "intruder_detected": None,
                "goal_status": "Verification error",
                "detection_status": "Detection unavailable",
                "message": str(exc),
                "status": "failed",
            }

            with self.lock:
                self.robot_goal_events["unknown"] = error_event.copy()

                self.robot_goal_history.insert(
                    0,
                    error_event.copy()
                )

                self.robot_goal_history = (
                    self.robot_goal_history[:100]
                )

            self.robotGoalCheckerServer.set_aborted(result)


    def get_robot_goal_events(self):
        """Return the latest verification event for each robot."""
        with self.lock:
            return {
                robot: event.copy()
                for robot, event in self.robot_goal_events.items()
            }


    def get_robot_goal_history(self):
        """Return the most recent robot verification events."""
        with self.lock:
            return [
                event.copy()
                for event in self.robot_goal_history
            ]
    # ========================================================
    # CONTROLLO PORTE
    # ========================================================

    def control_door(self, actuator, command):
        if actuator not in self.actuators_dict:
            raise ValueError(f"Porta sconosciuta: {actuator}")

        if command not in ("open", "close"):
            raise ValueError("Comando ammesso: open oppure close")

        service_name = self.actuators_dict[actuator][
            "ros_service_name"
        ]

        rospy.wait_for_service(service_name, timeout=5)

        service = rospy.ServiceProxy(
            service_name,
            doorStringCommand
        )

        request = doorStringCommandRequest()
        request.command = command

        response = service(request)

        return {
            "actuator": actuator,
            "command": command,
            "response": str(response),
        }

    def control_doors_batch(self, commands):
        task_ids = []

        for item in commands:
            task_id = self.submit_task(
                f"{item['actuator_name']} -> {item['command']}",
                self.control_door,
                item["actuator_name"],
                item["command"]
            )

            task_ids.append(task_id)

        return task_ids

    # ========================================================
    # TELECAMERE
    # ========================================================

    def open_cameras(self, camera_names):
        if len(camera_names) > MAX_VIDEO_STREAMS:
            raise ValueError(
                f"Massimo {MAX_VIDEO_STREAMS} stream simultanei"
            )

        topics = []

        for name in camera_names:
            key = name.lower()

            if key not in self.cameras_dict:
                raise ValueError(f"Camera sconosciuta: {name}")

            topics.append(
                self.cameras_dict[key]["ros_topic"]
            )

        if not CAMERA_SCRIPT.is_file():
            raise FileNotFoundError(
                f"Script video non trovato: {CAMERA_SCRIPT}"
            )
        print(CAMERA_SCRIPT)

        self.close_cameras()

        self.camera_process = subprocess.Popen(
            ["python3", str(CAMERA_SCRIPT), *topics]
        )

        self.camera_topics = topics

        return {
            "opened_topics": topics,
            "process_id": self.camera_process.pid,
        }

    def close_cameras(self):
        process = self.camera_process

        if process is not None and process.poll() is None:
            process.terminate()

            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

        self.camera_process = None
        self.camera_topics = []

        return {"status": "closed"}

    # ========================================================
    # CONTROLLO ESPERIMENTO E SEQUENZE SENSORI
    # ========================================================

    def next_sensor_sequence(self):
        """Richiede il passaggio alla sequenza di sensori successiva."""
        rospy.set_param("/next_sequence", True)

        return {
            "parameter": "/next_sequence",
            "value": True,
        }

    def stop_experiment(self):
        """Richiede l'interruzione dell'esperimento."""
        rospy.set_param("/stop_experiment", True)

        return {
            "parameter": "/stop_experiment",
            "value": True,
        }