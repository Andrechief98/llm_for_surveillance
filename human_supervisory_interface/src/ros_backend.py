
#!/usr/bin/env python3

import json
import re
import time
import threading
import subprocess
import logging

from pathlib import Path
from datetime import datetime
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Empty

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from gazebo_plugins.srv import (
    doorStringCommand,
    doorStringCommandRequest,
)

from llm_interface.srv import (
    triggerGpt,
    triggerGptResponse,
)

from llm_interface.msg import (
    goalCheckerLLMAction,
    goalCheckerLLMResult,
    goalCheckerLLMFeedback,
)


# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

JSON_DIRECTORY = (
    BASE_DIR / "../../llm_interface/config/ROS"
).resolve()

EXPERIMENTS_FILE = (
    BASE_DIR / "../../data_analysis/intruder_detection/experiments_HUMAN.json"
).resolve()

CAMERA_SCRIPT = (
    BASE_DIR / "../../llm_interface/src/display_camera.py"
).resolve()


MAX_CONCURRENT_TASKS = 8

MAX_VIDEO_STREAMS = 6

MAX_STORED_TASKS = 200

MAX_GOAL_HISTORY = 100


# ============================================================
# ROS PARAMETERS
# ============================================================

experiment_type=rospy.get_param("experiment_type", "")
world_id = str(rospy.get_param("world_id", 1))
modality = rospy.get_param("modality", "HUMAN")
teleport_robots = rospy.get_param("teleport_robots",True)
sensor_sequence = rospy.get_param("sensor_sequence", "").split(",")
trial_id = rospy.get_param("trial_id", "0")



logger = logging.getLogger(__name__)


# ============================================================
# BACKEND
# ============================================================

class SupervisorBackend:

    def __init__(self):

        self.lock = threading.RLock()

        self.camera_lock = threading.RLock()

        self.executor = ThreadPoolExecutor(
            max_workers=MAX_CONCURRENT_TASKS,
            thread_name_prefix="supervisor",
        )

        # ----------------------------------------------------
        # TASK STORAGE
        # ----------------------------------------------------

        self.tasks = OrderedDict()

        # ----------------------------------------------------
        # ROBOT STATE
        # ----------------------------------------------------

        self.robot_publishers = {}

        self.robot_locks = {}

        self.robot_goal_events = {}

        self.robot_goal_history = []

        # ----------------------------------------------------
        # CAMERA STATE
        # ----------------------------------------------------

        self.camera_process = None

        self.camera_topics = []

        # ----------------------------------------------------
        # SENSOR STATE
        # ----------------------------------------------------

        self.active_sensors = []

        self.last_sensor_update = None

        # ----------------------------------------------------
        # EXPERIMENT STATE
        # ----------------------------------------------------

        self.world_id = world_id
        
        self.config = None

        self.active = True

        self.experiments_data = {}

        self.run_id = None

        self.current_turn = 0

        self.data_experiment_run = {
            "sensor_sequence": sensor_sequence,
            "actions": [],
            "intruder_final_area": None,
            "success": 0,
        }

        # ----------------------------------------------------
        # INITIALIZATION
        # ----------------------------------------------------

        self.load_world_config()

        self.load_experiments()

        # ----------------------------------------------------
        # ROS SERVICE
        # ----------------------------------------------------

        self.rosServer = rospy.Service(
            "/alert",
            triggerGpt,
            self.handleDataMediatorAlert,
        )

        # ----------------------------------------------------
        # ACTION SERVER
        # ----------------------------------------------------

        self.robotGoalCheckerServer = actionlib.SimpleActionServer(
            "/robot_goal_checker",
            goalCheckerLLMAction,
            execute_cb=self.handleRobotGoalChecker,
            auto_start=False,
        )

        self.robotGoalCheckerServer.start()

        logger.info(
            "SupervisorBackend initialized for world %s",
            world_id,
        )


    # ========================================================
    # CONFIGURATION LOADING
    # ========================================================

    def load_world_config(self):

        json_path = (
            JSON_DIRECTORY / f"info_map{world_id}.json"
        )

        if not json_path.is_file():

            raise FileNotFoundError(
                f"World configuration not found: {json_path}"
            )

        with json_path.open(
            "r",
            encoding="utf-8",
        ) as file:

            config = json.load(file)

        self.config = config

        self.robots_dict = (
            config.get("ros_publishers", {})
            .get("robots", {})
        )

        self.cameras_dict = (
            config.get("ros_subscribers", {})
            .get("sensors", {})
        )

        self.actuators_dict = (
            config.get("ros_services", {})
            .get("actuators", {})
        )

        self.areas_dict = config.get(
            "areas", {}
        )

        self.robots_list = list(
            self.robots_dict.keys()
        )

        self.cameras_list = list(
            self.cameras_dict.keys()
        )

        self.actuators_list = list(
            self.actuators_dict.keys()
        )

        self.areas_list = list(
            self.areas_dict.keys()
        )

        self.robot_locks = {
            robot: threading.Lock()
            for robot in self.robots_list
        }

        self.robot_publishers = {

            robot: rospy.Publisher(
                entry["ros_topic"],
                PoseStamped,
                queue_size=10,
            )

            for robot, entry in self.robots_dict.items()
        }


    def load_experiments(self):

        if not EXPERIMENTS_FILE.is_file():

            logger.warning(
                "Experiments file not found: %s",
                EXPERIMENTS_FILE,
            )

            return

        with EXPERIMENTS_FILE.open(
            "r",
            encoding="utf-8",
        ) as file:

            self.experiments_data = json.load(file)



    # ========================================================
    # EXPERIMENT RUN MANAGEMENT
    # ========================================================

    def save_experiment_data(self):

        with self.lock:

            if not EXPERIMENTS_FILE.is_file():
                raise FileNotFoundError(EXPERIMENTS_FILE)

            with EXPERIMENTS_FILE.open(
                "r",
                encoding="utf-8",
            ) as file:

                experiments = json.load(file)


            trial_name = f"trial_{trial_id}"
            world_name = f"world_{world_id}"

            # Navigate to the current trial.
            trial_runs = experiments[experiment_type][modality][world_name][trial_name]

            new_run = True
            
            for existing_run_name, run_data in trial_runs.items():
                if sensor_sequence == run_data["sensor_sequence"]:
                    new_run = False
                    run_name = existing_run_name
                    
                    match = re.fullmatch(r"run_(\d+)", existing_run_name)
                    if match:
                        self.run_id = int(match.group(1))
                    break
    
            if new_run:
                self.run_id = len(trial_runs) + 1
                run_name = f"run_{self.run_id}"
    
            # Save experiment run
            trial_runs[run_name] = self.data_experiment_run

            # Write updated JSON.

            with open(EXPERIMENTS_FILE, "w", encoding="utf-8") as file:

                json.dump(
                    experiments,
                    file,
                    indent=4,
                    ensure_ascii=False,
                )

            logger.info(
                "Experiment saved: %s / %s / %s / %s",
                experiment_type,
                world_id,
                trial_id,
                self.run_id,
            )

            return self.run_id

    # ========================================================
    # TASK MANAGEMENT
    # ========================================================

    @staticmethod
    def timestamp():

        return datetime.now().isoformat(
            timespec="seconds"
        )


    def submit_task(self, description, function, *args):

        task_id = (
            datetime.now().strftime("%Y%m%d%H%M%S%f")
        )

        queued_at = time.perf_counter()

        with self.lock:

            self.tasks[task_id] = {
                "description": description,
                "status": "queued",
                "result": None,
                "error": None,
                "created_at": self.timestamp(),
                "queue_time": None,
                "execution_time": None,
                "_queued_at": queued_at,
                "_started_at": None,
            }

            # Keep only the latest tasks.
            while len(self.tasks) > MAX_STORED_TASKS:

                oldest_id = next(iter(self.tasks))

                if self.tasks[oldest_id]["status"] in (
                    "queued",
                    "running",
                ):

                    break

                self.tasks.popitem(last=False)


        def run():

            started = time.perf_counter()

            with self.lock:

                task = self.tasks.get(task_id)

                if task is None:
                    return

                task["status"] = "running"

                task["_started_at"] = started

                task["queue_time"] = (
                    started - task["_queued_at"]
                )

            try:

                result = function(*args)

                finished = time.perf_counter()

                with self.lock:

                    task = self.tasks.get(task_id)

                    if task is not None:

                        task["result"] = result

                        task["status"] = "succeeded"

                        task["execution_time"] = (
                            finished - started
                        )

            except Exception as exc:

                finished = time.perf_counter()

                logger.exception(
                    "Task %s failed",
                    task_id,
                )

                with self.lock:

                    task = self.tasks.get(task_id)

                    if task is not None:

                        task["error"] = str(exc)

                        task["status"] = "failed"

                        task["execution_time"] = (
                            finished - started
                        )

        future = self.executor.submit(run)

        return task_id


    def get_tasks(self):

        with self.lock:

            return {
                task_id: {
                    key: value
                    for key, value in task.items()
                    if not key.startswith("_")
                }

                for task_id, task in self.tasks.items()
            }


    def get_recent_tasks(self, limit=20):

        with self.lock:

            recent = list(
                self.tasks.items()
            )[-limit:]

            recent.reverse()

            return [
                (
                    task_id,
                    {
                        key: value
                        for key, value in task.items()
                        if not key.startswith("_")
                    },
                )

                for task_id, task in recent
            ]


    # ========================================================
    # ROBOT DEPLOYMENT
    # ========================================================

    def deploy_robot(self, robot, area):

        if robot not in self.robots_dict:

            raise ValueError(
                f"Unknown robot: {robot}"
            )

        if area not in self.areas_dict:

            raise ValueError(
                f"Unknown area: {area}"
            )

        area_x = self.areas_dict[area][
            "coordinates"
        ]["x"]

        area_y = self.areas_dict[area][
            "coordinates"
        ]["y"]

        # Serialize commands for the same robot.
        with self.robot_locks[robot]:

            if teleport_robots:

                rospy.wait_for_service(
                    "/gazebo/set_model_state",
                    timeout=5,
                )

                proxy = rospy.ServiceProxy(
                    "/gazebo/set_model_state",
                    SetModelState,
                )

                state = ModelState()

                state.model_name = robot

                state.reference_frame = "map"

                state.pose.position.x = area_x - 0.3

                state.pose.position.y = area_y - 0.3

                state.pose.position.z = 0.0

                state.pose.orientation.w = 1.0

                state.twist.linear.x = 0.0

                state.twist.linear.y = 0.0

                state.twist.angular.z = 0.0

                response = proxy(state)

                if not response.success:

                    raise RuntimeError(
                        response.status_message
                    )

            clear_costmaps = rospy.ServiceProxy(
                f"/{robot}/move_base/clear_costmaps",
                Empty,
            )

            rospy.wait_for_service(
                f"/{robot}/move_base/clear_costmaps",
                timeout=5,
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

        self._validate_deployments(deployments)

        if not deployments:
            return []

        turn_id = self.start_turn()

        self.log_action(
            turn_id,
            "deploy",
            {
                item["robot_to_send"]: item["area_to_reach"]
                for item in deployments
            },
        )

        return [
            self.submit_task(
                f"Deploy {item['robot_to_send']} to {item['area_to_reach']}",
                self.deploy_robot,
                item["robot_to_send"],
                item["area_to_reach"],
            )
            for item in deployments
        ]


    def _validate_deployments(self, deployments):

        for item in deployments:

            robot = item["robot_to_send"]

            area = item["area_to_reach"]

            if robot not in self.robots_dict:

                raise ValueError(
                    f"Unknown robot: {robot}"
                )

            if area not in self.areas_dict:

                raise ValueError(
                    f"Unknown area: {area}"
                )



    # ========================================================
    # SENSOR MANAGEMENT
    # ========================================================

    def handleDataMediatorAlert(self, req):

        if not req:
            return triggerGptResponse("Failed")

        try:

            request = json.loads(
                req.alert_info
            )

            activated_sensors = request.get(
                "activated_sensors",
                [],
            )

            if not isinstance(activated_sensors, list):

                raise ValueError(
                    "activated_sensors must be a list"
                )

            with self.lock:

                self.active_sensors = list(
                    activated_sensors
                )

                self.last_sensor_update = self.timestamp()

            return triggerGptResponse("Success")

        except Exception as exc:

            logger.exception(
                "Error processing sensor alert"
            )

            return triggerGptResponse(
                f"Error: {exc}"
            )


    def get_active_sensors(self):

        with self.lock:

            return {
                "sensors": self.active_sensors.copy(),

                "last_update": self.last_sensor_update,
            }


    # ========================================================
    # ROBOT GOAL VERIFICATION
    # ========================================================

    def handleRobotGoalChecker(self, goal):

        result = goalCheckerLLMResult()

        try:

            if goal is None:

                raise ValueError(
                    "No action goal received"
                )

            message = goal.message_for_LLM.strip()

            if not message:

                raise ValueError(
                    "Received message is empty"
                )

            logger.info(
                "Robot verification received: %s",
                message,
            )

            # ------------------------------------------------
            # EXTRACT ROBOT AND REPORT
            # ------------------------------------------------

            if ":" in message:

                robot, report = message.split(
                    ":",
                    1,
                )

                robot = robot.strip()

                report = report.strip()

            else:

                robot = "unknown"

                report = message

            report_lower = report.lower()

            # ------------------------------------------------
            # NAVIGATION STATUS
            # ------------------------------------------------

            goal_reached = any(
                phrase in report_lower

                for phrase in (
                    "intruder identified",
                    "no intruders identified",
                    "no intruder identified",
                    "goal reached",
                    "succeeded",
                )
            )

            goal_failed = any(
                phrase in report_lower

                for phrase in (
                    "aborted",
                    "rejected",
                    "failed",
                    "preempted",
                    "recalled",
                )
            )

            # ------------------------------------------------
            # AREA EXTRACTION
            # ------------------------------------------------

            area = "unknown"

            area_match = re.search(
                r"Area\s+(.+?):",
                report,
                re.IGNORECASE,
            )

            if area_match:

                area = area_match.group(1).strip()

            else:

                clear_match = re.search(
                    r"Area\s+(.+?)\s+clear",
                    report,
                    re.IGNORECASE,
                )

                if clear_match:

                    area = clear_match.group(1).strip()

            # ------------------------------------------------
            # INTRUDER DETECTION
            # ------------------------------------------------

            if "intruder identified" in report_lower:

                intruder_detected = True

                detection_status = "Intruder detected"

            elif (

                "no intruders identified" in report_lower

                or "no intruder identified" in report_lower

                or (
                    "area " in report_lower
                    and " clear" in report_lower
                )
            ):

                intruder_detected = False

                detection_status = "No intruder detected"

            else:

                intruder_detected = None

                detection_status = "Detection unavailable"

            # ------------------------------------------------
            # FAILURE OVERRIDE
            # ------------------------------------------------

            if goal_failed:

                goal_reached = False

                intruder_detected = None

                detection_status = "Detection not completed"

            elif not goal_reached:

                raise ValueError(
                    f"Unable to determine navigation status: {report}"
                )

            # ------------------------------------------------
            # BUILD EVENT
            # ------------------------------------------------

            event = {

                "timestamp": self.timestamp(),

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

            # ------------------------------------------------
            # UPDATE EVENT HISTORY
            # ------------------------------------------------

            with self.lock:

                self.robot_goal_events[robot] = event.copy()

                self.robot_goal_history.insert(
                    0,
                    event.copy(),
                )

                self.robot_goal_history = (
                    self.robot_goal_history[:MAX_GOAL_HISTORY]
                )

            # ------------------------------------------------
            # PUBLISH FEEDBACK
            # ------------------------------------------------

            feedback = goalCheckerLLMFeedback()

            feedback.status_LLM = (
                f"Robot {robot}, area {area}: "
                f"{event['goal_status']}"
            )

            self.robotGoalCheckerServer.publish_feedback(
                feedback
            )

            feedback.status_LLM = (
                f"Robot {robot}: {detection_status}"
            )

            self.robotGoalCheckerServer.publish_feedback(
                feedback
            )

            # ------------------------------------------------
            # FINAL RESULT
            # ------------------------------------------------

            result.success = (
                f"{event['goal_status']}; "
                f"{detection_status}"
            )

            if goal_reached:

                self.robotGoalCheckerServer.set_succeeded(
                    result
                )

            else:

                self.robotGoalCheckerServer.set_aborted(
                    result
                )

        except Exception as exc:

            logger.exception(
                "Robot goal verification failed"
            )

            result.success = (
                f"Verification failed: {exc}"
            )

            error_event = {

                "timestamp": self.timestamp(),

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

                self.robot_goal_events["unknown"] = (
                    error_event.copy()
                )

                self.robot_goal_history.insert(
                    0,
                    error_event.copy(),
                )

                self.robot_goal_history = (
                    self.robot_goal_history[:MAX_GOAL_HISTORY]
                )

            self.robotGoalCheckerServer.set_aborted(
                result
            )


    def get_robot_goal_events(self):

        with self.lock:

            return {
                robot: event.copy()

                for robot, event in self.robot_goal_events.items()
            }


    def get_robot_goal_history(self):

        with self.lock:

            return [
                event.copy()

                for event in self.robot_goal_history
            ]


    # ========================================================
    # DOOR AND ACTUATOR CONTROL
    # ========================================================

    def control_door(self, actuator, command):

        if actuator not in self.actuators_dict:

            raise ValueError(
                f"Unknown actuator: {actuator}"
            )

        if command not in ("open", "close"):

            raise ValueError(
                "Allowed commands: open or close"
            )

        service_name = self.actuators_dict[actuator][
            "ros_service_name"
        ]

        rospy.wait_for_service(
            service_name,
            timeout=5,
        )

        service = rospy.ServiceProxy(
            service_name,
            doorStringCommand,
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

        self._validate_door_commands(commands)

        if not commands:
            return []

        turn_id = self.start_turn()

        self.log_action(
            turn_id,
            "doors",
            {
                item["actuator_name"]: item["command"]
                for item in commands
            },
        )

        return [
            self.submit_task(
                f"{item['actuator_name']} -> {item['command']}",
                self.control_door,
                item["actuator_name"],
                item["command"],
            )
            for item in commands
        ]


    def _validate_door_commands(self, commands):

        for item in commands:

            actuator = item["actuator_name"]

            command = item["command"]

            if actuator not in self.actuators_dict:

                raise ValueError(
                    f"Unknown actuator: {actuator}"
                )

            if command not in ("open", "close"):

                raise ValueError(
                    f"Invalid command: {command}"
                )


    # ========================================================
    # CAMERA MANAGEMENT
    # ========================================================

    def open_cameras(self, camera_names):

        if len(camera_names) > MAX_VIDEO_STREAMS:

            raise ValueError(
                f"Maximum {MAX_VIDEO_STREAMS} camera streams allowed"
            )

        topics = []

        for name in camera_names:

            key = name.lower()

            if key not in self.cameras_dict:

                raise ValueError(
                    f"Unknown camera: {name}"
                )

            topics.append(
                self.cameras_dict[key]["ros_topic"]
            )

        if not CAMERA_SCRIPT.is_file():

            raise FileNotFoundError(
                f"Camera script not found: {CAMERA_SCRIPT}"
            )

        turn_id = self.start_turn()

        if turn_id is None:
            turn_id = self.start_turn()

        self.log_action(
            turn_id,
            "cameras",
            {
                camera: "open"
                for camera in camera_names
            },
        )

        with self.camera_lock:

            # Close previous process before starting another.
            self._close_cameras_locked()

            process = subprocess.Popen(
                [
                    "python3",
                    str(CAMERA_SCRIPT),
                    *topics,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            self.camera_process = process

            self.camera_topics = topics.copy()

            return {

                "opened_topics": topics,

                "process_id": process.pid,

                "status": "started",
            }


    def _close_cameras_locked(self):

        process = self.camera_process

        if process is not None:

            if process.poll() is None:

                process.terminate()

                try:

                    process.wait(timeout=3)

                except subprocess.TimeoutExpired:

                    process.kill()

                    process.wait()

            self.camera_process = None

        self.camera_topics = []


    def close_cameras(self):

        with self.camera_lock:

            self._close_cameras_locked()

        return {
            "status": "closed"
        }


    def get_camera_topics(self):

        with self.camera_lock:

            process = self.camera_process

            if process is None:

                return []

            # Avoid reporting feeds from an exited process.
            if process.poll() is not None:

                self.camera_process = None

                self.camera_topics = []

                return []

            return self.camera_topics.copy()



    # ========================================================
    # GLOBAL EXECUTION
    # ========================================================

    def execute_all_batch(
        self,
        deployments=None,
        camera_names=None,
        door_commands=None,
    ):

        deployments = deployments or []
        camera_names = camera_names or []
        door_commands = door_commands or []

        # ----------------------------------------------------
        # VALIDATE ALL OPERATIONS BEFORE SUBMISSION
        # ----------------------------------------------------

        self._validate_deployments(
            deployments
        )

        self._validate_door_commands(
            door_commands
        )

        if len(camera_names) > MAX_VIDEO_STREAMS:

            raise ValueError(
                f"Maximum {MAX_VIDEO_STREAMS} camera streams allowed"
            )

        for camera in camera_names:

            if camera.lower() not in self.cameras_dict:

                raise ValueError(
                    f"Unknown camera: {camera}"
                )

        if not (
            deployments
            or camera_names
            or door_commands
        ):

            raise ValueError(
                "No operations selected"
            )

        turn_id = self.start_turn()


        # ----------------------------------------------------
        # REGISTER ALL ACTIONS IN THE SAME TURN
        # ----------------------------------------------------

        if deployments:

            self.log_action(
                turn_id,
                "deploy",
                {
                    item["robot_to_send"]: item["area_to_reach"]
                    for item in deployments
                },
            )


        if camera_names:

            self.log_action(
                turn_id,
                "cameras",
                {
                    camera: "open"
                    for camera in camera_names
                },
            )


        if door_commands:

            self.log_action(
                turn_id,
                "doors",
                {
                    item["actuator_name"]: item["command"]
                    for item in door_commands
                },
            )

        # ----------------------------------------------------
        # SUBMIT ALL OPERATIONS
        # ----------------------------------------------------

        task_ids = []

        # Robot deployments

        for item in deployments:

            robot = item["robot_to_send"]

            area = item["area_to_reach"]

            task_id = self.submit_task(
                f"Deploy {robot} to {area}",
                self.deploy_robot,
                robot,
                area,
            )

            task_ids.append(task_id)


        # Camera opening

        if camera_names:

            task_id = self.submit_task(
                f"Open cameras: {', '.join(camera_names)}",
                self.open_cameras,
                list(camera_names),
            )

            task_ids.append(task_id)


        # Door commands

        for item in door_commands:

            actuator = item["actuator_name"]

            command = item["command"]

            task_id = self.submit_task(
                f"{actuator} -> {command}",
                self.control_door,
                actuator,
                command,
            )

            task_ids.append(task_id)


        return {
            "turn": turn_id,
            "task_ids": task_ids,
        }


    # ========================================================
    # EXPERIMENT CONTROL
    # ========================================================

    def next_sensor_sequence(self):

        intruder_final_area = rospy.get_param("/intruder_final_area","")
        with self.lock:
            self.data_experiment_run["intruder_final_area"] = intruder_final_area

        self.save_experiment_data()

        rospy.set_param("/next_sequence",True)

        return {

            "parameter": "/next_sequence",

            "value": True,
        }


    def stop_experiment(self):

        intruder_final_area = rospy.get_param("/intruder_final_area","")
        with self.lock:
            self.data_experiment_run["intruder_final_area"] = intruder_final_area

        self.save_experiment_data()
    
        rospy.set_param("/stop_experiment",True)

        return {

            "parameter": "/stop_experiment",

            "value": True,
        }


    def report_intruder_detected(self):

        intruder_final_area = rospy.get_param("/intruder_final_area","")

        with self.lock:
            self.data_experiment_run["intruder_final_area"] = intruder_final_area
            self.data_experiment_run["success"] = 1

        rospy.set_param("/intruder_detected", True)

        return {
            "intruder_detected": True,
            "false_positive": False,
        }


    def report_false_positive(self):

        intruder_final_area = rospy.get_param("/intruder_final_area", "")

        with self.lock:
            self.data_experiment_run["intruder_final_area"] = intruder_final_area
            self.data_experiment_run["success"] = (
                1 if intruder_final_area == "FP" else 0
            )

        return {
            "intruder_detected": False,
            "false_positive": True,
        }


    # ========================================================
    # TURN MANAGEMENT
    # ========================================================

    def start_turn(self):

        with self.lock:

            self.current_turn += 1

            turn = {
                "deploy": {},
                "cameras": {},
                "doors": {},
            }

            self.data_experiment_run["actions"].append(turn)

            return self.current_turn

    def log_action(self, turn_id, action_type, actions):

        with self.lock:

            if action_type not in ("deploy", "cameras", "doors"):
                raise ValueError(
                    f"Unknown action type: {action_type}"
                )

            index = turn_id - 1

            if index < 0 or index >= len(self.data_experiment_run["actions"]):
                raise ValueError(
                    f"Invalid turn: {turn_id}"
                )

            self.data_experiment_run["actions"][index][action_type].update(
                actions
            )

    # ========================================================
    # BACKEND STATUS
    # ========================================================

    def get_backend_status(self):

        with self.lock:

            task_counts = {

                "queued": 0,

                "running": 0,

                "succeeded": 0,

                "failed": 0,
            }

            for task in self.tasks.values():

                status = task.get("status")

                if status in task_counts:

                    task_counts[status] += 1

        return {

            "world_id": world_id,

            "robots": len(self.robots_list),

            "cameras": len(self.cameras_list),

            "actuators": len(self.actuators_list),

            "areas": len(self.areas_list),

            "active_sensors": len(self.active_sensors),

            "active_camera_topics": len(
                self.get_camera_topics()
            ),

            "tasks": task_counts,
        }


    # ========================================================
    # CLEANUP
    # ========================================================

    def shutdown(self):

        logger.info(
            "Shutting down SupervisorBackend"
        )

        with self.lock:

            if not self.active:

                return

            self.active = False

        # Stop camera process.

        try:

            self.close_cameras()

        except Exception:

            logger.exception(
                "Error closing camera process"
            )

        # Stop accepting new executor tasks and wait for
        # already submitted operations to finish.

        self.executor.shutdown(
            wait=True,
            cancel_futures=False,
        )

        logger.info(
            "SupervisorBackend shutdown completed"
        )
