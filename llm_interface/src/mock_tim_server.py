import copy
import logging
from flask import Flask, jsonify, request

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TIM_Mock_Gateway")

app = Flask(__name__)

# --- Default Fleet Baseline ---
# Go2: Indoor ground robot, high battery, Idle at (2.0, 1.5)
# Dronus: Aerial drone, good battery, Idle at (-3.0, -2.0)
# DJI: Drone in Maintenance
# Robox: Ground rover, low battery (18%) to test rejection criteria
INITIAL_FLEET_STATE = [
    {
        "id": "1234",
        "model": "go2",
        "name": "go2 sim 1",
        "type": "dog",
        "description": "go2 simulator",
        "map_id": "mId0",
        "map_type": "plan",
        "status": "Idle",
        "battery": 92,
        "position": {"x": 2.0, "y": 1.5, "z": 0.0},
        "target": None
    },
    {
        "id": "5678",
        "model": "dronus",
        "name": "dronus02",
        "type": "drone",
        "description": "test dronus",
        "map_id": "mId0",
        "map_type": "plan",
        "status": "Idle",
        "battery": 78,
        "position": {"x": -3.0, "y": -2.0, "z": 1.2},
        "target": None
    },
    {
        "id": "9901",
        "model": "dji",
        "name": "DJI Outdoor",
        "type": "drone",
        "description": "Test simulatore",
        "map_id": "mId1",
        "map_type": "geo",
        "status": "Maintenance",
        "battery": 100,
        "position": {"x": 8.0, "y": 4.0, "z": 0.0},
        "target": None
    },
    {
        "id": "7702",
        "model": "robox",
        "name": "robox test",
        "type": "rover",
        "description": "ground rover unit",
        "map_id": "mId0",
        "map_type": "plan",
        "status": "Idle",
        "battery": 18,  # Below 25% threshold to test low-battery rejection
        "position": {"x": 4.0, "y": 1.2, "z": 0.0},
        "target": None
    }
]

# --- Default Baseline Alarms ---
# Area C is configured as an indoor office zone
INITIAL_ACTIVE_ALARMS = [
    {
        "id": "23",
        "name": "alrm03",
        "model": "shelly",
        "description": "allarme finestra ufficio 3145",
        "status": "open_twilight",
        "area": "C",
        "environment": "indoor",
        "coordinates": {"x": 5.0, "y": 1.0, "z": 0.0},
        "map_id": "mId1",
        "map_type": "geo"
    }
]

# In-memory working state
FLEET_STATE = copy.deepcopy(INITIAL_FLEET_STATE)
ACTIVE_ALARMS = copy.deepcopy(INITIAL_ACTIVE_ALARMS)


def reset_mock_state():
    """Resets fleet and alarm registry to baseline."""
    global FLEET_STATE, ACTIVE_ALARMS
    FLEET_STATE = copy.deepcopy(INITIAL_FLEET_STATE)
    ACTIVE_ALARMS = copy.deepcopy(INITIAL_ACTIVE_ALARMS)
    logger.info("Mock TIM state restored to initial baseline.")


# ==========================================
# 0. Test Harness Reset Endpoint
# ==========================================

@app.route("/test/reset", methods=["GET", "POST"])
def reset_endpoint():
    reset_mock_state()
    return jsonify({"status": "success", "message": "State reset successfully"}), 200


# ==========================================
# 1. MultiRobot Telemetry Endpoints
# ==========================================

@app.route("/sec/multirobot/robot/connected", methods=["GET"])
@app.route("/gateway/multirobot/robot/connected", methods=["GET"])
def get_connected_robots():
    logger.info(f"Serving connected fleet: {len(FLEET_STATE)} robots online.")
    return jsonify(FLEET_STATE), 200


@app.route("/sec/multirobot/robot/connected/", methods=["GET"])
@app.route("/gateway/multirobot/robot/connected/", methods=["GET"])
def get_connected_by_model(model):
    filtered = [r for r in FLEET_STATE if r.get("model", "").lower() == str(model).lower()]
    return jsonify(filtered), 200


# ==========================================
# 2. Orchestrator Alarms Endpoints
# ==========================================

@app.route("/sec/orchestrator/alarm", methods=["GET"])
@app.route("/gateway/orchestrator/alarm", methods=["GET"])
def get_orchestrator_alarms():
    logger.info(f"Serving active orchestrator alarms: {len(ACTIVE_ALARMS)} active.")
    return jsonify(ACTIVE_ALARMS), 200


@app.route("/sec/orchestrator/alarm/deactive//", methods=["GET", "POST"])
@app.route("/gateway/orchestrator/alarm/deactive//", methods=["GET", "POST"])
def deactivate_alarm(model, alarm_id):
    global ACTIVE_ALARMS
    logger.info(f"Deactivating alarm: ID {alarm_id} (model: {model})")

    initial_count = len(ACTIVE_ALARMS)
    ACTIVE_ALARMS = [
        a for a in ACTIVE_ALARMS
        if not (str(a.get("id")) == str(alarm_id) or str(a.get("name")) == str(alarm_id))
    ]

    if len(ACTIVE_ALARMS) < initial_count:
        return jsonify({"status": "success", "message": f"Alarm {alarm_id} cleared"}), 200
    return jsonify({"status": "not_found", "message": f"Alarm {alarm_id} was not active"}), 404


# ==========================================
# 3. Model-Specific Robot Command Dispatch
# ==========================================

@app.route("/sec/<model>/command/<robot_id>", methods=["POST"])
@app.route("/sec/<model>/command/<robot_id>/", methods=["POST"])
@app.route("/gateway/<model>/command/<robot_id>", methods=["POST"])
def send_robot_command(model, robot_id):
    data = request.get_json(silent=True, force=True) or {}
    logger.info(f"Command received for {model} [{robot_id}]: {data}")

    command_name = data.get("command", "go-to-poi")
    params = data.get("parameters", [])

    target_poi = "alrm03"
    if params and isinstance(params, list):
        target_poi = params[0].get("value", "alrm03")

    req_id = str(robot_id).strip()
    req_model = str(model).strip().lower()

    for robot in FLEET_STATE:
        if str(robot["id"]) == req_id or robot.get("model", "").lower() == req_model:
            robot["status"] = f"Navigating to {target_poi}"
            robot["target"] = target_poi
            return jsonify({
                "status": "accepted",
                "robot_id": req_id,
                "model": req_model,
                "command": command_name,
                "action": f"Navigating towards {target_poi}"
            }), 200

    return jsonify({"error": f"Robot {req_id} not found"}), 404


# ==========================================
# 4. Local Test Helper
# ==========================================

@app.route("/test/trigger_alarm", methods=["POST"])
def trigger_alarm_for_testing():
    data = request.get_json(silent=True, force=True) or {}
    new_alarm = {
        "id": str(data.get("id", "99")),
        "name": data.get("name", "alrm03"),
        "model": data.get("model", "shelly"),
        "status": "open_twilight",
        "area": data.get("area", "C"),
        "environment": data.get("environment", "indoor"),
        "coordinates": data.get("coordinates", {"x": 5.0, "y": 1.0, "z": 0.0}),
        "map_id": "mId1",
        "map_type": "geo"
    }
    ACTIVE_ALARMS.append(new_alarm)
    logger.info(f"Injected test alarm: {new_alarm}")
    return jsonify({"status": "alarm_created", "alarm": new_alarm}), 201


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)