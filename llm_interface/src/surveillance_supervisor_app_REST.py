import chainlit as cl
from chainlit.input_widget import Select
from openai import AsyncOpenAI
import requests
import json
import os
import math
import logging
import asyncio

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TIM_Agent")

# TIM API Configuration (Local Mock -> Lab Gateway)
TIM_BASE_URL = os.getenv("TIM_BASE_URL", "http://localhost:5000")
TIM_HEADERS = {
    "Authorization": os.getenv("TIM_AUTH_TOKEN", "Bearer mock_token"),
    "Content-Type": "application/json"
}

# --- Multi-Factor Solver Configuration & Weights ---
W_DISTANCE = 0.65       # 65% proximity weighting
W_BATTERY = 0.35        # 35% battery preservation weighting
MIN_BATTERY_LEVEL = 25  # Minimum operational battery threshold (%)

# Mobility & Zone Compatibility
CAPABILITY_MAP = {
    "dog": {"indoor": True, "outdoor": True, "max_z": 0.5},
    "rover": {"indoor": True, "outdoor": False, "max_z": 0.2},
    "drone": {"indoor": False, "outdoor": True, "max_z": 50.0}
}

# --- Load Environment Knowledge Base ---
INFO_FILE_PATH = os.path.join(os.path.dirname(__file__), "info.json")
try:
    with open(INFO_FILE_PATH, "r") as f:
        MAP_DATA = json.load(f)
    logger.info(f"Loaded environment topography from {INFO_FILE_PATH}")
except Exception as e:
    logger.warning(f"Could not load info.json ({e}). Falling back to empty environment.")
    MAP_DATA = {}


# --- Spatial Resolution & Constrained Solver Functions ---

def get_zone_from_coords(x: float, y: float) -> str:
    """Matches coordinates (x, y) against area bounding boxes defined in info.json."""
    areas = MAP_DATA.get("areas", {})
    for zone_id, details in areas.items():
        xr = details.get("coordinate_ranges", {}).get("x", {})
        yr = details.get("coordinate_ranges", {}).get("y", {})

        x_min = xr.get("min", float("-inf"))
        x_max = xr.get("max", float("inf"))
        y_min = yr.get("min", float("-inf"))
        y_max = yr.get("max", float("inf"))

        if x_min <= x <= x_max and y_min <= y <= y_max:
            desc = details.get("description", "")
            poi = details.get("poi_name", "")
            label = f"Zone {zone_id}"
            if poi:
                label += f" ({poi})"
            elif desc:
                label += f" ({desc})"
            return label

    return "Unknown / In-Transit Area"


def get_alarm_target_coordinates(alarm_query: str, active_alarms: list = None):
    """Finds coordinates and environment type from live alarm telemetry, sensor registry, or area database."""
    if not alarm_query:
        return 5.0, 1.0, 0.0, "C", False

    query_str = str(alarm_query).strip().lower()

    # 1. Directly check live alarms returned by the orchestrator
    if active_alarms:
        for a in active_alarms:
            a_id = str(a.get("id", "")).lower()
            a_name = str(a.get("name", "")).lower()
            if query_str in (a_id, a_name) or a_name in query_str:
                is_outdoor = a.get("environment", "").lower() == "outdoor"
                if "coordinates" in a and isinstance(a["coordinates"], dict):
                    coords = a["coordinates"]
                    tx = float(coords.get("x", 5.0))
                    ty = float(coords.get("y", 1.0))
                    tz = float(coords.get("z", 0.0))
                    area_key = a.get("area", "C")
                    logger.info(f"[COORDS MATCH] Telemetry Alarm '{a_name}' -> ({tx}, {ty}, {tz}) in Zone {area_key}")
                    return tx, ty, tz, area_key, is_outdoor

                area_key = a.get("area")
                if area_key and area_key in MAP_DATA.get("areas", {}):
                    area_info = MAP_DATA["areas"][area_key]
                    coords = area_info.get("coordinates", {})
                    tx = float(coords.get("x", 5.0))
                    ty = float(coords.get("y", 1.0))
                    tz = float(coords.get("z", 0.0))
                    is_outdoor = area_info.get("environment", "").lower() == "outdoor"
                    logger.info(f"[COORDS MATCH] Telemetry Area Key '{area_key}' -> ({tx}, {ty}, {tz})")
                    return tx, ty, tz, area_key, is_outdoor

    # 2. Check deployed sensors in info.json
    sensors = MAP_DATA.get("deployed_assets", {}).get("sensors", {})
    for s_name, s_data in sensors.items():
        if s_name.lower() == query_str or str(s_data.get("id")).lower() == query_str:
            area_key = s_data.get("assigned_area", "C")
            area_info = MAP_DATA.get("areas", {}).get(area_key, {})
            coords = area_info.get("coordinates", {})
            tx = float(coords.get("x", 5.0))
            ty = float(coords.get("y", 1.0))
            tz = float(coords.get("z", 0.0))
            is_outdoor = area_info.get("environment", "").lower() == "outdoor"
            logger.info(f"[COORDS MATCH] info.json Sensor '{s_name}' -> Zone {area_key} at ({tx}, {ty}, {tz})")
            return tx, ty, tz, area_key, is_outdoor

    # 3. Check areas directly
    for area_key, area_data in MAP_DATA.get("areas", {}).items():
        poi = str(area_data.get("poi_name", "")).lower()
        if area_key.lower() == query_str or poi == query_str:
            coords = area_data.get("coordinates", {})
            tx = float(coords.get("x", 5.0))
            ty = float(coords.get("y", 1.0))
            tz = float(coords.get("z", 0.0))
            is_outdoor = area_data.get("environment", "").lower() == "outdoor"
            logger.info(f"[COORDS MATCH] info.json Area '{area_key}' -> ({tx}, {ty}, {tz})")
            return tx, ty, tz, area_key, is_outdoor

    logger.warning(f"[COORDS FALLBACK] Query '{alarm_query}' defaulted to Zone C (5.0, 1.0, 0.0)")
    return 5.0, 1.0, 0.0, "C", False


def select_best_robot(connected_robots: list, target_x: float, target_y: float, target_z: float = 0.0,
                      is_outdoor: bool = False, incident_type: str = "ground_intrusion"):
    """
    Multi-Factor & Constrained Solver:
    Filters robots by status, battery threshold, and environment compatibility,
    then scores eligible candidates using a normalized cost function (Distance 65%, Battery 35%).
    """
    registered_robots = MAP_DATA.get("deployed_assets", {}).get("robots", {})
    eligible_candidates = []
    rejections = []

    logger.info(f"[SOLVER START] Target: ({target_x}, {target_y}, {target_z}) | Outdoor: {is_outdoor} | Incident: {incident_type}")

    for r in connected_robots:
        r_id = str(r.get("id"))
        status = str(r.get("status", "")).strip().lower()
        raw_model = str(r.get("model", "")).lower()

        # Model matching against registry
        matched_model = raw_model
        for key in registered_robots.keys():
            if key in raw_model:
                matched_model = key
                break

        static_info = registered_robots.get(matched_model, {})
        r_type = str(r.get("type", static_info.get("type", "dog"))).lower()
        battery = int(r.get("battery", 100))

        # Constraint 1: Operational Status
        if status not in ["idle", "connected", "online"]:
            rejections.append(f"Robot {r_id} ({matched_model}): Disqualified (status is '{r.get('status')}', must be Idle)")
            continue

        if not static_info.get("operational", True):
            rejections.append(f"Robot {r_id} ({matched_model}): Disqualified (marked non-operational in registry)")
            continue

        # Constraint 2: Safe Battery Threshold
        if battery < MIN_BATTERY_LEVEL:
            rejections.append(f"Robot {r_id} ({matched_model}): Disqualified (battery {battery}% < {MIN_BATTERY_LEVEL}%)")
            continue

        # Constraint 3: Environmental & Mobility Suitability
        specs = CAPABILITY_MAP.get(r_type, {"indoor": True, "outdoor": True, "max_z": 1.0})
        if is_outdoor and not specs["outdoor"]:
            rejections.append(f"Robot {r_id} ({matched_model}): Disqualified (ground rover cannot operate outdoors)")
            continue
        if not is_outdoor and not specs["indoor"]:
            rejections.append(f"Robot {r_id} ({matched_model}): Disqualified (aerial drone cannot enter indoor zone)")
            continue

        # Constraint 4: Incident Type Capability Preference
        capabilities = static_info.get("capabilities", [])
        if incident_type == "aerial_surveillance" and "indoor_flight" not in capabilities and "outdoor_flight" not in capabilities:
            rejections.append(f"Robot {r_id} ({matched_model}): Lacks flight capabilities for aerial surveillance incident")
            continue

        # Calculate 3D Euclidean Distance
        pos = r.get("position", {})
        if isinstance(pos, dict):
            rx = float(pos.get("x", pos.get("X", r.get("x", 0.0))))
            ry = float(pos.get("y", pos.get("Y", r.get("y", 0.0))))
            rz = float(pos.get("z", pos.get("Z", r.get("z", 0.0))))
        else:
            rx = float(r.get("x", 0.0))
            ry = float(r.get("y", 0.0))
            rz = float(r.get("z", 0.0))

        dist = math.sqrt((rx - target_x)**2 + (ry - target_y)**2 + (rz - target_z)**2)

        eligible_candidates.append({
            "robot_id": r_id,
            "model": matched_model,
            "name": r.get("name", f"{matched_model} {r_id}"),
            "type": r_type,
            "battery": battery,
            "distance": dist,
            "capabilities": capabilities,
            "current_zone": r.get("current_zone", "Unknown")
        })

    if not eligible_candidates:
        logger.warning(f"[SOLVER REJECTION] No candidates eligible. Rejections: {rejections}")
        return None, rejections

    # Multi-Factor Cost Scoring
    max_dist = max(c["distance"] for c in eligible_candidates)
    if max_dist == 0:
        max_dist = 1.0

    for cand in eligible_candidates:
        norm_dist = cand["distance"] / max_dist
        norm_batt_penalty = (100.0 - cand["battery"]) / 100.0
        cand["composite_score"] = (W_DISTANCE * norm_dist) + (W_BATTERY * norm_batt_penalty)
        cand["distance"] = round(cand["distance"], 2)
        cand["score"] = round(cand["composite_score"], 3)

    # Sort ascending: lowest cost = highest rank
    eligible_candidates.sort(key=lambda c: c["composite_score"])
    best = eligible_candidates[0]

    logger.info(f"[SOLVER WINNER] Best robot: {best['robot_id']} ({best['model']}) | Cost Score: {best['score']}")
    return best, rejections


class RobotAssistant:
    def __init__(self):
        self.client = AsyncOpenAI()

        # Build topological summary
        zones_summary = []
        for zone, details in MAP_DATA.get("areas", {}).items():
            poi = details.get("poi_name", zone)
            desc = details.get("description", "")
            env = details.get("environment", "indoor")
            zones_summary.append(f"- Zone {zone} (POI: '{poi}', Env: {env}): {desc}")
        zones_context = "\n".join(zones_summary)

        # Build fleet summary
        fleet_summary = []
        for robot_key, rdata in MAP_DATA.get("deployed_assets", {}).get("robots", {}).items():
            model = rdata.get("model", robot_key)
            r_type = rdata.get("type", "unknown")
            operational = rdata.get("operational", True)
            caps = ", ".join(rdata.get("capabilities", []))
            fleet_summary.append(f"- Model '{model}' ({r_type}): Operational={operational} | Caps: [{caps}]")
        fleet_context = "\n".join(fleet_summary)

        if not cl.user_session.get("history"):
            cl.user_session.set("history", [
                {
                    "role": "system",
                    "content": (
                        "You are an autonomous Multi-Robot Surveillance Assistant operating via the TIM REST API.\n"
                        "You monitor and control deployed robots and sensors.\n\n"
                        f"AVAILABLE FLEET & STATUS:\n{fleet_context}\n\n"
                        f"FACILITY TOPOLOGY & ZONES:\n{zones_context}\n\n"
                        "OPERATIONAL RULES & SUPERVISORY MODES:\n"
                        "1. Default mode is HITL (Human-In-The-Loop).\n"
                        "2. MODE SWITCHING:\n"
                        "   - If the operator asks to switch to autonomous mode, ask for confirmation unless already confirmed.\n"
                        "   - When confirmed, call `set_operation_mode(mode='autonomous')`.\n"
                        "3. DISPATCH DECISION MAKING:\n"
                        "   - NEVER calculate distances or choose robots based on mental arithmetic.\n"
                        "   - Call `recommend_best_robot` to run the multi-factor constraint and cost optimization algorithm.\n"
                        "   - Present the chosen robot, distance, battery level, suitability score, and any disqualified units to the operator.\n"
                        "4. STRICT PARAMETER USAGE FOR `send_robots_to_area`:\n"
                        "   - `robot_id`: Must be the pure numeric ID string (e.g. '1234', '5678').\n"
                        "   - `model`: Must be the lowercase model key ('go2', 'dronus', 'dji', 'robox').\n"
                        "   - `target_poi`: Must be the exact POI or alarm name (e.g. 'alrm03', 'xingtera_01', 'area_B').\n"
                        "   - If the operator replies 'send it', 'deploy', or 'yes', take the recommended robot's numeric ID, model, and target_poi and call `send_robots_to_area` immediately.\n"
                        "5. TELEMETRY & STATUS:\n"
                        "   - When an operator asks for status, telemetry, alarms, or whether a robot was deployed, use the latest system state and conversation history.\n"
                        "   - If current confirmation is needed, call `retrieve_system_state`.\n"
                        "   - Report both the calculated Zone name and coordinates.\n"
                        "6. AUTONOMOUS BACKGROUND ACTIONS:\n"
                        "   - The background autonomous worker may append messages describing actions it already performed. Treat those messages as authoritative execution history.\n"
                        "   - If history says a robot was automatically dispatched, do not respond as if deployment is only a future possibility. Report that it was already deployed and, when useful, verify with `retrieve_system_state`.\n"
                        "Provide concise, operational, and professional answers."
                    )
                }
            ])

    async def handle_message(self, user_input):
        history = cl.user_session.get("history")
        history.append({"role": "user", "content": user_input})

        msg = cl.Message(content="")

        stream = await self.client.responses.create(
            model="gpt-4o",
            input=history,
            tools=[
                {
                    "type": "function",
                    "name": "recommend_best_robot",
                    "description": "Calculates the best robot using multi-factor constraint evaluation (battery >= 25%, indoor/outdoor suitability, status Idle, and composite distance/battery scoring).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "alarm_name": {
                                "type": "string",
                                "description": "The name or ID of the alarm or target zone (e.g. 'alrm03', '23', 'area_C')"
                            },
                            "incident_type": {
                                "type": "string",
                                "enum": ["ground_intrusion", "aerial_surveillance", "general_patrol"],
                                "description": "Type of event to tailor robot selection",
                                "default": "ground_intrusion"
                            }
                        },
                        "required": ["alarm_name"]
                    }
                },
                {
                    "type": "function",
                    "name": "set_operation_mode",
                    "description": "Switches system supervisory mode between 'autonomous' and 'hitl'.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string",
                                "enum": ["autonomous", "hitl"],
                                "description": "Target mode to set"
                            }
                        },
                        "required": ["mode"]
                    }
                },
                {
                    "type": "function",
                    "name": "send_robots_to_area",
                    "description": "Sends a target robot to a specific area or point of interest (POI).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "robot_id": {
                                "type": "string",
                                "enum": ["1234", "5678", "9901", "7702"],
                                "description": "Canonical numeric robot ID ('1234' for Go2, '5678' for Dronus)"
                            },
                            "model": {
                                "type": "string",
                                "enum": ["go2", "dronus", "dji", "robox"],
                                "description": "Canonical robot model key"
                            },
                            "target_poi": {
                                "type": "string",
                                "enum": ["alrm03", "xingtera_01", "area_A", "area_B", "corridor_central", "area_D", "area_F", "area_G", "area_H"],
                                "description": "Destination point of interest or alarm name"
                            }
                        },
                        "required": ["robot_id", "model", "target_poi"]
                    }
                },
                {
                    "type": "function",
                    "name": "retrieve_system_state",
                    "description": "Retrieves current status, position, semantic zone, and battery of connected robots and active alarms",
                    "parameters": {
                        "type": "object",
                        "properties": {}
                    }
                },
                {
                    "type": "function",
                    "name": "deactivate_alarm",
                    "description": "Deactivates a triggered alarm in the orchestrator",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "model": {
                                "type": "string",
                                "description": "Sensor model (e.g., 'shelly', 'xingtera')"
                            },
                            "alarm_id": {
                                "type": "string",
                                "description": "The unique alarm identifier"
                            }
                        },
                        "required": ["model", "alarm_id"]
                    }
                }
            ],
            stream=True
        )

        full_ai_response = ""
        final_tool_calls = {}
        active_steps = {}
        tool_outputs = []

        async for event in stream:
            if event.type == "response.created":
                pass
            elif event.type == "response.failed":
                await cl.Message(content=f"Error: {event.error.message}").send()
                break
            elif event.type == "response.output_text.delta":
                if not msg.id:
                    await msg.send()
                full_ai_response += event.delta
                await msg.stream_token(event.delta)
            elif event.type == "response.text.done":
                await msg.update()
            elif event.type == "response.output_item.added":
                if event.item.type == "function_call":
                    idx = event.output_index
                    tool_name = event.item.name
                    step = cl.Step(name=tool_name, type="tool")
                    step.language = "json"
                    await step.send()
                    final_tool_calls[idx] = event.item
                    active_steps[idx] = step
            elif event.type == "response.function_call_arguments.delta":
                idx = event.output_index
                if idx in final_tool_calls:
                    if final_tool_calls[idx].arguments is None:
                        final_tool_calls[idx].arguments = ""
                    final_tool_calls[idx].arguments += event.delta
                if idx in active_steps:
                    if active_steps[idx].input is None:
                        active_steps[idx].input = ""
                    await active_steps[idx].stream_token(event.delta, is_input=True)
            elif event.type == "response.function_call_arguments.done":
                idx = event.output_index
                tool_call = final_tool_calls[idx]
                step = active_steps[idx]
                args_str = event.arguments or "{}"

                try:
                    parsed_args = json.loads(args_str)
                except json.JSONDecodeError:
                    parsed_args = {}

                result = await cl.make_async(self.execute_rest_command)(tool_call.name, parsed_args)
                step.output = json.dumps(result, indent=2)
                await step.update()

                history.append({
                    "type": "function_call",
                    "call_id": tool_call.call_id,
                    "name": tool_call.name,
                    "arguments": args_str
                })
                history.append({
                    "type": "function_call_output",
                    "call_id": tool_call.call_id,
                    "output": json.dumps(result)
                })
                tool_outputs.append({
                    "call_id": tool_call.call_id,
                    "output": result
                })
            elif event.type == "response.completed":
                if msg.id:
                    await msg.update()
                break

        if full_ai_response:
            history.append({"role": "assistant", "content": full_ai_response})

        if tool_outputs:
            follow_up_stream = await self.client.responses.create(
                model="gpt-4o",
                input=history,
                tools=[],
                stream=True
            )
            tool_function_result_msg = ""
            async for event in follow_up_stream:
                if event.type == "response.failed":
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
                    break

            if tool_function_result_msg:
                history.append({"role": "assistant", "content": tool_function_result_msg})

        cl.user_session.set("history", history)

    def execute_rest_command(self, tool_name, args):
        """Dispatcher that invokes local algorithms and REST methods."""
        try:
            if hasattr(self, tool_name):
                method = getattr(self, tool_name)
                logger.info(f"Executing tool: {tool_name} with args: {args}")
                return method(**args)
            else:
                error_msg = f"Error: Command '{tool_name}' not implemented."
                logger.error(error_msg)
                return {"error": error_msg}
        except Exception as e:
            error_msg = f"Exception executing {tool_name}: {str(e)}"
            logger.error(error_msg)
            return {"error": error_msg}

    # --- Updated Multi-Factor Recommendation Tool ---

    def recommend_best_robot(self, alarm_name: str, incident_type: str = "ground_intrusion"):
        """Evaluates operational readiness, constraints, and composite cost."""
        state = self.retrieve_system_state()
        connected = state.get("connected_robots", [])
        active_alarms = state.get("active_alarms", [])

        if not connected:
            return {"error": "No connected robots available for assignment."}

        tx, ty, tz, area_key, is_outdoor = get_alarm_target_coordinates(alarm_name, active_alarms)

        best, rejections = select_best_robot(
            connected_robots=connected,
            target_x=tx,
            target_y=ty,
            target_z=tz,
            is_outdoor=is_outdoor,
            incident_type=incident_type
        )

        if not best:
            return {
                "error": "No operational robots satisfy deployment constraints.",
                "disqualified_units": rejections
            }

        return {
            "recommended_robot_id": best["robot_id"],
            "model": best["model"],
            "name": best["name"],
            "distance_meters": best["distance"],
            "battery_percent": best["battery"],
            "composite_cost_score": best["score"],
            "target_zone": area_key,
            "target_coordinates": {"x": tx, "y": ty, "z": tz},
            "environment": "outdoor" if is_outdoor else "indoor",
            "target_poi": alarm_name,
            "disqualified_units": rejections,
            "reasoning": (
                f"Selected {best['name']} (ID: {best['robot_id']}, Model: {best['model'].upper()}) "
                f"with score {best['score']} (Distance: {best['distance']}m, Battery: {best['battery']}%). "
                f"Candidate passed all constraints for {('Outdoor' if is_outdoor else 'Indoor')} Zone {area_key}."
            )
        }

    # --- Mode Controller Tool ---

    def set_operation_mode(self, mode: str):
        is_auto = (mode.lower() == "autonomous")
        cl.user_session.set("autonomous_mode", is_auto)
        logger.info(f"Mode switched via tool call to: {'AUTONOMOUS' if is_auto else 'HITL'}")
        return {"status": "success", "current_mode": "autonomous" if is_auto else "hitl"}

    # --- TIM REST API ENDPOINTS ---

    def retrieve_system_state(self):
        try:
            robots_resp = requests.get(f"{TIM_BASE_URL}/sec/multirobot/robot/connected", headers=TIM_HEADERS, timeout=4)
            alarms_resp = requests.get(f"{TIM_BASE_URL}/sec/orchestrator/alarm", headers=TIM_HEADERS, timeout=4)

            robots = robots_resp.json() if robots_resp.status_code == 200 else []
            alarms = alarms_resp.json() if alarms_resp.status_code == 200 else []

            # Enrich robots with topological zone
            for r in robots:
                pos = r.get("position", {})
                if isinstance(pos, dict):
                    rx = float(pos.get("x", pos.get("X", r.get("x", 0.0))))
                    ry = float(pos.get("y", pos.get("Y", r.get("y", 0.0))))
                else:
                    rx = float(r.get("x", 0.0))
                    ry = float(r.get("y", 0.0))

                r["current_zone"] = get_zone_from_coords(rx, ry)

            # Enrich alarms with assigned zone
            sensor_db = MAP_DATA.get("deployed_assets", {}).get("sensors", {})
            for a in alarms:
                name = a.get("name", "")
                if name in sensor_db:
                    zone_key = sensor_db[name].get("assigned_area", "")
                    zone_desc = MAP_DATA.get("areas", {}).get(zone_key, {}).get("description", "")
                    a["assigned_zone"] = f"Zone {zone_key} ({zone_desc})" if zone_desc else f"Zone {zone_key}"

            return {"connected_robots": robots, "active_alarms": alarms}
        except Exception as e:
            return {"error": f"Failed to reach TIM API: {str(e)}"}

    def send_robots_to_area(self, robot_id: str, target_poi: str = "alrm03", model: str = "go2"):
        """Sends a navigation command to a target robot with resilient payload handling."""
        model_clean = "go2"
        for m in ["go2", "dronus", "dji", "robox"]:
            if m in str(model).lower():
                model_clean = m
                break

        digits = "".join([c for c in str(robot_id) if c.isdigit()])
        robot_id_clean = digits if digits else str(robot_id).strip()

        poi_clean = str(target_poi).strip()
        if "alrm03" in poi_clean.lower() or poi_clean.lower() == "c":
            poi_clean = "alrm03"

        endpoint = f"{TIM_BASE_URL}/sec/{model_clean}/command/{robot_id_clean}"
        payload = {
            "command": "go-to-poi",
            "parameters": [{"key": 0, "value": poi_clean}]
        }

        try:
            logger.info(f"POST {endpoint} with payload {payload}")
            resp = requests.post(endpoint, headers=TIM_HEADERS, json=payload, timeout=5)
            logger.info(f"Gateway Response Code: {resp.status_code}, Body: {resp.text}")

            if resp.status_code in [200, 201, 202]:
                try:
                    return resp.json()
                except Exception:
                    return {"status": "accepted", "message": resp.text}
            else:
                return {
                    "error": f"Gateway returned status {resp.status_code}",
                    "details": resp.text
                }
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return {"error": f"Failed to send command to {model_clean}/{robot_id_clean}: {str(e)}"}

    def deactivate_alarm(self, model: str, alarm_id: str):
        endpoint = f"{TIM_BASE_URL}/sec/orchestrator/alarm/deactive/{model}/{alarm_id}"
        try:
            resp = requests.post(endpoint, headers=TIM_HEADERS, timeout=4)
            if resp.status_code not in [200, 201, 202]:
                resp = requests.get(endpoint, headers=TIM_HEADERS, timeout=4)

            if resp.status_code in [200, 201, 202]:
                try:
                    return resp.json()
                except Exception:
                    return {"status": "success", "message": resp.text}
            return {"status": "not_cleared", "details": resp.text}
        except Exception as e:
            return {"error": f"Failed to deactivate alarm: {str(e)}"}


# --- Autonomous Background Polling Worker ---

async def background_alarm_monitor(agent: RobotAssistant):
    """
    Continuously monitors active alarms while Fully Autonomous mode is enabled.

    Flow:
    1. Retrieve current alarms and fleet state.
    2. Select the best eligible robot using the constrained solver.
    3. Dispatch the selected robot through the TIM REST API.
    4. Verify from telemetry that the robot is navigating to the target.
    5. Clear the alarm only after deployment is confirmed.
    6. Store the autonomous action in chat history so the LLM knows what happened.
    """
    logger.info("Background alarm monitor worker started.")
    handled_alarms = set()

    while True:
        try:
            is_autonomous = cl.user_session.get("autonomous_mode", False)

            if is_autonomous:
                system_state = await cl.make_async(agent.retrieve_system_state)()

                if "error" in system_state:
                    logger.error(
                        f"[AUTONOMOUS STATE ERROR] Could not retrieve system state: "
                        f"{system_state.get('error')}"
                    )
                    await asyncio.sleep(5)
                    continue

                active_alarms = system_state.get("active_alarms", [])

                if isinstance(active_alarms, list):
                    for alarm in active_alarms:
                        alarm_id = str(alarm.get("id"))

                        if alarm_id in handled_alarms:
                            continue

                        alarm_model = alarm.get("model", "shelly")
                        alarm_name = alarm.get("name", "alrm03")
                        alarm_zone = alarm.get("assigned_zone", "Monitored Zone")

                        await cl.Message(
                            content=(
                                f"🚨 **Autonomous Trigger**: Detected alarm `{alarm_id}` "
                                f"at `{alarm_name}` ({alarm_zone}). "
                                f"Running multi-factor solver..."
                            )
                        ).send()

                        connected_robots = system_state.get("connected_robots", [])

                        tx, ty, tz, area_key, is_outdoor = get_alarm_target_coordinates(
                            alarm_name,
                            active_alarms
                        )

                        best_match, rejections = select_best_robot(
                            connected_robots=connected_robots,
                            target_x=tx,
                            target_y=ty,
                            target_z=tz,
                            is_outdoor=is_outdoor,
                            incident_type="ground_intrusion"
                        )

                        if not best_match:
                            await cl.Message(
                                content=(
                                    f"⚠️ **Dispatch Warning**: No operational robot "
                                    f"satisfied constraints for alarm `{alarm_id}`.\n"
                                    + "\n".join([f"- {r}" for r in rejections])
                                )
                            ).send()
                            continue

                        chosen_robot_id = best_match["robot_id"]
                        chosen_robot_model = best_match["model"]
                        dist = best_match["distance"]
                        batt = best_match["battery"]
                        score = best_match["score"]

                        # -------------------------------------------------
                        # 1. Dispatch the selected robot
                        # -------------------------------------------------
                        dispatch_result = await cl.make_async(
                            agent.send_robots_to_area
                        )(
                            robot_id=chosen_robot_id,
                            target_poi=alarm_name,
                            model=chosen_robot_model
                        )

                        logger.info(
                            f"[AUTONOMOUS DISPATCH RESULT] {dispatch_result}"
                        )

                        # -------------------------------------------------
                        # 2. Check whether the REST dispatch failed
                        # -------------------------------------------------
                        if (
                            not isinstance(dispatch_result, dict)
                            or "error" in dispatch_result
                        ):
                            error_details = (
                                dispatch_result.get(
                                    "error",
                                    "Unknown dispatch error"
                                )
                                if isinstance(dispatch_result, dict)
                                else str(dispatch_result)
                            )

                            await cl.Message(
                                content=(
                                    f"❌ **Autonomous Dispatch Failed**\n\n"
                                    f"- **Robot:** `{chosen_robot_id}` "
                                    f"({chosen_robot_model.upper()})\n"
                                    f"- **Target:** `{alarm_name}`\n"
                                    f"- **Reason:** `{error_details}`\n\n"
                                    f"The alarm has **not** been cleared."
                                )
                            ).send()

                            logger.error(
                                f"[AUTONOMOUS DISPATCH FAILED] "
                                f"Robot={chosen_robot_id}, "
                                f"Target={alarm_name}, "
                                f"Result={dispatch_result}"
                            )
                            continue

                        # -------------------------------------------------
                        # 3. Verify the updated robot telemetry
                        # -------------------------------------------------
                        updated_state = await cl.make_async(
                            agent.retrieve_system_state
                        )()

                        if "error" in updated_state:
                            await cl.Message(
                                content=(
                                    f"⚠️ **Dispatch Verification Failed**\n\n"
                                    f"The command for robot `{chosen_robot_id}` was "
                                    f"accepted, but updated telemetry could not be "
                                    f"retrieved.\n\n"
                                    f"The alarm will remain active."
                                )
                            ).send()
                            logger.error(
                                f"[AUTONOMOUS VERIFY ERROR] "
                                f"{updated_state.get('error')}"
                            )
                            continue

                        updated_robot = None

                        for robot in updated_state.get("connected_robots", []):
                            if str(robot.get("id")) == str(chosen_robot_id):
                                updated_robot = robot
                                break

                        if not updated_robot:
                            await cl.Message(
                                content=(
                                    f"⚠️ **Dispatch Verification Failed**\n\n"
                                    f"The command was accepted, but robot "
                                    f"`{chosen_robot_id}` could not be found in "
                                    f"the updated telemetry.\n\n"
                                    f"The alarm will remain active."
                                )
                            ).send()
                            continue

                        robot_status = str(
                            updated_robot.get("status", "Unknown")
                        )
                        robot_target = updated_robot.get(
                            "target",
                            alarm_name
                        )

                        # -------------------------------------------------
                        # 4. Confirm that the robot is actually navigating
                        # -------------------------------------------------
                        dispatch_confirmed = (
                            "navigating" in robot_status.lower()
                            and str(robot_target).lower()
                            == str(alarm_name).lower()
                        )

                        if not dispatch_confirmed:
                            await cl.Message(
                                content=(
                                    f"⚠️ **Dispatch Not Confirmed**\n\n"
                                    f"- **Robot:** `{chosen_robot_id}` "
                                    f"({chosen_robot_model.upper()})\n"
                                    f"- **Returned Status:** `{robot_status}`\n"
                                    f"- **Returned Target:** `{robot_target}`\n"
                                    f"- **Expected Target:** `{alarm_name}`\n\n"
                                    f"The alarm will remain active."
                                )
                            ).send()

                            logger.warning(
                                f"[AUTONOMOUS DISPATCH NOT CONFIRMED] "
                                f"Robot={chosen_robot_id}, "
                                f"Status={robot_status}, "
                                f"Target={robot_target}"
                            )
                            continue

                        # -------------------------------------------------
                        # 5. Deployment confirmed -> clear the alarm
                        # -------------------------------------------------
                        clear_result = await cl.make_async(
                            agent.deactivate_alarm
                        )(
                            model=alarm_model,
                            alarm_id=alarm_id
                        )

                        logger.info(
                            f"[AUTONOMOUS ALARM CLEAR RESULT] {clear_result}"
                        )

                        clear_success = (
                            isinstance(clear_result, dict)
                            and "error" not in clear_result
                            and clear_result.get("status")
                            not in ["not_cleared", "not_found"]
                        )

                        # The robot has already been dispatched successfully.
                        # Mark this alarm as handled to prevent duplicate dispatches,
                        # even if alarm clearing fails.
                        handled_alarms.add(alarm_id)

                        # -------------------------------------------------
                        # 6. Store autonomous action in LLM conversation
                        # -------------------------------------------------
                        history = cl.user_session.get("history") or []

                        history.append({
                            "role": "assistant",
                            "content": (
                                f"Autonomous system event: Alarm {alarm_id} at "
                                f"{alarm_name} was detected while Fully Autonomous "
                                f"mode was active. Robot {chosen_robot_id} "
                                f"({chosen_robot_model.upper()}) was selected by the "
                                f"multi-factor solver and successfully dispatched to "
                                f"{alarm_name}. Telemetry confirmed status "
                                f"'{robot_status}' with target '{robot_target}'. "
                                + (
                                    f"Alarm {alarm_id} was successfully cleared."
                                    if clear_success
                                    else (
                                        f"Robot deployment succeeded, but alarm "
                                        f"{alarm_id} could not be confirmed as cleared."
                                    )
                                )
                            )
                        })

                        cl.user_session.set("history", history)

                        # -------------------------------------------------
                        # 7. Report the result in the Chainlit UI
                        # -------------------------------------------------
                        resolution_summary = (
                            f"✅ **Autonomous Resolution Complete:**\n"
                            f"- **Selected Robot:** `{chosen_robot_id}` "
                            f"({chosen_robot_model.upper()})\n"
                            f"- **Deployment:** ✅ Successfully dispatched\n"
                            f"- **Robot Status:** `{robot_status}`\n"
                            f"- **Target:** `{robot_target}`\n"
                            f"- **Euclidean Distance:** `{dist}m` to target\n"
                            f"- **Battery Level:** `{batt}%`\n"
                            f"- **Suitability Score:** `{score}` "
                            f"(Distance 65%, Battery 35%)\n"
                            f"- **Target POI:** `{alarm_name}` ({alarm_zone})\n"
                        )

                        if clear_success:
                            resolution_summary += (
                                f"- **Cleared Alarm:** `{alarm_id}`"
                            )
                        else:
                            resolution_summary += (
                                f"- **Alarm Clear:** ⚠️ Could not confirm clearing "
                                f"`{alarm_id}`"
                            )

                        if rejections:
                            resolution_summary += (
                                "\n\n**Disqualified Candidates:**\n"
                                + "\n".join([f"- {r}" for r in rejections])
                            )

                        await cl.Message(
                            content=resolution_summary
                        ).send()

        except Exception as e:
            logger.exception(
                f"[Background Monitor Error]: {e}"
            )

        await asyncio.sleep(5)


# --- Chainlit UI Handlers ---

@cl.on_chat_start
async def start():
    agent = RobotAssistant()
    cl.user_session.set("agent", agent)
    cl.user_session.set("autonomous_mode", False)

    await cl.ChatSettings(
        [
            Select(
                id="operation_mode",
                label="Supervisory Mode",
                values=["HITL (Human-in-the-Loop)", "Fully Autonomous"],
                initial_index=0,
            )
        ]
    ).send()

    asyncio.create_task(background_alarm_monitor(agent))

    await cl.Message(
        content=(
            "### TIM Multi-Robot Surveillance Assistant\n\n"
            "System is online and monitoring connected fleet assets.\n\n"
            "* **Supervisory Mode:** `HITL (Human-in-the-Loop)`\n"
            "* **Active Filters:** Battery ≥ 25%, Operational Status (Idle), Environmental Boundaries\n"
            "* **Dispatch Metric:** Multi-factor weighted score (65% Proximity, 35% Battery Reserve)\n"
            "* **Controls:** Switch to autonomous mode via chat or the settings panel."
        )
    ).send()


@cl.on_settings_update
async def setup_agent(settings):
    selected_mode = settings["operation_mode"]
    is_auto = "Autonomous" in selected_mode
    cl.user_session.set("autonomous_mode", is_auto)
    await cl.Message(content=f"⚙️ System mode updated to: **{selected_mode}**").send()


@cl.on_message
async def main(message: cl.Message):
    agent = cl.user_session.get("agent")
    await agent.handle_message(message.content)