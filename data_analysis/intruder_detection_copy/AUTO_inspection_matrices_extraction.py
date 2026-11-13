
import json
import re
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

INPUT_FILE = BASE_DIR / "raw_data/experiments_AUTO.json"
OUTPUT_FILE = BASE_DIR / "inspection_matrices/inspection_matrices_AUTO.json"

RAG_FILES = {
    "world_1": BASE_DIR / "../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": BASE_DIR / "../../llm_interface/config/RAG/RAG_map2.json"
}


# ============================================================
# LOAD WORLD AREAS AND CAMERA MAPPINGS
# ============================================================

world_areas = {}
camera_mappings = {}

for world, path in RAG_FILES.items():

    with open(path, "r", encoding="utf-8") as file:
        rag_data = json.load(file)

    world_areas[world] = list(rag_data["areas"].keys())

    camera_mappings[world] = {
        sensor.lower(): info["areas"]
        for sensor, info in rag_data["sensors"].items()
        if sensor.lower().startswith("camera_")
    }


# ============================================================
# LOAD AUTO RESULTS
# ============================================================

with open(INPUT_FILE, "r", encoding="utf-8") as file:
    data = json.load(file)


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def extract_function_data(message):

    match = re.search(
        r"name\s*-\s*(.*?)\s*arguments\s*-\s*(\{.*\})",
        message.get("function_call", ""),
        re.DOTALL
    )

    if not match:
        return None, {}

    try:
        return match.group(1).strip(), json.loads(match.group(2))
    except json.JSONDecodeError:
        return None, {}


def add_inspection(matrix, area, value):

    current = matrix["1"][area]

    if current == 0.0:
        matrix["1"][area] = value

    elif isinstance(current, list):
        current.append(value)

    else:
        matrix["1"][area] = [current, value]


# ============================================================
# EXTRACT INSPECTION MATRICES
# ============================================================

results = {}

for experiment_type, methods in data.items():

    for method, worlds in methods.items():

        for world, models in worlds.items():

            areas = world_areas[world]
            cameras = camera_mappings[world]

            for model, trials in models.items():

                for trial, runs in trials.items():

                    for run, run_data in runs.items():

                        sensor_sequence = run_data["sensor_sequence"]
                        conversation = run_data["conversation"]

                        # Find completion of sensor sequence
                        sensor_count = 0
                        completion_index = None

                        for i, message in enumerate(conversation):

                            if "data_orchestrator" in message:

                                sensor_count += 1

                                if sensor_count == len(sensor_sequence):
                                    completion_index = i
                                    break

                        # Initialize one inspection turn
                        matrix = {
                            "1": {
                                area: 0.0
                                for area in areas
                            }
                        }

                        # Process messages after sensor completion
                        if completion_index is not None:

                            for message in conversation[completion_index + 1:]:

                                function_name, arguments = extract_function_data(message)

                                # Camera inspection
                                if function_name == "get_camera_feed":

                                    for camera in arguments.get("cameras_names_list", []):

                                        for area in cameras.get(camera.lower(), []):

                                            if area in areas:
                                                add_inspection(matrix, area, 1.0)

                                # Robot deployment
                                elif function_name == "send_robots_to_area":

                                    for robot in arguments.get("robots_sequence", []):

                                        area = robot.get("area_to_reach")

                                        if area in areas:
                                            add_inspection(matrix, area, 10.0)

                        # Count distinct inspected areas
                        n_inspected = sum(
                            value != 0.0
                            for value in matrix["1"].values()
                        )

                        # Store results
                        results.setdefault(method, {}) \
                            .setdefault(experiment_type, {}) \
                            .setdefault(world, {}) \
                            .setdefault(trial, {})[run] = {

                                "sensor_sequence": sensor_sequence,

                                "intruder_final_area": run_data["intruder_final_area"],

                                "success": run_data["success"],

                                "matrix": matrix,

                                "n_inspected_areas": n_inspected,

                                "n_total_areas": len(areas)
                            }


# ============================================================
# SAVE RESULTS
# ============================================================

with open(OUTPUT_FILE, "w", encoding="utf-8") as file:

    json.dump(
        results,
        file,
        indent=4,
        ensure_ascii=False
    )

print(f"AUTO matrices saved to: {OUTPUT_FILE}")
