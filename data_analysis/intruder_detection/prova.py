
import json
import re
from pathlib import Path

import pandas as pd


# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent


FILES = [
    BASE_DIR / "experiments_HITL.json",
    # BASE_DIR / "experiments_AUTO.json"
]


# RAG filenames associated with each world
RAG_FILES = {
    "world_1": BASE_DIR / "../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": BASE_DIR / "../../llm_interface/config/RAG/RAG_map2.json"
}

# Mapping between cameras and inspected areas.
# Modify according to your environment.
camera_to_area = {
    "camera_1": "A",
    "camera_2": "B",
    "camera_3": "C",
    "camera_4": "D",
}


# ============================================================
# 1. FIND ALL RUNS RECURSIVELY
# ============================================================

def find_runs(data, path=()):
    """
    Recursively searches for dictionaries containing
    sensor_sequence and conversation.
    """

    if isinstance(data, dict):

        if (
            "sensor_sequence" in data
            and "conversation" in data
        ):
            yield path, data
            return

        for key, value in data.items():
            yield from find_runs(value, path + (key,))

    elif isinstance(data, list):

        for i, value in enumerate(data):
            yield from find_runs(value, path + (i,))


# ============================================================
# 2. IDENTIFY REAL USER MESSAGES
# ============================================================

def is_real_user_message(message):
    """
    Distinguishes actual user messages from robot notifications.
    """

    if not isinstance(message, dict):
        return False

    if "user" not in message:
        return False

    content = message["user"]

    if not isinstance(content, str):
        return False

    # Robot notification, e.g.:
    # turtlebot3_2: Goal reached. Area F: Intruder identified

    if re.match(
        r"^\s*turtlebot\d+_\d+\s*:",
        content,
        re.IGNORECASE
    ):
        return False

    return True


# ============================================================
# 3. EXTRACT AREAS FROM FUNCTION CALLS
# ============================================================

def extract_function_data(message):
    """
    Extracts function name and arguments from function_call.
    """

    if not isinstance(message, dict):
        return None, None

    function_call = message.get("function_call")

    if not isinstance(function_call, str):
        return None, None

    match = re.search(
        r"name\s*-\s*(.*?)\s*arguments\s*-\s*(\{.*\})",
        function_call,
        re.DOTALL
    )

    if not match:
        return None, None

    function_name = match.group(1).strip()
    arguments_string = match.group(2).strip()

    try:
        arguments = json.loads(arguments_string)
    except json.JSONDecodeError:
        return None, None

    return function_name, arguments


# ============================================================
# 4. PROCESS ONE RUN
# ============================================================



def process_run(run_data, world, camera_mappings):

    conversation = run_data["conversation"]
    sensor_sequence = run_data["sensor_sequence"]

    expected_sensors = len(sensor_sequence)

    # ------------------------------------------------
    # 1. Find sensor completion
    # ------------------------------------------------

    orchestrator_count = 0
    sensor_completion_index = None

    for i, message in enumerate(conversation):

        if not isinstance(message, dict):
            continue

        if "data_orchestrator" in message:
            orchestrator_count += 1

            if orchestrator_count >= expected_sensors:
                sensor_completion_index = i
                break

    if sensor_completion_index is None:
        print("WARNING: Sensor sequence not completed")
        return {}

    post_sensor_conversation = conversation[
        sensor_completion_index + 1:
    ]

    # ------------------------------------------------
    # 2. Identify whether there are real user messages
    # ------------------------------------------------

    has_real_user = any(
        is_real_user_message(message)
        for message in post_sensor_conversation
    )

    # If no real user message exists, consider the
    # conversation until the last assistant response.

    if not has_real_user:

        last_assistant_index = None

        for i, message in enumerate(post_sensor_conversation):

            if isinstance(message, dict) and "assistant" in message:
                last_assistant_index = i

        if last_assistant_index is not None:
            relevant_conversation = post_sensor_conversation[
                :last_assistant_index + 1
            ]
        else:
            relevant_conversation = post_sensor_conversation

    else:
        relevant_conversation = post_sensor_conversation

    # ------------------------------------------------
    # 3. Process inspections
    # ------------------------------------------------

    current_turn = 1
    inspections = {}

    inspections[current_turn] = {}

    camera_mapping = camera_mappings.get(world, {})

    for message in relevant_conversation:

        if not isinstance(message, dict):
            continue

        # --------------------------------------------
        # Real user message: new LLM turn
        # --------------------------------------------

        if is_real_user_message(message):

            if has_real_user:
                current_turn += 1
                inspections.setdefault(current_turn, {})

            continue

        # --------------------------------------------
        # Ignore assistant messages
        # --------------------------------------------

        # --------------------------------------------
        # Extract function calls
        # --------------------------------------------

        function_name, arguments = extract_function_data(message)

        if function_name is None:
            continue

        inspections.setdefault(current_turn, {})

        # --------------------------------------------
        # Camera inspection
        # --------------------------------------------

        if function_name == "get_camera_feed":

            cameras = arguments.get(
                "cameras_names_list", []
            )

            for camera in cameras:

                areas = camera_mapping.get(
                    camera.lower(), []
                )

                if isinstance(areas, str):
                    areas = [areas]

                for area in areas:

                    inspections[current_turn][area] = max(
                        inspections[current_turn].get(area, 0),
                        0.5
                    )

        # --------------------------------------------
        # Robot deployment
        # --------------------------------------------

        elif function_name == "send_robots_to_area":

            robots = arguments.get(
                "robots_sequence", []
            )

            for robot in robots:

                area = robot.get("area_to_reach")

                if area is None:
                    continue

                inspections[current_turn][area] = max(
                    inspections[current_turn].get(area, 0),
                    1
                )

    return inspections

# ============================================================
# 5. CONVERT INSPECTIONS INTO A MATRIX
# ============================================================

def inspections_to_matrix(inspections, all_areas):

    all_turns = sorted(inspections.keys())

    matrix = pd.DataFrame(
        0.0,
        index=all_turns,
        columns=all_areas
    )

    for turn, areas in inspections.items():

        for area, value in areas.items():

            if area in matrix.columns:
                matrix.loc[turn, area] = value

            else:
                print(
                    f"WARNING: Area '{area}' not found "
                    f"in the RAG configuration."
                )

    matrix.index.name = "LLM_turn"

    return matrix
def load_camera_mappings():

    camera_mappings = {}
    world_areas = {}

    for world, rag_path in RAG_FILES.items():

        with open(rag_path, "r", encoding="utf-8") as file:
            rag_data = json.load(file)

        sensors = rag_data.get("sensors", {})

        camera_mappings[world] = {}

        for sensor_name, sensor_info in sensors.items():

            if sensor_name.lower().startswith("camera_"):

                areas = sensor_info.get("areas", [])

                camera_mappings[world][sensor_name.lower()] = areas

        # Extract ALL areas from the RAG JSON
        world_areas[world] = sorted(
            rag_data.get("areas", {}).keys()
        )

        print(f"\nCamera mapping loaded for {world}:")
        print(camera_mappings[world])

        print(f"All areas for {world}:")
        print(world_areas[world])

    return camera_mappings, world_areas

# ============================================================
# 6. LOAD CAMERA MAPPINGS
# ============================================================

camera_mappings, world_areas = load_camera_mappings()


# ============================================================
# 7. PROCESS EXPERIMENT FILES
# ============================================================

all_matrices = {}

total_matrices = 0
empty_matrices = 0
empty_matrix_runs = []


for filename in FILES:

    print(f"\nProcessing {filename}...")

    with open(filename, "r", encoding="utf-8") as file:
        data = json.load(file)

    method = Path(filename).stem.replace(
        "experiments_", ""
    )

    all_matrices[method] = {}

    for run_path, run_data in find_runs(data):

        # Identify world from JSON hierarchy
        world = next(
            (key for key in run_path if str(key).startswith("world_")),
            None
        )

        if world is None:
            print(f"WARNING: World not identified: {run_path}")
            continue

        if world not in camera_mappings:
            print(f"WARNING: No camera mapping for {world}")
            continue

        # Process run using the corresponding camera mapping
        inspections = process_run(
            run_data,
            world,
            camera_mappings
        )

        matrix = inspections_to_matrix(
            inspections,
            world_areas[world]
        )

        # Reconstruct original JSON hierarchy
        current = all_matrices[method]

        for key in run_path[:-1]:
            current = current.setdefault(key, {})

        run_name = run_path[-1]

        current[run_name] = {
            "sensor_sequence": run_data["sensor_sequence"],
            "matrix": matrix
        }

        # Count empty matrices
        total_matrices += 1

        is_empty = matrix.empty or (matrix == 0).all().all()

        if is_empty:
            empty_matrices += 1

            run_identifier = "/".join(
                [method] + [str(key) for key in run_path]
            )

            empty_matrix_runs.append(run_identifier)

            print(f"WARNING: Empty matrix - {run_identifier}")

        # Save individual matrix
        output_dir = (
            Path("inspection_matrices") / method
        )

        output_dir.mkdir(parents=True, exist_ok=True)

        safe_name = "_".join(map(str, run_path))

        matrix.to_csv(
            output_dir / f"{safe_name}.csv"
        )

        print(f"\nMethod: {method}")
        print(f"World: {world}")
        print(f"Run path: {run_path}")
        print(matrix)


# ============================================================
# 8. FINAL STATISTICS
# ============================================================

percentage_empty = (
    100 * empty_matrices / total_matrices
    if total_matrices > 0 else 0
)

print("\n" + "=" * 50)
print("FINAL STATISTICS")
print("=" * 50)

print(f"Total matrices generated: {total_matrices}")
print(f"Empty matrices: {empty_matrices}")
print(f"Non-empty matrices: {total_matrices - empty_matrices}")
print(f"Percentage of empty matrices: {percentage_empty:.2f}%")

print("\n" + "=" * 60)
print("EMPTY MATRICES DETAILS")
print("=" * 60)

if empty_matrix_runs:

    print(f"\nFound {len(empty_matrix_runs)} empty matrices:\n")

    for i, run in enumerate(empty_matrix_runs, 1):
        print(f"{i}. {run}")

else:
    print("\nNo empty matrices found.")

print("=" * 60)
print("\nProcessing completed.")