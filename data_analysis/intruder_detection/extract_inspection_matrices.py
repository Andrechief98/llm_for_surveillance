
import json
import re
from pathlib import Path
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent


FILES = [
    BASE_DIR / "experiments_HITL.json",
    BASE_DIR / "experiments_AUTO.json",
    BASE_DIR / "experiments_ILP.json"
]


# RAG filenames associated with each world
RAG_FILES = {
    "world_1": BASE_DIR / "../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": BASE_DIR / "../../llm_interface/config/RAG/RAG_map2.json"
}


OUTPUT_FILE = BASE_DIR / "inspection_matrices.json"


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
            and (
                "conversation" in data
                or "actions" in data
            )
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


def get_inspected_areas(matrix):

    array = matrix.to_numpy()

    # Sum along LLM turns
    area_vector = np.sum(array, axis=0)

    # Binary vector
    binary_vector = (area_vector > 0).astype(int)

    # Number of inspected areas
    n_inspected = int(np.sum(binary_vector))

    return binary_vector, n_inspected

def process_ilp_run(run_data, world):

    actions = run_data.get("actions", [])

    inspections = {}

    for turn, turn_actions in enumerate(actions, start=1):

        inspections[turn] = {}

        if not isinstance(turn_actions, dict):
            continue

        deployments = turn_actions.get("deploy", {})

        if not isinstance(deployments, dict):
            continue

        for robot, area in deployments.items():

            if not isinstance(area, str):
                continue

            inspections[turn][area] = 1

    return inspections
def find_stored_runs(data, path=()):

    if isinstance(data, dict):

        if (
            "sensor_sequence" in data
            and "success" in data
        ):
            yield path, data
            return

        if (
            "sensor_sequence" in data
            and "matrix" in data
        ):
            yield path, data
            return

        for key, value in data.items():
            yield from find_stored_runs(
                value, path + (key,)
            )

    elif isinstance(data, list):

        for i, value in enumerate(data):
            yield from find_stored_runs(
                value, path + (i,)
            )


def print_matrix(
    all_matrices,
    method,
    mode,
    world,
    model,
    trial,
    run
):

    try:
        matrix = (
            all_matrices[method]
            [mode]
            [method]
            [world]
            [model]
            [trial]
            [run]["matrix"]
        )

    except KeyError:
        print("Matrix not found!")
        return None

    print("\n" + "=" * 60)
    print(f"Method: {method}")
    print(f"Mode: {mode}")
    print(f"World: {world}")
    print(f"Model: {model}")
    print(f"Trial: {trial}")
    print(f"Run: {run}")
    print("=" * 60)

    print(matrix.to_string())

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

        # print(f"\nCamera mapping loaded for {world}:")
        # print(camera_mappings[world])

        # print(f"All areas for {world}:")
        # print(world_areas[world])

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

        if method == "ILP":

            inspections = process_ilp_run(
                run_data,
                world
            )

        else:

            inspections = process_run(
                run_data,
                world,
                camera_mappings
            )

        matrix = inspections_to_matrix(
            inspections,
            world_areas[world]
        )


        # ------------------------------------------------
        # Verify that the intruder final area was inspected
        # ------------------------------------------------

        success_rate = run_data["success"]
        intruder_final_area = run_data["intruder_final_area"]

        if success_rate == 1 and intruder_final_area != "FP":

            if (intruder_final_area not in matrix.columns or matrix[intruder_final_area].sum() == 0):

                print("\nWARNING: Intruder final area NOT inspected!")

                print(f"Method: {method}")
                print(f"World: {world}")
                print(f"Run path: {run_path}")
                print(f"Intruder final area: {intruder_final_area}")

                print("\nInspection matrix:")
                print(matrix.to_string())

                print("-" * 60)

        # Reconstruct original JSON hierarchy
        current = all_matrices[method]

        for key in run_path[:-1]:
            current = current.setdefault(key, {})

        run_name = run_path[-1]

        # Extract success directly from JSON
        success_rate = run_data["success"]

        intruder_final_area = run_data["intruder_final_area"]

        binary_vector, n_inspected = get_inspected_areas(matrix)

        current[run_name] = {
            "sensor_sequence": run_data["sensor_sequence"],
            "intruder_final_area": intruder_final_area,
            "success_rate": success_rate,
            "matrix": matrix,
            "inspected_areas": binary_vector,
            "n_inspected_areas": n_inspected,
            "efficiency": None
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


        # print(f"\nMethod: {method}")
        # print(f"World: {world}")
        # print(f"Run path: {run_path}")
        # print(matrix)


print("\nRUNS LOADED PER METHOD")

for method, method_data in all_matrices.items():

    runs = list(find_stored_runs(method_data))

    print(f"{method}: {len(runs)} runs")

    # if method == "ILP":
    #     for run_path, run_data in runs[:5]:
    #         print(
    #             "ILP example:",
    #             run_path,
    #             "Sensors:",
    #             run_data["sensor_sequence"]
    #         )

# ============================================================
# 8. CALCULATE INSPECTION EFFICIENCY
# ============================================================

# ------------------------------------------------------------
# 8.1 Find minimum inspected areas for each world and sequence
# ------------------------------------------------------------

minimum_areas = {}

for method, method_data in all_matrices.items():

    for run_path, run_data in find_stored_runs(method_data):

        # Identify world
        world = next(
            (str(key) for key in run_path
             if str(key).startswith("world_")),
            None
        )

        if world is None:
            continue

        # Identify sensor sequence
        sequence_key = json.dumps(
            run_data["sensor_sequence"],
            sort_keys=True,
            ensure_ascii=False
        )

        # Group by world + sensor sequence
        intruder_final_area = run_data["intruder_final_area"]

        group_key = (
            world,
            sequence_key,
            intruder_final_area
        )
                
        # Consider only successful runs
        if run_data["success_rate"] == 1:

            n_inspected = run_data["n_inspected_areas"]

            if group_key not in minimum_areas:
                minimum_areas[group_key] = n_inspected

            else:
                minimum_areas[group_key] = min(
                    minimum_areas[group_key],
                    n_inspected
                )


# ------------------------------------------------------------
# 8.2 Calculate efficiency for each run
# ------------------------------------------------------------
# all_matrices
# │
# ├── HITL
# │   ├── experiment_type
# │   │   ├── world_1
# │   │   │   ├── model
# │   │   │   │   ├── trial_1
# │   │   │   │   │   ├── run_1
# │   │   │   │   │   ├── run_2
# │   │   │   │   │   └── ...
# │   │   │   │   └── trial_2
# │   │   │   └── ...
# │   │   └── world_2
# │   └── ...
# │
# ├── AUTO
# │   └── ...
# │
# └── ILP
#     ├── experiment_type
#     │   ├── world_1
#     │   │   ├── B30
#     │   │   │   ├── trial_1
#     │   │   │   │   ├── run_1
#     │   │   │   │   └── ...
#     │   │   └── B60
#     │   └── world_2
#     └── ...


# Each run is a dictionary
# {
#     "sensor_sequence": run_data["sensor_sequence"],

#     "intruder_final_area": intruder_final_area,

#     "success_rate": success_rate,

#     "matrix": matrix,

#     "inspected_areas": binary_vector,

#     "n_inspected_areas": n_inspected,

#     "efficiency": None
# }

for method, method_data in all_matrices.items():

    for run_path, run_data in find_stored_runs(method_data):

        # Identify world
        world = next(
            (str(key) for key in run_path
             if str(key).startswith("world_")),
            None
        )

        if world is None:
            continue

        sequence_key = json.dumps(
            run_data["sensor_sequence"],
            sort_keys=True,
            ensure_ascii=False
        )

        intruder_final_area = run_data["intruder_final_area"]

        group_key = (
            world,
            sequence_key,
            intruder_final_area
        )

        SR = run_data["success_rate"]

        A_inspected = run_data["n_inspected_areas"]

        N_total = len(world_areas[world])

        if group_key in minimum_areas and N_total > 0:

            A_min = minimum_areas[group_key]

            efficiency = round(SR * (
                1 - (A_inspected - A_min) / (N_total - A_min)
            ), 2)

        else:
            print("Error in computing efficiency metric")
            A_min = None
            efficiency = 0.0

        # Store results in the corresponding run
        run_data["min_inspected_areas"] = A_min
        run_data["efficiency"] = efficiency

        # Identify trial
        trial = next(
            (
                str(key) for key in run_path
                if str(key).startswith("trial_")
            ),
            None
        )

        # Identify run
        run = next(
            (
                str(key) for key in run_path
                if str(key).startswith("run_")
            ),
            None
        )

        # Sensor sequence
        sensor_sequence = run_data["sensor_sequence"]

        if method == "ILP":

            ilp_model = next(
                (
                    str(key) for key in run_path
                    if re.fullmatch(r"B\d+", str(key))
                ),
                None
            )

            method_label = f"ILP-{ilp_model}"

        else:
            method_label = method

        # print(
        #     f"Method: {method_label} | "
        #     f"World: {world} | "
        #     f"Trial: {trial} | "
        #     f"Run: {run} | "
        #     f"Sensor sequence: {sensor_sequence} | "
        #     f"Inspected: {A_inspected} | "
        #     f"Minimum: {A_min} | "
        #     f"Success: {SR} | "
        #     f"Efficiency: {efficiency:.3f}"
        # )



# ============================================================
# 8.3 SAVE MATRICES AND INDIVIDUAL RUN EFFICIENCIES
# ============================================================


def build_results_json(all_matrices):

    results = {}

    for method, method_data in all_matrices.items():

        for run_path, run_data in find_stored_runs(method_data):

            experiment_type = str(run_path[0])

            world = next(
                str(k) for k in run_path
                if str(k).startswith("world_")
            )

            trial = next(
                str(k) for k in run_path
                if str(k).startswith("trial_")
            )

            run = str(run_path[-1])

            # Model identification
            if method == "ILP":
                model = next(
                    str(k) for k in run_path
                    if re.fullmatch(r"B\d+", str(k))
                )
            else:
                model = next(
                    str(k) for k in run_path
                    if str(k) not in (
                        experiment_type,
                        method,
                        world,
                    )
                    and not str(k).startswith(
                        ("trial_", "run_")
                    )
                )

            matrix = run_data["matrix"]

            matrix_dict = {
                str(index): {
                    str(column): float(matrix.loc[index, column])
                    for column in matrix.columns
                }
                for index in matrix.index
            }

            run_result = {
                "sensor_sequence": run_data["sensor_sequence"],
                "matrix": matrix_dict,
                "n_inspected_areas": int(run_data["n_inspected_areas"]),
                "min_inspected_areas": run_data["min_inspected_areas"],
                "n_total_areas": len(world_areas[world]),
                "success_rate": float(run_data["success_rate"]),
                "efficiency": float(run_data["efficiency"])
            }

            current = (
                results
                .setdefault(experiment_type, {})
                .setdefault(method, {})
                .setdefault(world, {})
                .setdefault(model, {})
                .setdefault(trial, {})
            )

            # Use the original run identifier
            current[run] = run_result

    return results
results_to_save = build_results_json(all_matrices)

with open(OUTPUT_FILE, "w", encoding="utf-8") as file:

    json.dump(
        results_to_save,
        file,
        indent=4,
        ensure_ascii=False
    )

print(f"\nInspection results saved to: {OUTPUT_FILE}")


# ============================================================
# 9. PLOT EFFICIENCY VS NUMBER OF SENSORS
# ============================================================

# ------------------------------------------------------------
# 9.1 Collect efficiencies
# ------------------------------------------------------------

# Key:
# (method, experiment_type, ilp_model,
#  n_sensors, world, trial)

trial_world_efficiencies = {}

for method, method_data in all_matrices.items():

    for run_path, run_data in find_stored_runs(method_data):

        # Experiment type
        experiment_type = run_path[0]

        # Identify world
        world = next(
            (
                str(key) for key in run_path
                if str(key).startswith("world_")
            ),
            None
        )

        if world is None:
            continue

        # Identify trial
        trial = next(
            (
                str(key) for key in run_path
                if str(key).startswith("trial_")
            ),
            None
        )

        if trial is None:
            continue

        # Identify ILP model (B30, B60, ...)
        if method == "ILP":

            ilp_model = next(
                (
                    str(key) for key in run_path
                    if re.fullmatch(r"B\d+", str(key))
                ),
                None
            )

            if ilp_model is None:
                print(
                    f"WARNING: ILP model not identified: {run_path}"
                )
                continue

        else:
            ilp_model = None

        # Number of sensors
        n_sensors = len(run_data["sensor_sequence"])

        efficiency = run_data["efficiency"]

        if efficiency is None:
            continue

        key = (
            method,
            experiment_type,
            ilp_model,
            n_sensors,
            world,
            trial
        )

        trial_world_efficiencies.setdefault(
            key, []
        ).append(efficiency)


# ------------------------------------------------------------
# 9.2 Average runs within each trial and world
# ------------------------------------------------------------

trial_world_means = {}

for key, efficiencies in trial_world_efficiencies.items():

    trial_world_means[key] = np.mean(efficiencies)


# ------------------------------------------------------------
# 9.3 Average across trials and worlds
# ------------------------------------------------------------

# Key:
# (method, experiment_type, ilp_model, n_sensors)

aggregated_efficiencies = {}

for key, efficiency in trial_world_means.items():

    (
        method,
        experiment_type,
        ilp_model,
        n_sensors,
        world,
        trial
    ) = key

    aggregate_key = (
        method,
        experiment_type,
        ilp_model,
        n_sensors
    )

    aggregated_efficiencies.setdefault(
        aggregate_key, []
    ).append(efficiency)


# Final averages
plot_data = {}

for key, values in aggregated_efficiencies.items():

    plot_data[key] = np.mean(values)


# ------------------------------------------------------------
# 9.4 Generate one plot per experiment_type
# ------------------------------------------------------------

methods_to_plot = ["HITL", "AUTO", "ILP"]

experiment_types = sorted(
    set(key[1] for key in plot_data.keys())
)

for experiment_type in experiment_types:

    plt.figure(figsize=(10, 6))

    for method in methods_to_plot:

        # ------------------------------------------------
        # HITL and AUTO
        # ------------------------------------------------

        if method != "ILP":

            points = []

            for key, efficiency in plot_data.items():

                (
                    current_method,
                    current_type,
                    current_model,
                    n_sensors
                ) = key

                if (
                    current_method == method
                    and current_type == experiment_type
                ):

                    points.append(
                        (n_sensors, efficiency)
                    )

            points.sort()

            if points:

                x_values, y_values = zip(*points)

                plt.plot(
                    x_values,
                    y_values,
                    marker="o",
                    linewidth=2,
                    markersize=7,
                    label=method
                )

        # ------------------------------------------------
        # ILP: separate curve for each model
        # ------------------------------------------------

        else:

            ilp_models = sorted(
                set(
                    key[2]
                    for key in plot_data.keys()
                    if (
                        key[0] == "ILP"
                        and key[1] == experiment_type
                    )
                )
            )

            for ilp_model in ilp_models:

                points = []

                for key, efficiency in plot_data.items():

                    (
                        current_method,
                        current_type,
                        current_model,
                        n_sensors
                    ) = key

                    if (
                        current_method == "ILP"
                        and current_type == experiment_type
                        and current_model == ilp_model
                    ):

                        points.append(
                            (n_sensors, efficiency)
                        )

                points.sort()

                if points:

                    x_values, y_values = zip(*points)

                    plt.plot(
                        x_values,
                        y_values,
                        marker="o",
                        linewidth=2,
                        markersize=7,
                        label=f"ILP - {ilp_model}"
                    )

    plt.xlabel("Number of sensors", fontsize=12)

    plt.ylabel(
        "Mean inspection efficiency",
        fontsize=12
    )

    plt.title(
        f"Inspection Efficiency vs Number of Sensors\n"
        f"Experiment type: {experiment_type}",
        fontsize=14
    )

    relevant_sensor_counts = [
        key[3]
        for key in plot_data.keys()
        if key[1] == experiment_type
    ]

    plt.xticks(sorted(set(relevant_sensor_counts)))

    plt.ylim(0, 1.05)

    plt.grid(
        True,
        linestyle="--",
        alpha=0.5
    )

    plt.legend(title="Method")

    plt.tight_layout()

    plt.show()

# ============================================================
# 8. FINAL STATISTICS
# ============================================================



percentage_empty = (
    100 * empty_matrices / total_matrices
    if total_matrices > 0 else 0
)

# print("\n" + "=" * 50)
# print("FINAL STATISTICS")
# print("=" * 50)

# print(f"Total matrices generated: {total_matrices}")
# print(f"Empty matrices: {empty_matrices}")
# print(f"Non-empty matrices: {total_matrices - empty_matrices}")
# print(f"Percentage of empty matrices: {percentage_empty:.2f}%")

# print("\n" + "=" * 60)
# print("EMPTY MATRICES DETAILS")
# print("=" * 60)

# if empty_matrix_runs:

#     print(f"\nFound {len(empty_matrix_runs)} empty matrices:\n")

#     for i, run in enumerate(empty_matrix_runs, 1):
#         print(f"{i}. {run}")

# else:
#     print("\nNo empty matrices found.")

# print("=" * 60)
# print("\nProcessing completed.")

# M1 = print_matrix(
#     all_matrices,
#     method="HITL",
#     mode="deterministic",
#     world="world_1",
#     model="gpt-6-luna",
#     trial="trial_1",
#     run="run_1"
# )

# M2 = print_matrix(
#     all_matrices,
#     method="HITL",
#     mode="deterministic",
#     world="world_1",
#     model="gpt-6-luna",
#     trial="trial_2",
#     run="run_7"
# )
# M3 = print_matrix(
#     all_matrices,
#     method="HITL",
#     mode="deterministic",
#     world="world_2",
#     model="gpt-6-luna",
#     trial="trial_1",
#     run="run_15"
# )

# M4 = print_matrix(
#     all_matrices,
#     method="HITL",
#     mode="deterministic",
#     world="world_2",
#     model="gpt-6-luna",
#     trial="trial_2",
#     run="run_13"
# )

