
# ============================================================
# 1. IMPORTS AND CONFIGURATION
# ============================================================

import json
import re
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


BASE_DIR = Path(__file__).resolve().parent

FILES = [
    BASE_DIR / "raw_data/experiments_AUTO.json",
    BASE_DIR / "raw_data/experiments_ILP.json"
]

HITL_CUSTOM_FILE = BASE_DIR / "processed_HITL_data/raw_inspection_matrices.json"

RAG_FILES = {
    "world_1": BASE_DIR / "../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": BASE_DIR / "../../llm_interface/config/RAG/RAG_map2.json"
}

OUTPUT_FILE = BASE_DIR / "inspection_matrices_refactored.json"


# ============================================================
# 2. DATA EXTRACTION
# ============================================================

def find_runs(data, path=()):
    """
    Find experiment runs recursively inside nested JSON data.

    A run is identified by the presence of the key "sensor_sequence".

    Example:
        data = {
            "experiment_1": {
                "runs": [
                    {"sensor_sequence": [10, 20, 30]},
                    {"sensor_sequence": [40, 50, 60]}
                ]
            }
        }

        for path, run in find_runs(data):
            print(path, run)

        Output:
            ('experiment_1', 'runs', 0) {'sensor_sequence': [10, 20, 30]}
            ('experiment_1', 'runs', 1) {'sensor_sequence': [40, 50, 60]}

    Args:
        data: Nested dictionary or list containing experiment data.
        path: Tuple tracking the current position in the JSON structure.

    Yields:
        A tuple containing:
            - path: Position of the run in the original JSON.
            - data: Dictionary containing the sensor_sequence.
    """

    if isinstance(data, dict):

        if "sensor_sequence" in data:
            yield path, data
            return

        for key, value in data.items():
            yield from find_runs(value, path + (key,))

    elif isinstance(data, list):

        for i, value in enumerate(data):
            yield from find_runs(value, path + (i,))


def extract_data():
    """
    Load all experiment files and generate inspection matrices.

    Returns:
        all_matrices: Nested dictionary containing AUTO and ILP runs.
        hitl_custom: Original HITL-custom JSON data.
        world_areas: Available inspection areas for each world.
        camera_mappings: Mapping between cameras and areas.
    """

    # --------------------------------------------------------
    # Load RAG configurations
    # --------------------------------------------------------

    world_areas = {}
    camera_mappings = {}

    for world, filename in RAG_FILES.items():

        with open(filename, encoding="utf-8") as f:
            rag = json.load(f)

        world_areas[world] = sorted(rag.get("areas", {}).keys())
        camera_mappings[world] = {}

        for sensor, info in rag.get("sensors", {}).items():

            if sensor.lower().startswith("camera_"):
                camera_mappings[world][sensor.lower()] = info.get("areas", [])

    # --------------------------------------------------------
    # Load HITL-custom data
    # --------------------------------------------------------

    with open(HITL_CUSTOM_FILE, encoding="utf-8") as f:
        hitl_custom = json.load(f)

    # --------------------------------------------------------
    # Load AUTO and ILP experiment files
    # --------------------------------------------------------

    all_matrices = {}

    for filename in FILES:

        method = filename.stem.replace("experiments_", "")
        print(f"\nProcessing {method}...")

        with open(filename, encoding="utf-8") as f:
            data = json.load(f)

        all_matrices[method] = {}

        for path, run in find_runs(data):

            # Identify the world from the JSON hierarchy
            world = next((str(k) for k in path if str(k).startswith("world_")), None)

            if world not in world_areas:
                print(f"WARNING: Unknown world: {world}")
                continue

            # ------------------------------------------------
            # Extract inspections from ILP actions
            # ------------------------------------------------

            if method == "ILP":

                inspections = {}

                for turn, actions in enumerate(run.get("actions", []), start=1):

                    inspections[turn] = {}

                    if not isinstance(actions, dict):
                        continue

                    deployments = actions.get("deploy", {})

                    if not isinstance(deployments, dict):
                        continue

                    for area in deployments.values():

                        if isinstance(area, str):
                            inspections[turn][area] = 1.0

            # ------------------------------------------------
            # Extract inspections from AUTO conversations
            # ------------------------------------------------

            else:

                inspections = {}
                conversation = run.get("conversation", [])
                expected_sensors = len(run.get("sensor_sequence", []))

                # Find the last sensor completion message
                completed_sensors = 0
                completion_index = None

                for i, message in enumerate(conversation):

                    if isinstance(message, dict) and "data_orchestrator" in message:
                        completed_sensors += 1

                        if completed_sensors >= expected_sensors:
                            completion_index = i
                            break

                if completion_index is None:
                    print(f"WARNING: Sensor sequence not completed: {path}")
                    continue

                conversation = conversation[completion_index + 1:]

                # Identify real user messages, excluding robot notifications
                def is_user_message(message):

                    if not isinstance(message, dict):
                        return False

                    content = message.get("user")

                    if not isinstance(content, str):
                        return False

                    return not re.match(r"^\s*turtlebot\d+_\d+\s*:", content, re.IGNORECASE)

                has_user_messages = any(is_user_message(m) for m in conversation)

                # If there are no user messages, stop at the last assistant response
                if not has_user_messages:

                    last_assistant = max(
                        (i for i, m in enumerate(conversation)
                         if isinstance(m, dict) and "assistant" in m),
                        default=len(conversation) - 1
                    )

                    conversation = conversation[:last_assistant + 1]

                # Process conversation turns
                current_turn = 1
                inspections[current_turn] = {}

                for message in conversation:

                    if not isinstance(message, dict):
                        continue

                    if is_user_message(message):

                        if has_user_messages:
                            current_turn += 1
                            inspections.setdefault(current_turn, {})

                        continue

                    # Extract function calls
                    function_call = message.get("function_call")

                    if not isinstance(function_call, str):
                        continue

                    match = re.search(
                        r"name\s*-\s*(.*?)\s*arguments\s*-\s*(\{.*\})",
                        function_call,
                        re.DOTALL
                    )

                    if not match:
                        continue

                    function_name = match.group(1).strip()

                    try:
                        arguments = json.loads(match.group(2).strip())
                    except json.JSONDecodeError:
                        continue

                    # Camera inspection: value = 0.5
                    if function_name == "get_camera_feed":

                        for camera in arguments.get("cameras_names_list", []):

                            areas = camera_mappings[world].get(camera.lower(), [])

                            if isinstance(areas, str):
                                areas = [areas]

                            for area in areas:
                                inspections[current_turn][area] = max(
                                    inspections[current_turn].get(area, 0),
                                    0.5
                                )

                    # Robot deployment: value = 1.0
                    elif function_name == "send_robots_to_area":

                        for robot in arguments.get("robots_sequence", []):

                            if not isinstance(robot, dict):
                                continue

                            area = robot.get("area_to_reach")

                            if area:
                                inspections[current_turn][area] = 1.0

            # ------------------------------------------------
            # Convert inspections into a DataFrame
            # ------------------------------------------------

            matrix = pd.DataFrame(
                0.0,
                index=sorted(inspections),
                columns=world_areas[world]
            )

            for turn, areas in inspections.items():

                for area, value in areas.items():

                    if area in matrix.columns:
                        matrix.loc[turn, area] = value
                    else:
                        print(f"WARNING: Unknown area: {area}")

            # Binary vector: 1 if an area was inspected at least once
            binary_vector = (matrix.sum(axis=0) > 0).astype(int)
            n_inspected = int(binary_vector.sum())

            # Check whether the final intruder area was inspected
            intruder = run["intruder_final_area"]
            success = run["success"]

            if success == 1 and intruder != "FP":

                if intruder not in matrix.columns or matrix[intruder].sum() == 0:
                    print(f"WARNING: Final area not inspected: {method} | {world} | {path} | {intruder}")

            # Restore the original experiment hierarchy
            current = all_matrices[method]

            for key in path[:-1]:
                current = current.setdefault(key, {})

            current[path[-1]] = {
                "sensor_sequence": run["sensor_sequence"],
                "intruder_final_area": intruder,
                "success_rate": success,
                "matrix": matrix,
                "inspected_areas": binary_vector.to_numpy(),
                "n_inspected_areas": n_inspected,
                "efficiency": None
            }

    print("\nRUNS LOADED")

    for method, data in all_matrices.items():
        print(f"{method}: {sum(1 for _ in find_runs(data))} runs")

    return all_matrices, hitl_custom, world_areas




# ============================================================
# 3. EFFICIENCY AND HITL-CUSTOM ANALYSIS
# ============================================================

def calculate_efficiency(all_matrices, hitl_custom, world_areas):
    """
    Calculate the minimum number of inspected areas and the
    inspection efficiency for all standard and HITL-custom runs.
    """

    minimum_areas = {}

    # --------------------------------------------------------
    # Find minimum inspected areas for AUTO and ILP
    # --------------------------------------------------------

    for method, data in all_matrices.items():

        for path, run in find_runs(data):

            world = next((str(k) for k in path if str(k).startswith("world_")), None)

            if world is None:
                continue

            sequence = json.dumps(run["sensor_sequence"], sort_keys=True, ensure_ascii=False)
            group = (world, sequence, run["intruder_final_area"])

            if run["success_rate"] == 1:

                n = run["n_inspected_areas"]

                minimum_areas[group] = min(minimum_areas.get(group, n), n)

    # --------------------------------------------------------
    # Include HITL-custom HUMAN inspections in the minimum
    # --------------------------------------------------------

    for path, run in find_runs(hitl_custom):

        world = next((str(k) for k in path if str(k).startswith("world_")), None)

        if world not in world_areas:
            continue

        if float(run["success"]) != 1:
            continue

        sequence = json.dumps(run["sensor_sequence"], sort_keys=True, ensure_ascii=False)
        group = (world, sequence, run["intruder_final_area"])

        human_data = run.get("matrix", {})
        human_areas = set()

        for turn_data in human_data.values():

            if not isinstance(turn_data, dict):
                continue

            for area, value in turn_data.get("human", {}).items():

                if isinstance(value, str) and value.lower().strip().startswith(("robot", "camera")):
                    human_areas.add(area)

        n_human = len(human_areas)

        minimum_areas[group] = min(minimum_areas.get(group, n_human), n_human)

    # --------------------------------------------------------
    # Calculate efficiency for each standard run
    # --------------------------------------------------------

    for method, data in all_matrices.items():

        for path, run in find_runs(data):

            world = next((str(k) for k in path if str(k).startswith("world_")), None)

            if world not in world_areas:
                continue

            sequence = json.dumps(run["sensor_sequence"], sort_keys=True, ensure_ascii=False)
            group = (world, sequence, run["intruder_final_area"])

            n_total = len(world_areas[world])
            n_inspected = run["n_inspected_areas"]
            success = float(run["success_rate"])

            a_min = minimum_areas.get(group)

            if a_min is None:
                efficiency = 0.0
                print(f"WARNING: Minimum not found: {group}")

            elif n_total == a_min:
                efficiency = success

            elif n_total > a_min:
                efficiency = success * (1 - (n_inspected - a_min) / (n_total - a_min))

            else:
                efficiency = 0.0
                print(f"WARNING: Invalid efficiency denominator: {group}")

            run["min_inspected_areas"] = a_min
            run["efficiency"] = round(efficiency, 2)

    print("\nMINIMUM INSPECTED AREAS")

    for group, value in minimum_areas.items():
        print(f"World: {group[0]} | Sequence: {group[1]} | Intruder: {group[2]} | Minimum: {value}")

    return minimum_areas


def process_hitl_custom(hitl_custom, minimum_areas, world_areas):
    """
    Process HITL-custom runs.

    Each run contains two inspection matrices:
        LLM matrix: inspections performed by the LLM.
        Human matrix: inspections performed by the human.

    Inspection values:
        Robot  = 1.0
        Camera = 0.5

    The human matrix is used for efficiency.
    Normalized Hamming distance compares the LLM and human matrices.
    """

    results = {}
    plot_data = []

    for path, run in find_runs(hitl_custom):

        world = next((str(k) for k in path if str(k).startswith("world_")), None)

        trial = next((str(k) for k in path if str(k).startswith("trial_")), None)

        if world not in world_areas or trial is None:
            continue

        # Identify experiment type and model
        experiment_type = str(path[0])

        model = next(
            (str(k) for k in path if str(k) not in (experiment_type, "HITL", world)
             and not str(k).startswith(("trial_", "run_"))),
            None
        )

        if model is None:
            print(f"WARNING: Invalid HITL-custom path: {path}")
            continue

        # ----------------------------------------------------
        # Convert original annotations into two DataFrames
        # ----------------------------------------------------

        llm_data = {}
        human_data = {}

        for turn, annotations in run.get("matrix", {}).items():

            if not isinstance(annotations, dict):
                continue

            llm_data[int(turn)] = {}
            human_data[int(turn)] = {}

            for area in world_areas[world]:

                for source, destination in (
                    ("LLM", llm_data),
                    ("human", human_data)
                ):

                    value = annotations.get(source, {}).get(area, "")

                    if isinstance(value, str):
                        value = value.lower().strip()
                    else:
                        value = ""

                    if value.startswith("robot"):
                        score = 1.0
                    elif value.startswith("camera"):
                        score = 0.5
                    else:
                        score = 0.0

                    destination[int(turn)][area] = score

        matrix_llm = pd.DataFrame.from_dict(llm_data, orient="index")
        matrix_human = pd.DataFrame.from_dict(human_data, orient="index")

        matrix_llm = matrix_llm.reindex(columns=world_areas[world], fill_value=0.0)
        matrix_human = matrix_human.reindex(columns=world_areas[world], fill_value=0.0)

        # ----------------------------------------------------
        # Count inspected areas
        # ----------------------------------------------------

        n_llm = int((matrix_llm.sum(axis=0) > 0).sum())
        n_human = int((matrix_human.sum(axis=0) > 0).sum())

        # ----------------------------------------------------
        # Calculate normalized Hamming distance
        # ----------------------------------------------------

        all_turns = sorted(set(matrix_llm.index) | set(matrix_human.index))

        distances = []

        for turn in all_turns:

            llm = matrix_llm.loc[turn].to_numpy(dtype=float) if turn in matrix_llm.index else np.zeros(len(world_areas[world]))
            human = matrix_human.loc[turn].to_numpy(dtype=float) if turn in matrix_human.index else np.zeros(len(world_areas[world]))

            # Count different areas
            hamming = np.count_nonzero(llm != human)

            # Count areas inspected by at least one side
            union = np.count_nonzero((llm != 0) | (human != 0))

            distance = hamming / union if union > 0 else 0.0
            distances.append(distance)

        mean_hamming = float(np.mean(distances)) if distances else 0.0

        # ----------------------------------------------------
        # Calculate human inspection efficiency
        # ----------------------------------------------------

        sequence = json.dumps(run["sensor_sequence"], sort_keys=True, ensure_ascii=False)
        group = (world, sequence, run["intruder_final_area"])

        a_min = minimum_areas.get(group)
        n_total = len(world_areas[world])
        success = float(run["success"])

        if a_min is None:
            efficiency = 0.0
            print(f"WARNING: HITL-custom minimum not found: {group}")

        elif n_total == a_min:
            efficiency = success

        elif n_total > a_min:
            efficiency = success * (1 - (n_human - a_min) / (n_total - a_min))

        else:
            efficiency = 0.0

        efficiency = round(efficiency, 2)

        # ----------------------------------------------------
        # Store data for plotting
        # ----------------------------------------------------

        plot_data.append({
            "experiment_type": experiment_type,
            "world": world,
            "trial": trial,
            "n_sensors": len(run["sensor_sequence"]),
            "success_rate": success,
            "efficiency": efficiency,
            "normalized_hamming_distance": mean_hamming
        })

        # ----------------------------------------------------
        # Store both matrices and metrics
        # ----------------------------------------------------

        run_name = str(path[-1])

        current = (
            results
            .setdefault(experiment_type, {})
            .setdefault("HITL-custom", {})
            .setdefault(world, {})
            .setdefault(model, {})
            .setdefault(trial, {})
        )

        current[run_name] = {
            "sensor_sequence": run["sensor_sequence"],
            "matrix_LLM": matrix_llm.to_dict(orient="index"),
            "matrix_human": matrix_human.to_dict(orient="index"),
            "n_inspected_areas_LLM": n_llm,
            "n_inspected_areas_human": n_human,
            "n_total_areas": n_total,
            "min_inspected_areas_human": a_min,
            "intruder_final_area": run["intruder_final_area"],
            "success_rate": success,
            "efficiency_human": efficiency,
            "normalized_hamming_distance": round(mean_hamming, 2),
            "normalized_hamming_distance_per_turn": {
                str(turn): round(float(distance), 2)
                for turn, distance in zip(all_turns, distances)
            }
        }

    return results, plot_data


# ============================================================
# 4. SAVE RESULTS
# ============================================================

def save_results(all_matrices, hitl_results, world_areas):
    """
    Combine standard and HITL-custom results into one JSON structure.

    DataFrames are converted into dictionaries because JSON
    cannot directly serialize pandas DataFrames.
    """

    results = {}

    # --------------------------------------------------------
    # Save AUTO and ILP results
    # --------------------------------------------------------

    for method, data in all_matrices.items():

        for path, run in find_runs(data):

            experiment_type = str(path[0])

            world = next((str(k) for k in path if str(k).startswith("world_")), None)

            trial = next((str(k) for k in path if str(k).startswith("trial_")), None)

            if world is None or trial is None:
                continue

            # Identify model
            if method == "ILP":

                model = next(
                    (str(k) for k in path if re.fullmatch(r"B\d+", str(k))),
                    None
                )

            else:

                model = next(
                    (str(k) for k in path
                     if str(k) not in (experiment_type, method, world)
                     and not str(k).startswith(("trial_", "run_"))),
                    None
                )

            if model is None:
                print(f"WARNING: Model not identified: {path}")
                continue

            # Convert inspection matrix to a JSON-compatible dictionary
            matrix = {
                str(turn): {
                    str(area): float(value)
                    for area, value in row.items()
                }
                for turn, row in run["matrix"].to_dict(orient="index").items()
            }

            run_result = {
                "sensor_sequence": run["sensor_sequence"],
                "matrix": matrix,
                "n_inspected_areas": int(run["n_inspected_areas"]),
                "min_inspected_areas": run.get("min_inspected_areas"),
                "n_total_areas": len(world_areas[world]),
                "success_rate": float(run["success_rate"]),
                "efficiency": float(run["efficiency"])
            }

            # Rebuild the original hierarchy
            current = (
                results
                .setdefault(experiment_type, {})
                .setdefault(method, {})
                .setdefault(world, {})
                .setdefault(model, {})
                .setdefault(trial, {})
            )

            current[str(path[-1])] = run_result

    # --------------------------------------------------------
    # Add HITL-custom results
    # --------------------------------------------------------

    for experiment_type, methods in hitl_results.items():

        results.setdefault(experiment_type, {})

        results[experiment_type]["HITL-custom"] = methods["HITL-custom"]

    # --------------------------------------------------------
    # Export JSON
    # --------------------------------------------------------

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=4, ensure_ascii=False)

    print(f"\nResults saved to: {OUTPUT_FILE}")

    return results


# ============================================================
# 5. PLOTS AND STATISTICS
# ============================================================

def plot_results(all_matrices, hitl_plot_data):
    """Generate inspection efficiency and Hamming distance plots."""

    # ========================================================
    # 5.1 INSPECTION EFFICIENCY
    # ========================================================

    # Group efficiency values by method, experiment, model,
    # number of sensors, world and trial.

    grouped = {}

    for method, data in all_matrices.items():

        for path, run in find_runs(data):

            world = next((str(k) for k in path if str(k).startswith("world_")), None)
            trial = next((str(k) for k in path if str(k).startswith("trial_")), None)

            if world is None or trial is None:
                continue

            model = next(
                (str(k) for k in path if re.fullmatch(r"B\d+", str(k))),
                None
            ) if method == "ILP" else None

            key = (
                method,
                str(path[0]),
                model,
                len(run["sensor_sequence"]),
                world,
                trial
            )

            grouped.setdefault(key, []).append(run["efficiency"])

    # Add HITL-custom human efficiency
    for run in hitl_plot_data:

        key = (
            "HITL-custom",
            run["experiment_type"],
            None,
            run["n_sensors"],
            run["world"],
            run["trial"]
        )

        grouped.setdefault(key, []).append(run["efficiency"])

    # First average runs within each trial and world
    trial_means = {
        key: np.mean(values)
        for key, values in grouped.items()
    }

    # Then average across trials and worlds
    aggregated = {}

    for key, value in trial_means.items():

        method, experiment, model, n_sensors, world, trial = key

        aggregate_key = (method, experiment, model, n_sensors)

        aggregated.setdefault(aggregate_key, []).append(value)

    plot_data = {
        key: np.mean(values)
        for key, values in aggregated.items()
    }

    # --------------------------------------------------------
    # Generate one efficiency plot per experiment type
    # --------------------------------------------------------

    methods = ["HITL", "HITL-custom", "AUTO", "ILP"]

    experiment_types = sorted(set(key[1] for key in plot_data))

    for experiment in experiment_types:

        plt.figure(figsize=(10, 6))

        for method in methods:

            # AUTO, HITL and HITL-custom
            if method != "ILP":

                points = sorted(
                    (key[3], value)
                    for key, value in plot_data.items()
                    if key[0] == method and key[1] == experiment
                )

                if points:

                    x, y = zip(*points)

                    plt.plot(x, y, marker="o", linewidth=2, markersize=7, label=method)

            # ILP: one curve for each model
            else:

                models = sorted(
                    set(key[2] for key in plot_data
                        if key[0] == "ILP" and key[1] == experiment)
                )

                for model in models:

                    points = sorted(
                        (key[3], value)
                        for key, value in plot_data.items()
                        if key[0] == "ILP" and key[1] == experiment and key[2] == model
                    )

                    if points:

                        x, y = zip(*points)

                        plt.plot(x, y, marker="o", linewidth=2, markersize=7, label=f"ILP - {model}")

        sensor_counts = sorted(set(key[3] for key in plot_data if key[1] == experiment))

        plt.xlabel("Number of sensors", fontsize=12)
        plt.ylabel("Mean inspection efficiency", fontsize=12)
        plt.title(f"Inspection Efficiency vs Number of Sensors\nExperiment type: {experiment}", fontsize=14)
        plt.xticks(sensor_counts)
        plt.ylim(0, 1.05)
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.legend(title="Method")
        plt.tight_layout()
        plt.show()

    # ========================================================
    # 5.2 NORMALIZED HAMMING DISTANCE
    # ========================================================

    # Group Hamming distances by experiment type and sensor count
    hir_grouped = {}

    for run in hitl_plot_data:

        key = (run["experiment_type"], run["n_sensors"])

        hir_grouped.setdefault(key, []).append(run["normalized_hamming_distance"])

    # Calculate mean, standard deviation and number of runs
    hir_statistics = {}

    for key, values in hir_grouped.items():

        hir_statistics[key] = {
            "mean": float(np.mean(values)),
            "std_dev": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
            "n_runs": len(values)
        }

    # Print final statistics
    print("\n" + "=" * 75)
    print("NORMALIZED HAMMING DISTANCE - FINAL STATISTICS")
    print("=" * 75)

    for key in sorted(hir_statistics):

        experiment, n_sensors = key
        stats = hir_statistics[key]

        print(
            f"Mode: {experiment:12s} | "
            f"N_s: {n_sensors:2d} | "
            f"Mean HIR: {stats['mean']:.4f} | "
            f"Std Dev: {stats['std_dev']:.4f} | "
            f"Runs: {stats['n_runs']}"
        )

    # --------------------------------------------------------
    # Plot Hamming distance
    # --------------------------------------------------------

    plt.figure(figsize=(8, 6))

    SHOW_ERROR_BARS = False

    for experiment in sorted(set(key[0] for key in hir_statistics)):

        points = sorted(
            (n_sensors, stats["mean"], stats["std_dev"])
            for (current, n_sensors), stats in hir_statistics.items()
            if current == experiment
        )

        if not points:
            continue

        x = [p[0] for p in points]
        y = [p[1] for p in points]
        errors = [p[2] for p in points]

        if SHOW_ERROR_BARS:

            plt.errorbar(x, y, yerr=errors, marker="o", linewidth=2, markersize=7, capsize=5, label=experiment)

        else:

            plt.plot(x, y, marker="o", linewidth=2, markersize=7, label=experiment)

    plt.xlabel(r"$N_s$", fontsize=14)
    plt.ylabel(r"$\bar{HIR}$", fontsize=14)
    plt.xticks(sorted(set(key[1] for key in hir_statistics)))
    plt.ylim(0, 1.05)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(title="Mode")
    plt.tight_layout()

    plt.savefig(BASE_DIR / "normalized_hamming_distance.pdf", format="pdf", bbox_inches="tight")

    plt.show()



if __name__ == "__main__":

    print("\n" + "=" * 60)
    print("INSPECTION ANALYSIS")
    print("=" * 60)

    # --------------------------------------------------------
    # 1. Extract all experiment data
    # --------------------------------------------------------

    all_matrices, hitl_custom, world_areas = extract_data()

    # --------------------------------------------------------
    # 2. Calculate minimum inspected areas and efficiency
    # --------------------------------------------------------

    minimum_areas = calculate_efficiency(
        all_matrices,
        hitl_custom,
        world_areas
    )

    # --------------------------------------------------------
    # 3. Process HITL-custom
    # --------------------------------------------------------

    hitl_results, hitl_plot_data = process_hitl_custom(
        hitl_custom,
        minimum_areas,
        world_areas
    )

    # --------------------------------------------------------
    # 4. Save all results into one JSON file
    # --------------------------------------------------------

    results = save_results(
        all_matrices,
        hitl_results,
        world_areas
    )

    # --------------------------------------------------------
    # 5. Generate plots and final statistics
    # --------------------------------------------------------

    plot_results(
        all_matrices,
        hitl_plot_data
    )

    print("\n" + "=" * 60)
    print("PROCESSING COMPLETED")
    print("=" * 60)

    print(f"Results saved to: {OUTPUT_FILE}")
    print(f"Experiment methods: {list(all_matrices.keys())}")
    print(f"HITL-custom runs: {len(hitl_plot_data)}")


