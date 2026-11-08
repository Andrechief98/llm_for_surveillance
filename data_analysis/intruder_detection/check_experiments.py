
import json
import os
from collections import defaultdict


# ============================================================
# CONFIGURATION
# ============================================================

script_dir = os.path.dirname(__file__)
JSON_FILE = os.path.join(script_dir, "experiments_HITL.json")

# Modes available in the JSON
ALL_MODES = [
    "HITL",
    # "AUTO",
    # "HUMAN",
    # "ILP",
]

# Select which modes to print
MODES_TO_PRINT = [
    "HITL",
    # "AUTO",
    # "HUMAN",
    # "ILP",
]

# Select the number of sensors to check
# Examples:
# [1]       -> only experiments with 1 sensor
# [1, 2]    -> experiments with 1 or 2 sensors
# [1, 2, 3] -> experiments with 1, 2 or 3 sensors
SENSOR_COUNTS = [4]


# ============================================================
# EXPECTED SEQUENCES
# Format: experiment_type | sensor_sequence | target_area
# ============================================================

EXPECTED_RAW = {

"1": """
 deterministic|Camera_1|B
 deterministic|Lidar_3|C
 deterministic|Lidar_6|G
 deterministic|Lidar_2|A
 deterministic|Camera_2|F

 deterministic|Lidar_5,Camera_2|F
 deterministic|Lidar_6,Lidar_1|A
 deterministic|Lidar_5,Lidar_4|C
 deterministic|Lidar_7,Lidar_6|E
 deterministic|Camera_1,Lidar_6|G

 deterministic|Lidar_3,Camera_1,Lidar_2|A
 deterministic|Lidar_1,Lidar_5,Lidar_4|C
 deterministic|Camera_2,Lidar_6,Lidar_7|G
 deterministic|Lidar_6,Lidar_7,Lidar_8|I
 deterministic|Lidar_8,Camera_2,Lidar_5|D

 deterministic|Camera_2,Lidar_6,Lidar_7,Lidar_8|L
 deterministic|Lidar_7,Lidar_6,Lidar_5,Lidar_4|C
 deterministic|Lidar_5,Lidar_6,Lidar_1,Lidar_2|A
 deterministic|Lidar_1,Lidar_8,Lidar_7,Lidar_6|E
 deterministic|Camera_1,Lidar_3,Lidar_4,Lidar_5|D

 uncertain|Camera_2|-
 uncertain|Lidar_8|RAND(L,A,I)
 uncertain|Lidar_1|-
 uncertain|Lidar_5|RAND(F,C,D)
 uncertain|Camera_1|RAND(B,C,E)

 uncertain|Camera_1,Lidar_3|RAND(E,A,C)
 uncertain|Lidar_1,Lidar_8|RAND(H,G,B)
 uncertain|Lidar_3,Lidar_7|C
 uncertain|Lidar_4,Lidar_1|E
 uncertain|Lidar_7,Lidar_8|RAND(E,A,C)

 uncertain|Camera_1,Lidar_6,Lidar_5|G
 uncertain|Lidar_8,Lidar_1,Camera_2|A
 uncertain|Lidar_4,Lidar_5,Lidar_6|RAND(B,I,F)
 uncertain|Lidar_2,Lidar_1,Lidar_8|RAND(G,E,B)
 uncertain|Camera_2,Lidar_2,Lidar_5|D

 uncertain|Camera_1,Lidar_2,Lidar_1,Lidar_7|A
 uncertain|Lidar_8,Lidar_1,Camera_1,Lidar_3|C
 uncertain|Lidar_5,Lidar_4,Lidar_3,Camera_1|RAND(E,I,A)
 uncertain|Lidar_3,Camera_1,Lidar_6,Lidar_7|RAND(L,E,I)
 uncertain|Lidar_7,Lidar_8,Camera_2,Lidar_1|E
""",

"2": """
 deterministic|Camera_9|D
 deterministic|Camera_2|E
 deterministic|Lidar_7|L
 deterministic|Lidar_11|S
 deterministic|Lidar_2|T

 deterministic|Camera_5,Lidar_1|A
 deterministic|Lidar_2,Lidar_8|M
 deterministic|Lidar_8,Lidar_14|V
 deterministic|Lidar_13,Lidar_12|R
 deterministic|Camera_7,Lidar_6|I

 deterministic|Camera_10,Lidar_2,Lidar_8|M
 deterministic|Lidar_1,Camera_10,Lidar_2|S
 deterministic|Camera_6,Lidar_11,Lidar_12|P
 deterministic|Lidar_5,Lidar_2,Lidar_1|F
 deterministic|Lidar_10,Lidar_9,Lidar_12|R

 deterministic|Camera_9,Camera_10,Lidar_1,Camera_5|F
 deterministic|Lidar_3,Lidar_4,Lidar_5,Lidar_6|S
 deterministic|Camera_6,Lidar_11,Lidar_2,Camera_10|A
 deterministic|Lidar_11,Lidar_10,Lidar_8,Lidar_7|L
 deterministic|Lidar_5,Lidar_4,Lidar_2,Lidar_3|G

 uncertain|Camera_5|-
 uncertain|Lidar_3|RAND(A,T,S)
 uncertain|Lidar_6|-
 uncertain|Lidar_14|RAND(Z,R,P)
 uncertain|Camera_9|RAND(A,C,E)

 uncertain|Lidar_4,Lidar_2|RAND(C,B,S)
 uncertain|Lidar_1,Camera_10|RAND(E,D,C)
 uncertain|Camera_10,Lidar_8|A
 uncertain|Lidar_5,Camera_1|B
 uncertain|Camera_6,Lidar_11|RAND(S,N,P)

 uncertain|Camera_6,Lidar_11,Lidar_4|R
 uncertain|Lidar_14,Camera_2,Lidar_10|N
 uncertain|Lidar_6,Lidar_8,Lidar_12|RAND(R,T,Z)
 uncertain|Lidar_5,Lidar_2,Lidar_1|RAND(D,E,C)
 uncertain|Camera_4,Lidar_13,Lidar_12|RAND(Q,R,N)

 uncertain|Camera_8,Camera_10,Lidar_2,Lidar_13|T
 uncertain|Lidar_7,Lidar_2,Lidar_10,Lidar_11|R
 uncertain|Camera_4,Lidar_13,Lidar_2,Lidar_1|RAND(C,E,D)
 uncertain|Lidar_12,Lidar_10,Lidar_9,Lidar_8|RAND(T,L,R)
 uncertain|Lidar_2,Camera_10,Lidar_1,Camera_2|E
"""
}


# ============================================================
# PARSE EXPECTED SEQUENCES
# ============================================================

def parse_expected_sequences(raw_data):

    expected = {}

    for world, content in raw_data.items():

        expected[world] = {
            "deterministic": [],
            "uncertain": [],
        }

        for line in content.splitlines():

            line = line.strip()

            if not line:
                continue

            line = line.lstrip("#").strip()

            parts = line.split("|")

            if len(parts) < 2:
                continue

            experiment_type = parts[0].strip()

            if experiment_type not in expected[world]:
                continue

            sensors = tuple(
                sensor.strip()
                for sensor in parts[1].split(",")
                if sensor.strip()
            )

            if sensors:
                expected[world][experiment_type].append(sensors)

    return expected


# ============================================================
# NORMALIZE WORLD NAME
# ============================================================

def normalize_world(world):

    return world.lower().replace("world_", "")


# ============================================================
# GET TRIALS
# ============================================================

def get_models(world_data):

    is_model_layer = any(
        not key.startswith("trial_")
        for key in world_data
    )

    if is_model_layer:
        return world_data.items()

    return [("N/A", world_data)]


# ============================================================
# ANALYSIS
# ============================================================

def analyze_experiments(
    data,
    expected,
    modes,
    sensor_counts
):

    results = {}

    sensor_counts = set(sensor_counts)

    for mode in modes:

        results[mode] = {}

        for experiment_type in ["deterministic", "uncertain"]:

            experiment_data = data.get(experiment_type, {})
            mode_data = experiment_data.get(mode, {})

            if not isinstance(mode_data, dict):
                continue

            results[mode][experiment_type] = {}

            for world, world_data in mode_data.items():

                if not isinstance(world_data, dict):
                    continue

                world_id = normalize_world(world)

                expected_sequences = expected.get(
                    world_id, {}
                ).get(experiment_type, [])

                # Filter expected sequences by sensor count
                expected_sequences = [
                    sequence
                    for sequence in expected_sequences
                    if len(sequence) in sensor_counts
                ]

                if not expected_sequences:
                    continue

                results[mode][experiment_type][world] = {}

                for model, model_data in get_models(world_data):

                    if not isinstance(model_data, dict):
                        continue

                    for trial, trial_data in model_data.items():

                        if not trial.startswith("trial_"):
                            continue

                        if not isinstance(trial_data, dict):
                            continue

                        executed_sequences = []

                        for run, run_data in trial_data.items():

                            if not run.startswith("run_"):
                                continue

                            if not isinstance(run_data, dict):
                                continue

                            sensors = run_data.get("sensor_sequence")

                            if not isinstance(sensors, list) or not sensors:
                                continue

                            # Filter executed sequences by sensor count
                            if len(sensors) not in sensor_counts:
                                continue

                            executed_sequences.append(tuple(sensors))

                        if not executed_sequences:
                            continue

                        executed_set = set(executed_sequences)

                        missing_by_sensor_count = defaultdict(list)

                        for sequence in expected_sequences:

                            if sequence not in executed_set:

                                missing_by_sensor_count[
                                    len(sequence)
                                ].append(sequence)

                        results[mode][experiment_type][world].setdefault(
                            model, {}
                        )[trial] = {
                            "missing": dict(missing_by_sensor_count),
                            "n_runs": len(executed_sequences),
                            "n_executed": len(executed_set),
                            "n_expected": len(expected_sequences),
                        }

    return results


# ============================================================
# PRINT RESULTS
# ============================================================

def print_results(results):

    for mode, experiments in results.items():

        print("\n" + "=" * 75)
        print(f"MODE: {mode}")
        print("=" * 75)

        for experiment_type in ["deterministic", "uncertain"]:

            print(f"\n  Experiment type: {experiment_type}")
            print("  " + "-" * 65)

            worlds = experiments.get(experiment_type, {})

            if not worlds:
                print("    No trials with executed runs.")
                continue

            for world, models in worlds.items():

                print(f"\n    {world}")

                for model, trials in models.items():

                    for trial, stats in trials.items():

                        print(f"\n      Model: {model}")
                        print(f"      Trial: {trial}")
                        print(f"      Executed runs: {stats['n_runs']}")
                        print(f"      Unique executed sequences: {stats['n_executed']}")
                        print(f"      Expected sequences: {stats['n_expected']}")

                        missing = stats["missing"]

                        if not missing:
                            print("      All expected sequences executed!")
                            continue

                        for n_sensors in sorted(missing):

                            sequences = missing[n_sensors]

                            print(
                                f"\n      Missing sequences "
                                f"({n_sensors} sensor(s)):"
                            )

                            for sequence in sequences:
                                print(f"        - {list(sequence)}")


# ============================================================
# MAIN
# ============================================================

def main():

    # Validate configuration
    invalid_modes = set(MODES_TO_PRINT) - set(ALL_MODES)

    if invalid_modes:
        raise ValueError(
            f"Invalid modes: {sorted(invalid_modes)}"
        )

    if not SENSOR_COUNTS or any(
        not isinstance(n, int) or n < 1
        for n in SENSOR_COUNTS
    ):
        raise ValueError(
            "SENSOR_COUNTS must contain positive integers."
        )

    with open(JSON_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    expected = parse_expected_sequences(EXPECTED_RAW)

    results = analyze_experiments(
        data,
        expected,
        MODES_TO_PRINT,
        SENSOR_COUNTS
    )

    print_results(results)


if __name__ == "__main__":
    main()
