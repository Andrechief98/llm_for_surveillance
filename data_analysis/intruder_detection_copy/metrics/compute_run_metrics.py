
import json
import numpy as np
from pathlib import Path
from itertools import combinations



# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

INPUT_FILES = [
    BASE_DIR / "../inspection_matrices/inspection_matrices_HITL.json",
    BASE_DIR / "../inspection_matrices/inspection_matrices_AUTO.json",
    BASE_DIR / "../inspection_matrices/inspection_matrices_ILP.json"
]

METRICS_OUTPUT_FILE = BASE_DIR / "run_metrics.json"
JACCARD_OUTPUT_FILE = BASE_DIR / "jaccard_similarity.json"

# ============================================================
# LOAD INPUT FILES
# ============================================================

data = {}

for filename in INPUT_FILES:

    with open(filename, "r", encoding="utf-8") as file:
        data.update(json.load(file))


# ============================================================
# 1. COUNT INSPECTED AREAS AND BUILD BINARY DICTIONARY
# ============================================================

def get_inspected_areas(matrix):

    inspected_areas = {
        area: 0
        for turn_data in matrix.values()
        for area in turn_data
    }

    for turn_data in matrix.values():

        for area, value in turn_data.items():

            if isinstance(value, list):
                is_inspected = any(v != 0 for v in value)
            else:
                is_inspected = value != 0

            if is_inspected:
                inspected_areas[area] = 1

    n_inspected = sum(inspected_areas.values())

    return n_inspected, inspected_areas


# ============================================================
# 2. NORMALIZED HAMMING DISTANCE
# ============================================================

def normalized_hamming_distance(matrix_llm, matrix_human):

    turns = set(matrix_llm) | set(matrix_human)

    distances_per_turn = {}

    for turn in sorted(turns, key=lambda x: int(x)):

        llm = matrix_llm.get(turn, {})
        human = matrix_human.get(turn, {})

        areas = set(llm) | set(human)

        hamming = 0
        active_union = 0

        for area in areas:

            llm_active = llm.get(area, 0) != 0
            human_active = human.get(area, 0) != 0

            hamming += llm_active != human_active
            active_union += llm_active or human_active

        distance = (
            hamming / active_union
            if active_union > 0 else 0.0
        )

        distances_per_turn[turn] = distance

    mean_distance = (
        float(np.mean(list(distances_per_turn.values())))
        if distances_per_turn else 0.0
    )

    return mean_distance, distances_per_turn


# ============================================================
# 3. FIND MINIMUM INSPECTED AREAS
# ============================================================

minimum_areas = {}

for method, method_data in data.items():

    for mode, worlds in method_data.items():

        for world, trials in worlds.items():

            for trial, runs in trials.items():

                for run, run_data in runs.items():

                    if float(run_data["success"]) != 1:
                        continue

                    sequence_key = json.dumps(
                        run_data["sensor_sequence"],
                        sort_keys=True
                    )

                    group_key = (
                        world,
                        sequence_key,
                        run_data["intruder_final_area"]
                    )

                    if method == "HITL":
                        matrix = run_data["matrix_human"]
                    else:
                        matrix = run_data["matrix"]

                    n_inspected, _ = get_inspected_areas(matrix)

                    if group_key not in minimum_areas:
                        minimum_areas[group_key] = n_inspected
                    else:
                        minimum_areas[group_key] = min(
                            minimum_areas[group_key],
                            n_inspected
                        )


# ============================================================
# 4. CALCULATE METRICS FOR EACH RUN
# ============================================================

results = {}

for method, method_data in data.items():

    results[method] = {}

    for mode, worlds in method_data.items():

        results[method][mode] = {}

        for world, trials in worlds.items():

            results[method][mode][world] = {}

            for trial, runs in trials.items():

                results[method][mode][world][trial] = {}

                for run, run_data in runs.items():

                    sequence = run_data["sensor_sequence"]
                    success = float(run_data["success"])

                    # ----------------------------------------
                    # HITL
                    # ----------------------------------------

                    if method == "HITL":

                        matrix_llm = run_data["matrix_LLM"]
                        matrix_human = run_data["matrix_human"]

                        n_llm, _ = get_inspected_areas(matrix_llm)

                        n_human, inspected_vector = get_inspected_areas(matrix_human)

                        distance, distances_per_turn = normalized_hamming_distance(
                            matrix_llm,
                            matrix_human
                        )

                        matrix = matrix_human
                        n_inspected = n_human

                    # ----------------------------------------
                    # AUTO / ILP
                    # ----------------------------------------

                    else:

                        matrix = run_data["matrix"]

                        n_inspected, inspected_vector = get_inspected_areas(matrix)

                    # ----------------------------------------
                    # COMMON METRICS
                    # ----------------------------------------

                    n_total = run_data["n_total_areas"]

                    sequence_key = json.dumps(
                        sequence,
                        sort_keys=True
                    )

                    group_key = (
                        world,
                        sequence_key,
                        run_data["intruder_final_area"]
                    )

                    A_min = minimum_areas.get(group_key)

                    if A_min is not None and n_total > A_min:

                        efficiency = round(
                            success * (
                                1 - (n_inspected - A_min) /
                                (n_total - A_min)
                            ),
                            2
                        )

                    elif A_min is not None:

                        efficiency = success

                    else:

                        efficiency = 0.0

                    # ----------------------------------------
                    # BUILD RUN RESULT
                    # ----------------------------------------

                    run_result = {
                        "sensor_sequence": sequence,
                        "inspection_binary_vector": inspected_vector,
                    }

                    if method == "HITL":

                        run_result.update({
                            "n_inspected_areas_LLM": n_llm,
                            "n_inspected_areas_human": n_human,
                            "n_total_areas": n_total,
                            "min_inspected_areas": A_min,
                            "intruder_final_area": run_data["intruder_final_area"],
                            "success_rate": success,
                            "efficiency_human": round(efficiency,2),
                            "normalized_hamming_distance": round(distance, 2),
                            "normalized_hamming_distance_per_turn": {
                                turn: round(value, 2)
                                for turn, value in distances_per_turn.items()
                            }
                        })

                    else:
                        run_result.update({
                            "n_inspected_areas": n_inspected,
                            "n_total_areas": n_total,
                            "min_inspected_areas": A_min,
                            "intruder_final_area": run_data["intruder_final_area"],
                            "success_rate": success,
                            "efficiency" : round(efficiency,2)
                        })
                        # run_result["efficiency"] = efficiency

                    results[method][mode][world][trial][run] = run_result


# ============================================================
# 5. SAVE RESULTS
# ============================================================

with open(METRICS_OUTPUT_FILE, "w", encoding="utf-8") as file:

    json.dump(
        results,
        file,
        indent=4,
        ensure_ascii=False
    )

print(f"Run metrics saved to: {METRICS_OUTPUT_FILE}")




# ============================================================
# 6. JACCARD SIMILARITY
# ============================================================

def jaccard_similarity(vector_a, vector_b):

    areas = set(vector_a) | set(vector_b)

    intersection = sum(
        vector_a.get(area, 0) == 1 and
        vector_b.get(area, 0) == 1
        for area in areas
    )

    union = sum(
        vector_a.get(area, 0) == 1 or
        vector_b.get(area, 0) == 1
        for area in areas
    )

    return intersection / union if union > 0 else 1.0


# ============================================================
# 7. GROUP RUNS BY SENSOR SEQUENCE
# ============================================================

sequence_groups = {}

for method, method_data in results.items():

    for experiment_type, worlds in method_data.items():

        for world, trials in worlds.items():

            for trial, runs in trials.items():

                for run_data in runs.values():

                    sequence = tuple(run_data["sensor_sequence"])

                    sequence_key = ", ".join(sequence)

                    group_key = (
                        experiment_type,
                        world,
                        trial,
                        sequence_key
                    )

                    sequence_groups.setdefault(
                        group_key, {}
                    )

                    sequence_groups[group_key].setdefault(
                        method, []
                    )

                    sequence_groups[group_key][method].append(
                        run_data["inspection_binary_vector"]
                    )


# ============================================================
# 8. CALCULATE JACCARD FOR ALL METHOD PAIRS
# ============================================================

jaccard_results = {}

for (
    experiment_type,
    world,
    trial,
    sequence
), method_data in sequence_groups.items():

    jaccard_results.setdefault(experiment_type, {})
    jaccard_results[experiment_type].setdefault(world, {})
    jaccard_results[experiment_type][world].setdefault(
        trial, {}
    )

    pairwise_results = {}

    available_methods = [
        method
        for method in results.keys()
        if method in method_data
    ]

    for method_a, method_b in combinations(
        available_methods, 2
    ):

        pair_name = f"{method_a}-{method_b}"

        vector_a = method_data[method_a][0]
        vector_b = method_data[method_b][0]

        similarity = jaccard_similarity(
            vector_a,
            vector_b
        )

        pairwise_results[pair_name] = round(similarity, 4)

    jaccard_results[experiment_type][world][trial][sequence] = pairwise_results


# ============================================================
# 9. SAVE JACCARD RESULTS
# ============================================================

with open(JACCARD_OUTPUT_FILE, "w", encoding="utf-8") as file:

    json.dump(
        jaccard_results,
        file,
        indent=4,
        ensure_ascii=False
    )

print(f"Jaccard similarities saved to: {JACCARD_OUTPUT_FILE}")

# ============================================================
# 10. PROCESSING SUMMARY
# ============================================================

# print("\n" + "=" * 70)
# print("PROCESSING COMPLETED")
# print("=" * 70)

# print(f"Run metrics file: {METRICS_OUTPUT_FILE}")
# print(f"Jaccard file: {JACCARD_OUTPUT_FILE}")

# for experiment_type, worlds in jaccard_results.items():

#     n_sequences = sum(
#         len(sequences)
#         for trials in worlds.values()
#         for sequences in trials.values()
#     )

#     print(
#         f"{experiment_type}: "
#         f"{n_sequences} sensor sequences"
#     )