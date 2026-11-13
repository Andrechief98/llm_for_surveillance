
import json
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent

INPUT_FILE = BASE_DIR / "processed_HITL_data/raw_inspection_matrices.json"
OUTPUT_FILE = BASE_DIR / "inspection_matrices/inspection_matrices_HITL.json"

RAG_FILES = {
    "world_1": BASE_DIR / "../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": BASE_DIR / "../../llm_interface/config/RAG/RAG_map2.json"
}


# Load world areas
world_areas = {}

for world, path in RAG_FILES.items():

    with open(path, "r", encoding="utf-8") as file:
        world_areas[world] = list(json.load(file)["areas"].keys())


# Load HITL results
with open(INPUT_FILE, "r", encoding="utf-8") as file:
    data = json.load(file)


# Convert inspection values
def inspection_value(value):

    if value.startswith("robot"):
        return 10.0

    if value.lower().startswith("camera"):
        return 1.0

    return 0.0


# Extract matrices
results = {}

for experiment_type, methods in data.items():

    for method, worlds in methods.items():

        for world, models in worlds.items():

            areas = world_areas[world]

            for model, trials in models.items():

                for trial, runs in trials.items():

                    for run, run_data in runs.items():

                        matrix_llm = {}
                        matrix_human = {}

                        for turn, turn_data in run_data["matrix"].items():

                            matrix_llm[turn] = {
                                area: inspection_value(
                                    turn_data["LLM"].get(area, "")
                                )
                                for area in areas
                            }

                            matrix_human[turn] = {
                                area: inspection_value(
                                    turn_data["human"].get(area, "")
                                )
                                for area in areas
                            }

                        n_inspected_llm = sum(
                            any(turn[area] > 0 for turn in matrix_llm.values())
                            for area in areas
                        )

                        n_inspected_human = sum(
                            any(turn[area] > 0 for turn in matrix_human.values())
                            for area in areas
                        )

                        results.setdefault("HITL", {}) \
                            .setdefault(experiment_type, {}) \
                            .setdefault(world, {}) \
                            .setdefault(trial, {})[run] = {

                                "sensor_sequence": run_data["sensor_sequence"],

                                "intruder_final_area": run_data["intruder_final_area"],

                                "success": run_data["success"],

                                "matrix_LLM": matrix_llm,

                                "matrix_human": matrix_human,

                                "n_inspected_areas_LLM": n_inspected_llm,

                                "n_inspected_areas_human": n_inspected_human,

                                "n_total_areas": len(areas)
                            }


# Save results
with open(OUTPUT_FILE, "w", encoding="utf-8") as file:

    json.dump(
        results,
        file,
        indent=4,
        ensure_ascii=False
    )

print(f"HITL-custom matrices saved to: {OUTPUT_FILE}")
