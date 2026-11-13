
import json
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent

INPUT_FILE = BASE_DIR / "raw_data/experiments_ILP.json"
OUTPUT_FILE = BASE_DIR / "inspection_matrices/inspection_matrices_ILP.json"

RAG_FILES = {
    "world_1": BASE_DIR / "../../llm_interface/config/RAG/RAG_map1.json",
    "world_2": BASE_DIR / "../../llm_interface/config/RAG/RAG_map2.json"
}


# Load world areas
world_areas = {}

for world, path in RAG_FILES.items():

    with open(path, "r", encoding="utf-8") as file:
        world_areas[world] = list(json.load(file)["areas"].keys())


# Load ILP results
with open(INPUT_FILE, "r", encoding="utf-8") as file:
    data = json.load(file)


# Extract inspection matrices
results = {}

for experiment_type, methods in data.items():

    for method, models in methods.items():

        for model, worlds in models.items():

            model_name = f"{method}-{model}"

            results.setdefault(model_name, {})

            for world, trials in worlds.items():

                areas = world_areas[world]

                for trial, runs in trials.items():

                    for run, run_data in runs.items():

                        matrix = {}

                        for turn, actions in enumerate(
                            run_data["actions"], start=1
                        ):

                            matrix[str(turn)] = {
                                area: 0.0 for area in areas
                            }

                            for area in actions["deploy"].values():

                                if area in areas:
                                    matrix[str(turn)][area] = 10.0

                        n_inspected = sum(
                            any(
                                turn[area] > 0
                                for turn in matrix.values()
                            )
                            for area in areas
                        )

                        results[model_name] \
                            .setdefault(experiment_type, {}) \
                            .setdefault(world, {}) \
                            .setdefault(trial, {})[run] = {

                                "sensor_sequence": run_data["sensor_sequence"],
                                "intruder_final_area": run_data["intruder_final_area"],
                                "success": run_data["success"],
                                "matrix": matrix,
                                "n_inspected_areas": n_inspected,
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

print(f"ILP matrices saved to: {OUTPUT_FILE}")
