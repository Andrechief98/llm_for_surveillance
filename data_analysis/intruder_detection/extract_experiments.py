
import json
from pathlib import Path
from pprint import pprint

# Directory dello script
SCRIPT_DIR = Path(__file__).resolve().parent
INPUT_FILE = SCRIPT_DIR / "experiments_HITL.json"


def extract_data():

    with open(INPUT_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    uncertain = data.get("uncertain", {})

    results = {}

    for world in ["world_1", "world_2"]:

        world_data = uncertain.get("HITL", {}).get(world, {})

        results[world] = {}

        for model, trials in world_data.items():

            for trial, runs in trials.items():

                results[world].setdefault(trial, {})

                for run, run_data in runs.items():

                    # Estrazione diretta della sequenza
                    sensor_sequence = run_data.get(
                        "sensor_sequence", []
                    )

                    # Estrazione dell'area finale
                    final_area = run_data.get(
                        "intruder_final_area", None
                    )

                    # Associazione sequenza -> area finale
                    sensor_key = tuple(sensor_sequence)

                    results[world][trial].setdefault(
                        sensor_key, []
                    )

                    results[world][trial][sensor_key].append(
                        final_area
                    )

        # Ordinamento per numero crescente di sensori
        for trial in results[world]:

            results[world][trial] = dict(
                sorted(
                    results[world][trial].items(),
                    key=lambda item: (len(item[0]), item[0])
                )
            )

    return results


if __name__ == "__main__":

    results = extract_data()

    pprint(results, sort_dicts=False)
