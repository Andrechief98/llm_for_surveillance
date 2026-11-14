
import json
import re

input_file = "evaluations.json"
output_file = "evaluations.json"

with open(input_file, "r", encoding="utf-8") as f:
    data = json.load(f)


def sort_runs(data):
    for rag in data.values():
        for world in rag.values():
            for model in world.values():
                for trial in model.values():

                    # Identifica le chiavi run_X
                    run_keys = [
                        key for key in trial.keys()
                        if re.fullmatch(r"run_\d+", key)
                    ]

                    # Ordina numericamente le run
                    sorted_run_keys = sorted(
                        run_keys,
                        key=lambda x: int(x.split("_")[1])
                    )

                    # Ricostruisce il dizionario mantenendo
                    # invariati tutti i valori
                    sorted_trial = {}

                    for key in trial:
                        if not re.fullmatch(r"run_\d+", key):
                            sorted_trial[key] = trial[key]

                    for key in sorted_run_keys:
                        sorted_trial[key] = trial[key]

                    trial.clear()
                    trial.update(sorted_trial)

    return data


data = sort_runs(data)

with open(output_file, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2, ensure_ascii=False)

print("Riordinamento completato!")
