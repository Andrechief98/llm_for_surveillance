import json

json_path = "./llm_data_analysis/data.json"

def check_consistency(data):
    errors = []

    for sheet_name, entries in data.items():
        for idx, entry in enumerate(entries):
            seq = entry.get("sensors_sequence", [])
            for trial_idx, trial in enumerate(entry.get("trials", [])):
                annotations = trial.get("Annotations", {})
                
                # Somma delle HALLUCINATIONS
                hallucinations = annotations.get("HALLUCINATION", [])
                sum_hallucinations = sum([list(h.values())[0] for h in hallucinations])

                # Somma degli PLANNING ERROR
                error_plans = annotations.get("PLANNING ERROR", [])
                sum_error_plan = sum([list(e.values())[0] for e in error_plans])

                # Somma degli ACTION ERROR
                error_actions = annotations.get("ACTION ERROR", [])
                sum_error_action = sum([list(e.values())[0] for e in error_actions])

                # Valori di riferimento dalle metriche
                tot_all = trial.get("TOT HAL", 0)
                ep = trial.get("PE", 0)
                ea = trial.get("AE", 0)

                # Verifica coerenza
                if sum_hallucinations != tot_all:
                    errors.append({
                        "sheet": sheet_name,
                        "sensors_sequence": seq,
                        "trial": trial_idx + 1,
                        "metric": "TOT HAL",
                        "expected": tot_all,
                        "found": sum_hallucinations
                    })

                if sum_error_plan != ep:
                    errors.append({
                        "sheet": sheet_name,
                        "sensors_sequence": seq,
                        "trial": trial_idx + 1,
                        "metric": "PE",
                        "expected": ep,
                        "found": sum_error_plan
                    })

                if sum_error_action != ea:
                    errors.append({
                        "sheet": sheet_name,
                        "sensors_sequence": seq,
                        "trial": trial_idx + 1,
                        "metric": "AE",
                        "expected": ea,
                        "found": sum_error_action
                    })

    return errors


try:
    with open(json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)

    errors = check_consistency(json_data)

    if errors:
        print("Discrepanze trovate:")
        for e in errors:
            print(f"- [Sheet: {e['sheet']}] Trial {e['trial']} in sequence {e['sensors_sequence']}: "
                  f"{e['metric']} = {e['expected']} (found {e['found']})")
    else:
        print("All metrics are coherent.")

except Exception as e:
    print(f"Error during the check: {e}")
