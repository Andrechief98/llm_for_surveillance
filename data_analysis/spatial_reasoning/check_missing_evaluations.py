
import json
import sys
import os
from pathlib import Path
from itertools import product

script_dir = os.path.dirname(__file__) 

def check_missing_evaluations(
    json_path,
    modalities,
    worlds,
    models,
    trials,
    runs
):
    # Legge il JSON
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Errore: file non trovato: {json_path}")
        return
    except json.JSONDecodeError as e:
        print(f"Errore nel formato JSON: {e}")
        return

    missing = []

    # Mantiene la struttura:
    # modalità -> world -> modello -> trial -> run
    for modality, world, model, trial, run in product(
        modalities, worlds, models, trials, runs
    ):
        path = (
            data.get(modality, {})
                .get(world, {})
                .get(model, {})
                .get(trial, {})
        )

        # Una valutazione manca se la run non esiste
        # oppure se è vuota o null
        if run not in path or path[run] is None or path[run] == {}:
            missing.append({
                "modalità": modality,
                "world": world,
                "modello": model,
                "trial": trial,
                "run": run
            })

    # Stampa i risultati
    if not missing:
        print("Tutte le valutazioni sono presenti!")
        return

    print(f"Valutazioni mancanti: {len(missing)}\n")

    for item in missing:
        print(
            f"{item['modalità']} / "
            f"{item['world']} / "
            f"{item['modello']} / "
            f"{item['trial']} / "
            f"{item['run']}"
        )

    print(f"\nTotale valutazioni mancanti: {len(missing)}")


if __name__ == "__main__":
    # Sostituisci questi nomi con quelli effettivi nel tuo JSON
    modalities = [
        "rag_with_functions",
        "rag_with_semantics",
        "image",
        "image_and_rag_with_functions",
        "image_and_rag_with_semantics"
    ]

    worlds = ["world_1", "world_2"]

    models = ["gpt-4o", "gpt-6-luna"]

    trials = ["trial_1", "trial_2", "trial_3"]

    runs = ["run_1", "run_2", "run_3", "run_4"]

    json_path = os.path.join(script_dir, "./evaluations.json")

    check_missing_evaluations(
        json_path,
        modalities,
        worlds,
        models,
        trials,
        runs
    )
