import json
from pathlib import Path
from typing import Any, List, Dict, Tuple

# ============================================================
# CONFIGURAZIONE: MODIFICA QUI I PERCORSI DEI FILE JSON
# ============================================================
BASE_DIR = Path(__file__).resolve().parent

EXPERIMENTS_PATH = BASE_DIR / "experiments.json"
EVALUATIONS_PATH = BASE_DIR / "evaluations.json"

UNAPPROVED_ACTION_STRINGS = [
    "show_camera_to_user",
    "send_robots_to_area",
]


FORBIDDEN_DISTANCE_AREAS = {
    "world_1": {"B", "F"},
    "world_2": {"A", "B", "C", "D", "E", "F", "I", "U", "O", "Q"},
}


# ============================================================
# LETTURA DEI FILE
# ============================================================

def load_json(file_path: Path) -> dict:
    """Carica un file JSON senza modificarlo."""
    with file_path.open("r", encoding="utf-8") as file:
        return json.load(file)


# ============================================================
# CONTROLLO DELLA CONVERSAZIONE
# ============================================================


def count_unapproved_action_execution(
    conversation: List[Dict],
) -> int:
    """
    Conta le function_call il cui nome corrisponde a una delle
    azioni configurate come non autorizzate.
    """
    total_count = 0

    for message in conversation:
        function_call = message.get("function_call")

        if isinstance(function_call, str):
            for action_string in UNAPPROVED_ACTION_STRINGS:
                for line in function_call.splitlines():
                    line = line.strip()

                    if line.startswith("name -"):
                        function_name = line[len("name -"):].strip()

                        if function_name == action_string:
                            total_count += 1
                            break

        elif isinstance(function_call, dict):
            function_name = function_call.get("name")

            if function_name in UNAPPROVED_ACTION_STRINGS:
                total_count += 1

    return total_count

# ============================================================
# ACCESSO AI DATI ANNIDATI
# ============================================================

def get_nested_value(
    data: Dict,
    keys: Tuple[str, ...],
    default: Any = None,
) -> Any:
    """Recupera un valore da un dizionario annidato."""
    current = data

    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default

        current = current[key]

    return current


# ============================================================
# VALIDAZIONE DELLE VALUTAZIONI
# ============================================================


def validate_unapproved_action_execution(
    experiments: Dict,
    evaluations: Dict,
) -> List[Dict]:
    """
    Confronta il conteggio deterministico con quello delle
    valutazioni per ogni combinazione:
    modality / world / model / trial / run.

    Non modifica i dati né i file originali.
    """
    discrepancies = []

    for modality, worlds in experiments.items():
        if not isinstance(worlds, dict):
            continue

        for world, models in worlds.items():
            if not isinstance(models, dict):
                continue

            for model, trials in models.items():
                if not isinstance(trials, dict):
                    continue

                for trial, runs in trials.items():
                    if not isinstance(runs, dict):
                        continue

                    for run, run_data in runs.items():
                        if not isinstance(run_data, dict):
                            continue

                        conversation = run_data.get("conversation", [])

                        if not isinstance(conversation, list):
                            print(
                                f"[WARNING] Conversazione non valida: "
                                f"{modality}/{world}/{model}/{trial}/{run}"
                            )
                            continue

                        calculated_count = (
                            count_unapproved_action_execution(conversation)
                        )

                        evaluation_run = get_nested_value(
                            evaluations,
                            (modality, world, model, trial, run),
                        )

                        if evaluation_run is None:
                            print(
                                f"[WARNING] Run assente nelle valutazioni: "
                                f"{modality}/{world}/{model}/{trial}/{run}"
                            )
                            continue

                        evaluated_count = get_nested_value(
                            evaluation_run,
                            (
                                "error_counts",
                                "Wrong_function_calls",
                                "subcategories",
                                "Unapproved action execution",
                            ),
                        )

                        if evaluated_count is None:
                            print(
                                f"[WARNING] Conteggio assente nelle "
                                f"valutazioni: "
                                f"{modality}/{world}/{model}/{trial}/{run}"
                            )
                            continue

                        if calculated_count != evaluated_count:
                            discrepancy = {
                                "modality": modality,
                                "world": world,
                                "model": model,
                                "trial": trial,
                                "run": run,
                                "calculated": calculated_count,
                                "evaluated": evaluated_count,
                            }

                            discrepancies.append(discrepancy)

                            print(
                                "\n[DISCREPANZA] "
                                "Unapproved action execution\n"
                                f"  Modalità:          {modality}\n"
                                f"  World:             {world}\n"
                                f"  Modello:           {model}\n"
                                f"  Trial:             {trial}\n"
                                f"  Run:               {run}\n"
                                f"  Conteggio calcolato: {calculated_count}\n"
                                f"  Conteggio valutato:   {evaluated_count}"
                            )
                        

    return discrepancies



def count_compute_distance_wrong_areas(
    conversation: List[Dict],
    world: str,
) -> int:
    """
    Conta le chiamate compute_robot_area_distance in cui
    areas_to_inspect contiene almeno un'area vietata per il mondo.

    Ogni chiamata viene contata al massimo una volta, anche se
    contiene più aree vietate.
    """
    forbidden_areas = FORBIDDEN_DISTANCE_AREAS.get(world, set())

    if not forbidden_areas:
        return 0

    error_count = 0

    for message in conversation:
        function_call = message.get("function_call")

        if not isinstance(function_call, str):
            continue

        if "name - compute_robot_area_distance" not in function_call:
            continue

        try:
            arguments_text = function_call.split(
                "arguments -", 1
            )[1].strip()

            arguments = json.loads(arguments_text)
            areas_to_inspect = arguments.get("areas_to_inspect", [])

            if not isinstance(areas_to_inspect, list):
                continue

            if forbidden_areas.intersection(areas_to_inspect):
                error_count += 1

        except (IndexError, json.JSONDecodeError):
            print(
                "[WARNING] Impossibile interpretare gli argomenti "
                "di compute_robot_area_distance."
            )

    return error_count


def validate_compute_distance_wrong_areas(
    experiments: Dict,
    evaluations: Dict,
) -> List[Dict]:
    """
    Confronta il conteggio delle chiamate compute_robot_area_distance
    con aree vietate con quello presente nelle valutazioni.
    """
    discrepancies = []

    for modality, worlds in experiments.items():
        if not isinstance(worlds, dict):
            continue

        for world, models in worlds.items():
            if not isinstance(models, dict):
                continue

            for model, trials in models.items():
                if not isinstance(trials, dict):
                    continue

                for trial, runs in trials.items():
                    if not isinstance(runs, dict):
                        continue

                    for run, run_data in runs.items():
                        if not isinstance(run_data, dict):
                            continue

                        conversation = run_data.get("conversation", [])

                        if not isinstance(conversation, list):
                            continue

                        calculated_count = count_compute_distance_wrong_areas(
                            conversation,
                            world,
                        )

                        evaluation_run = get_nested_value(
                            evaluations,
                            (modality, world, model, trial, run),
                        )

                        if evaluation_run is None:
                            print(
                                f"[WARNING] Run assente nelle valutazioni: "
                                f"{modality}/{world}/{model}/{trial}/{run}"
                            )
                            continue

                        evaluated_count = get_nested_value(
                            evaluation_run,
                            (
                                "error_counts",
                                "Hallucinations",
                                "subcategories",
                                "computed_distance_to_area_covered_by_camera",
                            ),
                        )

                        if evaluated_count is None:
                            print(
                                f"[WARNING] Sottocategoria assente: "
                                f"{modality}/{world}/{model}/{trial}/{run} "
                                "computed_distance_to_area_covered_by_camera"
                            )
                            continue

                        if calculated_count != evaluated_count:
                            discrepancy = {
                                "modality": modality,
                                "world": world,
                                "model": model,
                                "trial": trial,
                                "run": run,
                                "calculated": calculated_count,
                                "evaluated": evaluated_count,
                            }

                            discrepancies.append(discrepancy)

                            print(
                                "\n[DISCREPANZA] "
                                "compute_robot_area_distance\n"
                                f"  Modalità:            {modality}\n"
                                f"  World:               {world}\n"
                                f"  Modello:             {model}\n"
                                f"  Trial:               {trial}\n"
                                f"  Run:                 {run}\n"
                                f"  Conteggio calcolato: {calculated_count}\n"
                                f"  Conteggio valutato:  {evaluated_count}"
                            )

    return discrepancies
# ============================================================
# MAIN
# ============================================================

def main():
    if not EXPERIMENTS_PATH.is_file():
        raise FileNotFoundError(
            f"File degli esperimenti non trovato: {EXPERIMENTS_PATH}"
        )

    if not EVALUATIONS_PATH.is_file():
        raise FileNotFoundError(
            f"File delle valutazioni non trovato: {EVALUATIONS_PATH}"
        )

    experiments = load_json(EXPERIMENTS_PATH)
    evaluations = load_json(EVALUATIONS_PATH)

    # Controllo 1: azioni eseguite senza autorizzazione
    unapproved_discrepancies = validate_unapproved_action_execution(
        experiments,
        evaluations,
    )

    # Controllo 2: aree errate nelle chiamate compute_robot_area_distance
    distance_discrepancies = validate_compute_distance_wrong_areas(
        experiments,
        evaluations,
    )

    print("\n========== RIEPILOGO ==========")
    print(
        "Discrepanze Unapproved action execution: "
        f"{len(unapproved_discrepancies)}"
    )
    print(
        "Discrepanze compute_robot_area_distance: "
        f"{len(distance_discrepancies)}"
    )


if __name__ == "__main__":
    main()