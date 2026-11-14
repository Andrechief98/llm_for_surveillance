
import json
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

INPUT_FILE = Path("robot_deployments.json")
DISTANCES_FILE = Path("robot_area_distances.json")

EVALUATIONS_FILE = Path("evaluations.json")
COMPARISON_FILE = Path("deployment_comparison.json")

ERROR_CATEGORY = "Reasoning_errors"
ERROR_NAME = "unoptimal robot deployment"

# Tolleranza percentuale
TOLERANCE = 0.5


# ============================================================
# SINGLE AREA EVALUATION
# ============================================================

def evaluate_single_area(deployment, computed_distance):
    """
    Verifica se il robot selezionato per una singola area
    è quello che determina la distanza minima.
    """

    robot, area = next(iter(deployment.items()))

    if area not in computed_distance:
        raise ValueError(
            f"Area '{area}' non presente in computed_distance"
        )

    distances = computed_distance[area]

    if robot not in distances:
        raise ValueError(
            f"Distanza di '{robot}' per l'area '{area}' non disponibile"
        )

    actual_distance = distances[robot]

    best_robot = min(distances, key=distances.get)
    minimum_distance = distances[best_robot]

    if actual_distance > minimum_distance:
        result = "inefficient"
    else:
        result = "efficient"

    return {
        "actual_deployment": {
            robot: {
                "area": area,
                "distance": actual_distance
            }
        },

        "actual_total_distance": round(actual_distance, 3),

        "inverse_deployment": None,
        "inverse_total_distance": None,

        "best_robot": best_robot,
        "minimum_distance": round(minimum_distance, 3),

        "difference_percentage": (
            round(
                ((actual_distance - minimum_distance) / minimum_distance) * 100,
                2
            ) if minimum_distance > 0
            else (0.0 if actual_distance == 0 else float("inf"))
        ),

        "tolerance": TOLERANCE,
        "result": result
    }


# ============================================================
# CALCULATE DEPLOYMENT DISTANCE
# ============================================================

def calculate_deployment_distance(deployment, computed_distance):
    """Calcola la distanza totale di un deployment."""

    total_distance = 0.0
    details = {}

    for robot, area in deployment.items():

        if area not in computed_distance:
            raise ValueError(
                f"Area '{area}' non presente in computed_distance"
            )

        if robot not in computed_distance[area]:
            raise ValueError(
                f"Distanza di '{robot}' per l'area '{area}' non disponibile"
            )

        distance = computed_distance[area][robot]

        details[robot] = {
            "area": area,
            "distance": distance
        }

        total_distance += distance

    return total_distance, details


# ============================================================
# INVERT DEPLOYMENT
# ============================================================

def invert_deployment(deployment):
    """Inverte ciclicamente le aree assegnate ai robot."""

    robots = list(deployment.keys())
    areas = list(deployment.values())

    if len(set(areas)) != len(areas):
        raise ValueError(
            "Impossibile invertire il deployment: aree duplicate"
        )

    return {
        robot: areas[-(i + 1)]
        for i, robot in enumerate(robots)
    }


# ============================================================
# EVALUATE TURN
# ============================================================

def evaluate_turn(turn_data, computed_distance):

    deployment = turn_data.get("deployments", {})

    if not deployment:
        return None

    # --------------------------------------------------------
    # CASE 1: SINGLE AREA
    # --------------------------------------------------------

    if len(set(deployment.values())) == 1:

        return evaluate_single_area(
            deployment,
            computed_distance
        )

    # --------------------------------------------------------
    # CASE 2: MULTIPLE AREAS
    # --------------------------------------------------------

    actual_distance, actual_details = calculate_deployment_distance(
        deployment,
        computed_distance
    )

    inverse_deployment = invert_deployment(deployment)

    inverse_distance, inverse_details = calculate_deployment_distance(
        inverse_deployment,
        computed_distance
    )

    if inverse_distance > 0:

        difference_percentage = (
            (actual_distance - inverse_distance)
            / inverse_distance
        ) * 100

    elif actual_distance > 0:

        difference_percentage = float("inf")

    else:

        difference_percentage = 0.0

    if difference_percentage > TOLERANCE:
        result = "inefficient"

    elif difference_percentage < -TOLERANCE:
        result = "efficient"

    else:
        result = "equivalent"

    return {
        "actual_deployment": actual_details,
        "actual_total_distance": round(actual_distance, 3),

        "inverse_deployment": inverse_details,
        "inverse_total_distance": round(inverse_distance, 3),

        "difference_percentage": round(difference_percentage, 2),

        "tolerance": TOLERANCE,
        "result": result
    }


# ============================================================
# UPDATE REASONING ERRORS
# ============================================================

def update_reasoning_errors(run_evaluation, inefficient_count):
    """
    Ricalcola i contatori dei Reasoning_errors senza
    incrementare i valori precedentemente salvati.
    """

    error_counts = run_evaluation.setdefault("error_counts", {})

    reasoning_errors = error_counts.setdefault(
        ERROR_CATEGORY,
        {}
    )

    subcategories = reasoning_errors.setdefault(
        "subcategories",
        {}
    )

    subcategories.setdefault(ERROR_NAME, 0)

    subcategories[ERROR_NAME] = inefficient_count

    reasoning_total = sum(subcategories.values())

    reasoning_errors["total"] = reasoning_total

    macro_totals = run_evaluation.setdefault("macro_totals", {})

    macro_totals[ERROR_CATEGORY] = reasoning_total

    run_evaluation["total_errors"] = sum(
        macro_totals.values()
    )


# ============================================================
# PROCESS EXPERIMENTS
# ============================================================

def process_experiments(
    processed_data,
    evaluations,
    distances_data
):

    comparison_results = {}

    inefficient_deployments = 0
    evaluated_turns = 0

    # Global counter for each model.
    inefficient_by_model = {}

    for experiment, worlds in processed_data.items():

        comparison_results[experiment] = {}

        for world, models in worlds.items():

            comparison_results[experiment][world] = {}

            # ------------------------------------------------
            # Select distances for current world.
            # ------------------------------------------------

            computed_distance = distances_data.get(world, {})

            if not computed_distance:

                print(
                    f"[WARNING] No distances available for {world}"
                )

            for model, trials in models.items():

                comparison_results[experiment][world][model] = {}

                # Initialize model counter.
                inefficient_by_model.setdefault(model, 0)

                for trial, runs in trials.items():

                    comparison_results[experiment][world][model][trial] = {}

                    for run, run_data in runs.items():

                        comparison_results[
                            experiment
                        ][
                            world
                        ][
                            model
                        ][
                            trial
                        ][run] = {}

                        try:

                            run_evaluation = (
                                evaluations
                                .setdefault(experiment, {})
                                .setdefault(world, {})
                                .setdefault(model, {})
                                .setdefault(trial, {})
                                .setdefault(run, {})
                            )

                        except (TypeError, AttributeError):

                            print(
                                f"Struttura non valida: "
                                f"{experiment}/{world}/{model}/"
                                f"{trial}/{run}"
                            )

                            continue

                        # Counter local to the current run.
                        run_inefficient_count = 0

                        for turn, turn_data in run_data.items():

                            if not turn.startswith("turn_"):
                                continue

                            try:

                                result = evaluate_turn(
                                    turn_data,
                                    computed_distance
                                )

                                if result is None:
                                    continue

                                evaluated_turns += 1

                                # --------------------------------
                                # Save detailed comparison.
                                # --------------------------------

                                comparison_results[
                                    experiment
                                ][
                                    world
                                ][
                                    model
                                ][
                                    trial
                                ][
                                    run
                                ][turn] = result

                                if result["inverse_total_distance"] is not None:

                                    comparison_text = (
                                        f"effettivo: {result['actual_total_distance']:.2f}, "
                                        f"inverso: {result['inverse_total_distance']:.2f}"
                                    )

                                else:

                                    comparison_text = (
                                        f"effettivo: {result['actual_total_distance']:.2f}, "
                                        f"robot migliore: {result['best_robot']}, "
                                        f"distanza minima: {result['minimum_distance']:.2f}"
                                    )

                                print(
                                    f"{experiment}/{world}/{model}/"
                                    f"{trial}/{run}/{turn}: "
                                    f"{result['result']} "
                                    f"({comparison_text}, "
                                    f"differenza: {result['difference_percentage']}%, "
                                    f"tolleranza: {TOLERANCE}%)"
                                )

                                # --------------------------------
                                # Count inefficient deployments.
                                # --------------------------------

                                if result["result"] == "inefficient":

                                    run_inefficient_count += 1
                                    inefficient_deployments += 1

                                    inefficient_by_model[model] += 1

                            except (ValueError, KeyError, TypeError) as e:

                                print(
                                    f"Errore in "
                                    f"{experiment}/{world}/{model}/"
                                    f"{trial}/{run}/{turn}: {e}"
                                )

                        # ----------------------------------------
                        # Update evaluation counters for run.
                        # ----------------------------------------

                        update_reasoning_errors(
                            run_evaluation,
                            run_inefficient_count
                        )

    return (
        comparison_results,
        evaluated_turns,
        inefficient_deployments,
        inefficient_by_model
    )


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # 1. Read deployment data.
    # --------------------------------------------------------

    with INPUT_FILE.open("r", encoding="utf-8") as file:
        processed_data = json.load(file)

    # --------------------------------------------------------
    # 2. Read distance database.
    # --------------------------------------------------------

    with DISTANCES_FILE.open("r", encoding="utf-8") as file:
        distances_data = json.load(file)

    # --------------------------------------------------------
    # 3. Read existing evaluations.
    # --------------------------------------------------------

    with EVALUATIONS_FILE.open("r", encoding="utf-8") as file:
        evaluations = json.load(file)

    # --------------------------------------------------------
    # 4. Process experiments.
    # --------------------------------------------------------

    (
        comparison_results,
        evaluated,
        inefficient,
        inefficient_by_model
    ) = process_experiments(
        processed_data,
        evaluations,
        distances_data
    )

    # --------------------------------------------------------
    # 5. Update evaluations.json.
    # --------------------------------------------------------

    with EVALUATIONS_FILE.open("w", encoding="utf-8") as file:

        json.dump(
            evaluations,
            file,
            indent=4,
            ensure_ascii=False,
            allow_nan=False
        )

    # --------------------------------------------------------
    # 6. Save deployment comparison.
    # --------------------------------------------------------

    with COMPARISON_FILE.open("w", encoding="utf-8") as file:

        json.dump(
            comparison_results,
            file,
            indent=4,
            ensure_ascii=False,
            allow_nan=False
        )

    # --------------------------------------------------------
    # 7. Final summary.
    # --------------------------------------------------------

    print("\n" + "=" * 60)
    print("DEPLOYMENT EVALUATION SUMMARY")
    print("=" * 60)

    print(f"Tolleranza applicata: {TOLERANCE}%")
    print(f"Turni valutati: {evaluated}")
    print(f"Deployment inefficienti: {inefficient}")
    print(f"Errori aggiunti: {inefficient}")

    # --------------------------------------------------------
    # 8. Inefficient deployments by model.
    # --------------------------------------------------------

    print("\n" + "-" * 60)
    print("INEFFICIENT DEPLOYMENTS BY MODEL")
    print("-" * 60)

    for model, count in inefficient_by_model.items():

        print(
            f"{model}: {count} inefficient deployments"
        )

    # --------------------------------------------------------
    # 9. Output files.
    # --------------------------------------------------------

    print(f"\nFile aggiornato: {EVALUATIONS_FILE}")
    print(f"File creato: {COMPARISON_FILE}")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
