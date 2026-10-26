import os
import json
import math
import re
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CONFIGURAZIONE
# ============================================================
script_dir = os.path.dirname(__file__)

JSON_PATH = os.path.join(script_dir, "evaluations.json")
OUTPUT_DIR = Path(os.path.join(script_dir,"results"))

FIGURE_NAME = "spatial_reasoning"

FIGURE_SIZE_PER_SUBPLOT = (6.5, 5)
DPI = 300

# Varianza campionaria: ddof=1
VARIANCE_DDOF = 1
# ddof=1 — varianza campionaria: utile se consideriamo le run osservate come un campione e vogliamo stimare la variabilità del comportamento del modello in generale.

# ddof=0 — varianza della popolazione osservata: utile se vogliamo semplicemente descrivere la dispersione delle run che hai effettivamente eseguito, senza stimare una popolazione più ampia.

# Se True, mostra la figura a schermo.
SHOW_PLOT = True


# ============================================================
# FUNZIONI DI SUPPORTO
# ============================================================

def natural_key(value):
    """Ordina correttamente nomi come world_2, world_10."""
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", str(value))
    ]


def count_sensors(sensor_sequence):
    """
    Conta i sensori distinti nella sequenza.

    Gestisce, ad esempio:
    ["Lidar_4, Lidar_3, Camera_1, Lidar_2"]

    e anche:
    ["Lidar_4", "Lidar_3", "Camera_1", "Lidar_2"]
    """
    if not sensor_sequence:
        return 0

    if isinstance(sensor_sequence, str):
        sensor_sequence = [sensor_sequence]

    sensors = []

    for item in sensor_sequence:
        if not isinstance(item, str):
            continue

        sensors.extend(
            sensor.strip()
            for sensor in item.split(",")
            if sensor.strip()
        )

    # Conta ogni sensore una sola volta.
    return len(set(sensors))


def get_total_errors(run_data):
    """Legge total_errors oppure ricava il totale da error_counts."""
    if run_data.get("total_errors") is not None:
        return float(run_data["total_errors"])

    error_counts = run_data.get("error_counts", {})

    return float(sum(
        value
        for value in error_counts.values()
        if isinstance(value, (int, float))
    ))


def get_assistant_messages(run_data):
    """Restituisce il numero di messaggi dell'assistente."""
    message_counts = run_data.get("message_counts", {})
    value = message_counts.get("assistant")

    if value is None:
        return None

    return float(value)


# ============================================================
# LETTURA E FLATTENING DEL JSON
# ============================================================

def load_runs(json_path):
    with open(json_path, "r", encoding="utf-8") as file:
        data = json.load(file)

    records = []

    # Ogni chiave di primo livello è una modalità:
    # rag_with_functions, rag_with_semantics, ecc.
    for mode, worlds in data.items():

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

                    for run_id, run_data in runs.items():

                        if not isinstance(run_data, dict):
                            continue

                        n_sensors = count_sensors(
                            run_data.get("sensor_sequence", [])
                        )

                        errors = get_total_errors(run_data)
                        assistant_messages = get_assistant_messages(
                            run_data
                        )

                        # Se manca il denominatore o è zero,
                        # il tasso di errore non è definito.
                        if (
                            assistant_messages is None
                            or assistant_messages <= 0
                        ):
                            error_rate = np.nan
                        else:
                            error_rate = errors / assistant_messages

                        records.append({
                            "mode": mode,
                            "world": world,
                            "model": model,
                            "trial": trial,
                            "run": run_id,
                            "n_sensors": n_sensors,
                            "total_errors": errors,
                            "assistant_messages": assistant_messages,
                            "error_rate": error_rate,
                        })

    df = pd.DataFrame(records)

    if df.empty:
        raise ValueError(
            "Non sono stati trovati dati validi nel JSON."
        )

    invalid_sensors = df["n_sensors"] <= 0

    if invalid_sensors.any():
        warnings.warn(
            f"{invalid_sensors.sum()} run senza sensori validi "
            "saranno escluse dalle statistiche."
        )

    df = df[
        (~invalid_sensors)
        & df["error_rate"].notna()
    ].copy()

    if df.empty:
        raise ValueError(
            "Nessuna run utilizzabile dopo i controlli."
        )

    return df


# ============================================================
# STATISTICHE: MEDIA E VARIANZA
# ============================================================

def compute_statistics(df):
    """
    Calcola media, varianza campionaria e numero di osservazioni
    per modalità, modello e numero di sensori.

    Ogni osservazione è il tasso di errore di una singola run.
    """
    group_columns = ["mode", "model", "n_sensors"]

    # summary = (
    #     df.groupby(group_columns)["error_rate"]
    #     .agg(
    #         mean_error_rate="mean",
    #         variance_error_rate=lambda x: x.var(
    #             ddof=VARIANCE_DDOF
    #         ),
    #         std_error_rate="std",
    #         n_observations="count",
    #     )
    #     .reset_index()
    # )

    summary = (
        df.groupby(["mode", "model", "n_sensors"])["error_rate"]
        .agg(
            mean_error_rate="mean",
            std_deviation="std",        # Già calcolata di default partendo dalla varianza campionaria
            n_observations="count",
        )
        .reset_index()
    )

    return summary


# ============================================================
# GRAFICI
# ============================================================

def plot_spatial_reasoning(summary, output_dir):
    modes = sorted(
        summary["mode"].unique(),
        key=natural_key
    )

    models = sorted(
        summary["model"].unique(),
        key=natural_key
    )

    # Stessi colori per lo stesso modello in ogni subplot.
    cmap = plt.get_cmap("tab10", max(len(models), 1))
    model_colors = {
        model: cmap(i)
        for i, model in enumerate(models)
    }

    n_modes = len(modes)
    ncols = min(3, n_modes)
    nrows = math.ceil(n_modes / ncols)

    fig, axes = plt.subplots(
        nrows=nrows,
        ncols=ncols,
        figsize=(
            FIGURE_SIZE_PER_SUBPLOT[0] * ncols,
            FIGURE_SIZE_PER_SUBPLOT[1] * nrows,
        ),
        squeeze=False,
        
    )

    axes = axes.flatten()

    for ax, mode in zip(axes, modes):
        mode_data = summary[summary["mode"] == mode]

        for model in models:
            model_data = mode_data[
                mode_data["model"] == model
            ].sort_values("n_sensors")

            x = model_data["n_sensors"].to_numpy()
            y = model_data["mean_error_rate"].to_numpy()
            yerr = model_data["std_deviation"].to_numpy()

            ax.errorbar(
                x,
                y,
                yerr=yerr,
                marker="o",
                markersize=6,
                linewidth=1.8,
                capsize=4,
                elinewidth=1.2,
                color=model_colors[model],
                label=model,
            )

        ax.set_title(mode.replace("_", " "))
        ax.set_xlabel("Number of active sensors")
        ax.set_ylabel("Mean normalized error rate")

        ax.set_xticks(
            sorted(mode_data["n_sensors"].unique())
        )

        ax.set_ylim(-1, 6)


        ax.grid(True, linestyle="--", alpha=0.35)
        ax.legend()

    # Nasconde eventuali subplot inutilizzati.
    for ax in axes[n_modes:]:
        ax.set_visible(False)

    fig.suptitle(
        "Spatial Reasoning",
        fontsize=16,
        fontweight="bold",
    )

    fig.tight_layout(rect=[0, 0, 1, 0.95])

    png_path = output_dir / f"{FIGURE_NAME}.png"
    pdf_path = output_dir / f"{FIGURE_NAME}.pdf"

    fig.savefig(png_path, dpi=DPI, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")

    if SHOW_PLOT:
        plt.show()

    plt.close(fig)

    print(f"Grafico salvato: {png_path}")
    print(f"Grafico salvato: {pdf_path}")


# ============================================================
# TABELLE RIASSUNTIVE
# ============================================================

def export_tables(summary, output_dir):
    # 1. Tabella numerica completa in formato long.
    long_path = output_dir / "error_rate_statistics.csv"
    summary.to_csv(long_path, index=False)

    # 2. Tabella con indice gerarchico:
    #    numero sensori -> modello
    #    colonne = modalità
    #
    # Ogni cella riporta media e varianza.
    table = summary.copy()

    table["mean_and_std"] = table.apply(
    lambda row: (
            f"{row['mean_error_rate']:.4f} ± "
            f"{row['std_deviation']:.4f}"
            if pd.notna(row["std_deviation"])
            else f"{row['mean_error_rate']:.4f} ± N/A"
        ),
        axis=1,
    )

    pivot = table.pivot(
        index=["n_sensors", "model"],
        columns="mode",
        values="mean_and_std",
    )

    pivot = pivot.sort_index(level="n_sensors")

    # Ordina i modelli naturalmente all'interno di ogni gruppo
    ordered_indices = []

    for sensor_count in sorted(pivot.index.get_level_values("n_sensors").unique()):
        models_for_sensor = pivot.loc[sensor_count].index.tolist()
        models_for_sensor = sorted(models_for_sensor, key=natural_key)

        ordered_indices.extend(
            (sensor_count, model)
            for model in models_for_sensor
        )

    pivot = pivot.reindex(
        pd.MultiIndex.from_tuples(
            ordered_indices,
            names=["n_sensors", "model"],
        )
    )

    pivot.columns.name = None

    csv_path = output_dir / "error_rate_summary_table.csv"
    latex_path = output_dir / "error_rate_summary_table.tex"

    pivot.to_csv(csv_path)
    pivot.to_latex(
        latex_path,
        multirow=True,
        escape=True,
        caption=(
            "Mean and sample variance of the normalized "
            "error rate by sensor count, model, and mode."
        ),
        label="tab:spatial_reasoning",
    )

    print("\nTabella riassuntiva")
    print("Valori nelle celle: media / varianza campionaria\n")
    print(pivot.to_string())

    print(f"\nStatistiche numeriche: {long_path}")
    print(f"Tabella CSV: {csv_path}")
    print(f"Tabella LaTeX: {latex_path}")


# ============================================================
# MAIN
# ============================================================

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    df = load_runs(JSON_PATH)

    summary = compute_statistics(df)

    # Salva anche i dati di ogni run per verificare
    # i calcoli e l'aggregazione.
    df.to_csv(
        OUTPUT_DIR / "individual_run_error_rates.csv",
        index=False,
    )

    plot_spatial_reasoning(summary, OUTPUT_DIR)
    export_tables(summary, OUTPUT_DIR)


if __name__ == "__main__":
    main()