
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

METRICS_INPUT_FILE = BASE_DIR / "run_metrics.json"
JACCARD_INPUT_FILE = BASE_DIR / "jaccard_similarity.json"

SHOW_ERROR_BARS = False

METHODS_ORDER = [
    "HITL",
    "AUTO",
    "ILP-B30",
    "ILP-B60",
    "ILP-B120"
]

METRICS = [
    "efficiency",
    "success_rate",
    "normalized_hamming_distance"
]


# ============================================================
# 1. LOAD DATA
# ============================================================

with open(METRICS_INPUT_FILE, "r", encoding="utf-8") as file:
    data = json.load(file)

with open(JACCARD_INPUT_FILE, "r", encoding="utf-8") as file:
    jaccard_data = json.load(file)


# ============================================================
# 2. EXTRACT RUN METRICS
# ============================================================

records = []

for method, method_data in data.items():

    for experiment_type, experiment_type_data in method_data.items():

        for world, world_data in experiment_type_data.items():

            for trial, trial_data in world_data.items():

                for run, run_data in trial_data.items():

                    if not isinstance(run_data, dict):
                        continue

                    if "sensor_sequence" not in run_data:
                        continue

                    sequence = run_data["sensor_sequence"]

                    records.append({
                        "method": method,
                        "experiment_type": experiment_type,
                        "world": world,
                        "trial": trial,
                        "run": run,
                        "n_sensors": len(sequence),
                        "success_rate": run_data.get("success_rate"),
                        "efficiency": run_data.get("efficiency", run_data.get("efficiency_human")),
                        "normalized_hamming_distance": run_data.get("normalized_hamming_distance")
                    })


df = pd.DataFrame(records)

# DataFrame structure (one row per experimental run):
#
#
#      | method   | experiment_type | world   | trial   | run    | n_sensors | success_rate | efficiency | normalized_hamming_distance |
#      |----------|----------------|---------|---------|--------|-----------|--------------|------------|------------------------------|
#      | HITL     | deterministic  | world_1 | trial_1 | run_1  | 1         | 1.0          | 0.78       | 0.67                         |
#      | HITL     | deterministic  | world_1 | trial_1 | run_2  | 1         | 1.0          | 0.78       | 0.67                         |
#      | HITL     | deterministic  | world_1 | trial_1 | run_3  | 1         | 1.0          | 0.62       | 0.62                         |
#      | HITL     | deterministic  | world_1 | trial_1 | run_4  | 1         | 1.0          | 0.88       | 0.33                         |
#      | HITL     | deterministic  | world_1 | trial_1 | run_5  | 1         | 1.0          | 1.00       | 0.00                         |
#      | ...      | ...            | ...     | ...     | ...    | ...       | ...          | ...        | ...                          |
#      | ILP-B120 | uncertain      | world_2 | trial_2 | run_16 | 4         | 1.0          | 1.00       | NaN                          |
#      | ILP-B120 | uncertain      | world_2 | trial_2 | run_17 | 4         | 0.0          | 0.00       | NaN                          |
#      | ILP-B120 | uncertain      | world_2 | trial_2 | run_18 | 4         | 1.0          | 1.00       | NaN                          |
#      | ILP-B120 | uncertain      | world_2 | trial_2 | run_19 | 4         | 1.0          | 0.87       | NaN                          |
#      | ILP-B120 | uncertain      | world_2 | trial_2 | run_20 | 4         | 1.0          | 1.00       | NaN                          |
#
#
# Each row corresponds to one run from the original JSON.
# The JSON hierarchy:
# method -> experiment_type -> world -> trial -> run
# is flattened into a tabular DataFrame.

if df.empty:
    raise ValueError("No run metrics found. Check the input JSON hierarchy.")

print("\nTOTAL RUNS:", len(df))

print("\nRUNS PER METHOD:")
print(df.groupby("method").size())


# ============================================================
# 3. EXTRACT JACCARD SIMILARITY
# ============================================================

jaccard_records = []

for experiment_type, worlds in jaccard_data.items():

    for world, trials in worlds.items():

        for trial, sequences in trials.items():

            for sequence, pairwise_results in sequences.items():

                n_sensors = len([
                    sensor for sensor in sequence.split(",")
                    if sensor.strip()
                ])

                for method_pair, similarity in pairwise_results.items():

                    jaccard_records.append({
                        "experiment_type": experiment_type,
                        "world": world,
                        "trial": trial,
                        "sensor_sequence": sequence,
                        "n_sensors": n_sensors,
                        "method_pair": method_pair,
                        "jaccard_similarity": similarity
                    })


jaccard_df = pd.DataFrame(jaccard_records)

if jaccard_df.empty:
    raise ValueError("No Jaccard similarity data found.")

print("\nTOTAL JACCARD COMPARISONS:", len(jaccard_df))

print("\nJACCARD COMPARISONS PER PAIR:")
print(jaccard_df.groupby("method_pair").size())


# ============================================================
# 3. AGGREGATE METRICS ACROSS INDEPENDENT RUNS
# ============================================================

aggregate_columns = [
    "method",
    "experiment_type",
    "n_sensors"
]

aggregated = (
    df
    .groupby(aggregate_columns, dropna=False)[METRICS]
    .agg(["mean", "std", "count"])
    .reset_index()
)

# Flatten MultiIndex columns
aggregated.columns = [
    f"{metric}_{statistic}" if statistic else metric
    for metric, statistic in aggregated.columns
]

aggregated = aggregated.fillna(0)


# ============================================================
# 4. AGGREGATE JACCARD SIMILARITY
# ============================================================

jaccard_aggregated = (
    jaccard_df
    .groupby([
        "method_pair",
        "experiment_type",
        "n_sensors"
    ])["jaccard_similarity"]
    .agg(["mean", "std", "count"])
    .reset_index()
    .fillna(0)
)


print("\n" + "=" * 80)
print("JACCARD SIMILARITY - FINAL STATISTICS")
print("=" * 80)

print(jaccard_aggregated.to_string(index=False))


# ============================================================
# 5. HELPER FUNCTION FOR PLOTTING
# ============================================================

def plot_metric(
    aggregated_data,
    metric,
    ylabel,
    # title,
    ylim=None,
    show_error_bars=False
):

    experiment_types = sorted(
        aggregated_data["experiment_type"].unique()
    )

    for experiment_type in experiment_types:

        plt.figure(figsize=(10, 6))

        experiment_data = aggregated_data[
            aggregated_data["experiment_type"] == experiment_type
        ]

        for method in METHODS_ORDER:

            method_data = experiment_data[
                experiment_data["method"] == method
            ].sort_values("n_sensors")

            if method_data.empty:
                continue

            x = method_data["n_sensors"].to_numpy()

            y = method_data[
                f"{metric}_mean"
            ].to_numpy()

            errors = method_data[
                f"{metric}_std"
            ].to_numpy()

            if show_error_bars:

                plt.errorbar(
                    x,
                    y,
                    yerr=errors,
                    marker="o",
                    linewidth=2,
                    markersize=7,
                    capsize=5,
                    capthick=1.5,
                    elinewidth=1.5,
                    label=method
                )

            else:

                plt.plot(
                    x,
                    y,
                    marker="o",
                    linewidth=2,
                    markersize=7,
                    label=method
                )

        plt.xlabel(
            r"$N_s$",
            fontsize=14
        )

        plt.ylabel(
            ylabel,
            fontsize=12
        )

        plt.title(
            f"{experiment_type}",
            fontsize=14
        )

        sensor_counts = experiment_data[
            "n_sensors"
        ].unique()

        plt.xticks(sorted(sensor_counts))

        if ylim is not None:
            plt.ylim(*ylim)

        plt.grid(
            True,
            linestyle="--",
            alpha=0.5
        )

        plt.legend(title="Method")

        plt.tight_layout()

        # Display directly on screen.
        plt.show()

        plt.close()


# ============================================================
# 5. JACCARD SIMILARITY PLOT
# ============================================================

method_pairs = sorted(
    jaccard_aggregated["method_pair"].unique()
)

experiment_types = sorted(
    jaccard_aggregated["experiment_type"].unique()
)

n_pairs = len(method_pairs)

n_cols = 5
n_rows = (n_pairs + n_cols - 1) // n_cols

fig, axes = plt.subplots(
    n_rows,
    n_cols,
    figsize=(5 * n_cols, 4 * n_rows),
    squeeze=False
)

axes = axes.flatten()

for i, method_pair in enumerate(method_pairs):

    ax = axes[i]

    pair_data = jaccard_aggregated[
        jaccard_aggregated["method_pair"] == method_pair
    ]

    for experiment_type in experiment_types:

        experiment_data = pair_data[
            pair_data["experiment_type"] == experiment_type
        ].sort_values("n_sensors")

        if experiment_data.empty:
            continue

        x = experiment_data["n_sensors"].to_numpy()

        y = experiment_data["mean"].to_numpy()

        errors = experiment_data["std"].to_numpy()

        if SHOW_ERROR_BARS:

            ax.errorbar(
                x,
                y,
                yerr=errors,
                marker="o",
                linewidth=2,
                markersize=7,
                capsize=5,
                capthick=1.5,
                elinewidth=1.5,
                label=experiment_type
            )

        else:

            ax.plot(
                x,
                y,
                marker="o",
                linewidth=2,
                markersize=7,
                label=experiment_type
            )

    ax.set_xlabel(
        r"$N_s$",
        fontsize=10
    )

    ax.set_ylabel(
        r"$\overline{JS}$",
        fontsize=14
    )

    ax.set_title(
        method_pair,
        fontsize=12
    )

    sensor_counts = sorted(
        pair_data["n_sensors"].unique()
    )

    ax.set_xticks(sensor_counts)

    ax.set_ylim(0, 1.05)

    ax.grid(
        True,
        linestyle="--",
        alpha=0.5
    )

    ax.legend(title="Experiment")


# Hide unused subplots
for i in range(n_pairs, len(axes)):
    axes[i].set_visible(False)


fig.suptitle(
    "Mean Jaccard Similarity",
    fontsize=16,
    y=1.02
)

plt.tight_layout()

plt.show()

plt.close()


# ============================================================
# 6. SUCCESS RATE
# ============================================================

plot_metric(
    aggregated_data=aggregated,
    metric="success_rate",
    ylabel=r"$\overline{SR}$",
    # title=None,
    ylim=(0, 1.05),
    show_error_bars=SHOW_ERROR_BARS
)


# ============================================================
# 7. INSPECTION EFFICIENCY
# ============================================================

plot_metric(
    aggregated_data=aggregated,
    metric="efficiency",
    ylabel=r"$\overline{\eta}$",
    # title=None,
    ylim=(0, 1.05),
    show_error_bars=SHOW_ERROR_BARS
)


# ============================================================
# 8. NORMALIZED HAMMING DISTANCE (HITL)
# ============================================================

hir_data = df[
    df["method"] == "HITL"
].copy()

hir_data = hir_data.dropna(
    subset=["normalized_hamming_distance"]
)


if not hir_data.empty:

    hir_group_columns = [
        "experiment_type",
        "n_sensors"
    ]

    hir_statistics = (
        hir_data
        .groupby(hir_group_columns)[
            "normalized_hamming_distance"
        ]
        .agg(["mean", "std", "count"])
        .reset_index()
        .fillna(0)
    )

    print("\n" + "=" * 80)
    print("NORMALIZED HAMMING DISTANCE - FINAL STATISTICS")
    print("=" * 80)

    print(hir_statistics.to_string(index=False))

    experiment_types = sorted(
        hir_statistics["experiment_type"].unique()
    )

    plt.figure(figsize=(8, 6))

    for experiment_type in experiment_types:

        points = hir_statistics[
            hir_statistics["experiment_type"] == experiment_type
        ].sort_values("n_sensors")

        x = points["n_sensors"].to_numpy()
        y = points["mean"].to_numpy()
        errors = points["std"].to_numpy()

        if SHOW_ERROR_BARS:

            plt.errorbar(
                x,
                y,
                yerr=errors,
                marker="o",
                linewidth=2,
                markersize=7,
                capsize=5,
                capthick=1.5,
                elinewidth=1.5,
                label=experiment_type
            )

        else:

            plt.plot(
                x,
                y,
                marker="o",
                linewidth=2,
                markersize=7,
                label=experiment_type
            )

    plt.xlabel(
        r"$N_s$",
        fontsize=14
    )

    plt.ylabel(
        r"$\overline{HIR}$",
        fontsize=14
    )

    plt.xticks(
        sorted(hir_statistics["n_sensors"].unique())
    )

    plt.ylim(0, 1.05)

    plt.grid(
        True,
        linestyle="--",
        alpha=0.5
    )

    plt.legend(title="Experiment")

    plt.tight_layout()

    plt.show()

    plt.close()

else:

    print("\nNo HITL normalized Hamming distance data found.")



# ============================================================
# 8. AGGREGATED METRICS TABLES - MEAN ± STD_DEV
# ============================================================

METHODS_ORDER = [
    "HITL",
    "AUTO",
    "ILP-B30",
    "ILP-B60",
    "ILP-B120"
]

METRICS_TO_DISPLAY = {
    "success_rate": "Success Rate",
    "efficiency": "Inspection Efficiency",
}

for experiment_type in sorted(aggregated["experiment_type"].unique()):

    experiment_data = aggregated[
        aggregated["experiment_type"] == experiment_type
    ]

    for metric, metric_label in METRICS_TO_DISPLAY.items():

        print("\n" + "=" * 100)
        print(f"{metric_label} - {experiment_type}")
        print("=" * 100)

        table = {}

        for n_sensors in sorted(experiment_data["n_sensors"].unique()):

            table[n_sensors] = {}

            for method in METHODS_ORDER:

                row = experiment_data[
                    (experiment_data["n_sensors"] == n_sensors) &
                    (experiment_data["method"] == method)
                ]

                if row.empty:
                    table[n_sensors][method] = "-"
                    continue

                mean = row.iloc[0][f"{metric}_mean"]
                std = row.iloc[0][f"{metric}_std"]

                table[n_sensors][method] = (
                    f"{mean:.3f} ± {std:.3f}"
                )

        table_df = pd.DataFrame.from_dict(
            table,
            orient="index"
        )

        table_df.index.name = "N_s"
        table_df = table_df.reindex(columns=METHODS_ORDER)

        print(table_df.to_string())

# ============================================================
# 9. FINAL STATISTICS
# ============================================================

print("\n" + "=" * 80)
print("PROCESSING COMPLETED")
print("=" * 80)

print(f"Total runs processed: {len(df)}")

print("\nMethods found:")

for method in METHODS_ORDER:

    count = len(
        df[df["method"] == method]
    )

    if count > 0:
        print(f"{method}: {count} runs")

