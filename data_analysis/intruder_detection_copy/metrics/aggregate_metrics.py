
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
COSINE_INPUT_FILE = BASE_DIR / "cosine_similarity.json"

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

PLOTS_DIR = BASE_DIR / "plots"
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

# ============================================================
# 1. LOAD DATA
# ============================================================

with open(METRICS_INPUT_FILE, "r", encoding="utf-8") as file:
    data = json.load(file)

with open(JACCARD_INPUT_FILE, "r", encoding="utf-8") as file:
    jaccard_data = json.load(file)

with open(COSINE_INPUT_FILE, "r", encoding="utf-8") as file:
    cosine_data = json.load(file)


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
                        "efficiency": run_data.get(
                            "efficiency",
                            run_data.get("efficiency_human")
                        ),
                        "normalized_hamming_distance": run_data.get(
                            "normalized_hamming_distance"
                        )
                    })


df = pd.DataFrame(records)


# DataFrame structure (one row per experimental run):
#
# | method | experiment_type | world   | trial   | run   | n_sensors | success_rate | efficiency | normalized_hamming_distance |
# |--------|-----------------|---------|---------|-------|-----------|--------------|------------|------------------------------|
# | HITL   | deterministic   | world_1 | trial_1 | run_1 | 1         | 1.0          | 0.78       | 0.67                         |
# | HITL   | deterministic   | world_1 | trial_1 | run_2 | 1         | 1.0          | 0.78       | 0.67                         |
# | AUTO   | uncertain       | world_2 | trial_2 | run_1 | 4         | 1.0          | 1.00       | NaN                          |
# | ...    | ...             | ...     | ...     | ...   | ...       | ...          | ...        | ...                          |
#
# Each row corresponds to one run from the original JSON.
#
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


# DataFrame structure (one row per method-pair comparison):
#
# | experiment_type | world   | trial   | sensor_sequence | n_sensors | method_pair | jaccard_similarity |
# |-----------------|---------|---------|-----------------|-----------|-------------|--------------------|
# | deterministic   | world_1 | trial_1 | camera_1        | 1         | HITL-AUTO   | 0.85               |
# | deterministic   | world_1 | trial_1 | camera_1        | 1         | HITL-ILP    | 0.72               |
# | deterministic   | world_1 | trial_1 | camera_1        | 1         | AUTO-ILP    | 0.90               |
# | ...             | ...     | ...     | ...             | ...       | ...         | ...                |
#
# Each row corresponds to one pairwise Jaccard similarity comparison.
#
# The JSON hierarchy:
# experiment_type -> world -> trial -> sensor_sequence -> method_pair
# is flattened into a tabular DataFrame.


if jaccard_df.empty:
    raise ValueError("No Jaccard similarity data found.")

print("\nTOTAL JACCARD COMPARISONS:", len(jaccard_df))

print("\nJACCARD COMPARISONS PER PAIR:")
print(jaccard_df.groupby("method_pair").size())


# ============================================================
# 4. EXTRACT COSINE SIMILARITY
# ============================================================

cosine_records = []

for experiment_type, worlds in cosine_data.items():

    for world, trials in worlds.items():

        for trial, sequences in trials.items():

            for sequence, similarity_data in sequences.items():

                n_sensors = len([
                    sensor for sensor in sequence.split(",")
                    if sensor.strip()
                ])

                # Binary and weighted cosine similarities
                for similarity_type in ["binary", "weighted"]:

                    pairwise_results = similarity_data.get(
                        similarity_type, {}
                    )

                    for method_pair, similarity in pairwise_results.items():

                        cosine_records.append({
                            "experiment_type": experiment_type,
                            "world": world,
                            "trial": trial,
                            "sensor_sequence": sequence,
                            "n_sensors": n_sensors,
                            "method_pair": method_pair,
                            "similarity_type": similarity_type,
                            "cosine_similarity": similarity
                        })


cosine_df = pd.DataFrame(cosine_records)


# DataFrame structure (one row per method-pair cosine similarity comparison):
#
# | experiment_type | world   | trial   | sensor_sequence | n_sensors | method_pair | similarity_type | cosine_similarity |
# |-----------------|---------|---------|-----------------|-----------|-------------|-----------------|-------------------|
# | deterministic   | world_1 | trial_1 | camera_1        | 1         | HITL-AUTO   | binary          | 0.92              |
# | deterministic   | world_1 | trial_1 | camera_1        | 1         | HITL-ILP    | binary          | 0.85              |
# | deterministic   | world_1 | trial_1 | camera_1        | 1         | HITL-AUTO   | weighted        | 0.87              |
# | ...             | ...     | ...     | ...             | ...       | ...         | ...             | ...               |
#
# Each row corresponds to one pairwise cosine similarity comparison.
#
# The similarity_type column distinguishes between binary and weighted
# inspection vectors.
#
# The JSON hierarchy:
# experiment_type -> world -> trial -> sensor_sequence
# -> similarity_type -> method_pair
# is flattened into a tabular DataFrame.


if cosine_df.empty:
    raise ValueError("No Cosine similarity data found.")

print("\nTOTAL COSINE COMPARISONS:", len(cosine_df))

print("\nCOSINE COMPARISONS PER PAIR:")
print(cosine_df.groupby(
    ["similarity_type", "method_pair"]
).size())


# ============================================================
# 5. AGGREGATE RUN METRICS
# ============================================================

# Define the variables used for aggregation.
# Runs sharing the same method, experiment type, and number
# of sensors are assigned to the same group.

aggregate_columns = [
    "method",
    "experiment_type",
    "n_sensors"
]

# Group the dataframe according to the selected variables.

grouped = df.groupby(
    aggregate_columns,
    dropna=False
)

# Select the metrics to be aggregated.

grouped_metrics = grouped[METRICS]

# Calculate the mean, standard deviation, and number of observations
# for each metric within each group.

aggregated = grouped_metrics.agg([
    "mean",
    "std",
    "count"
])

# Reset the group indices and convert them into regular dataframe columns.

aggregated = aggregated.reset_index()

# Flatten MultiIndex columns.
# For example: efficiency + mean -> efficiency_mean.

aggregated.columns = [
    f"{metric}_{statistic}" if statistic else metric
    for metric, statistic in aggregated.columns
]

# Replace missing values with zero.

aggregated = aggregated.fillna(0)


# ============================================================
# 6. AGGREGATE JACCARD SIMILARITY
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
    .rename(columns={
        "mean": "jaccard_similarity_mean",
        "std": "jaccard_similarity_std",
        "count": "jaccard_similarity_count"
    })
    .fillna(0)
)

print("\n" + "=" * 80)
print("JACCARD SIMILARITY - FINAL STATISTICS")
print("=" * 80)

print(jaccard_aggregated.to_string(index=False))


# ============================================================
# 7. AGGREGATE COSINE SIMILARITY
# ============================================================

cosine_aggregated = (
    cosine_df
    .groupby([
        "method_pair",
        "experiment_type",
        "n_sensors",
        "similarity_type"
    ])["cosine_similarity"]
    .agg(["mean", "std", "count"])
    .reset_index()
    .rename(columns={
        "mean": "cosine_similarity_mean",
        "std": "cosine_similarity_std",
        "count": "cosine_similarity_count"
    })
    .fillna(0)
)

print("\n" + "=" * 80)
print("COSINE SIMILARITY - FINAL STATISTICS")
print("=" * 80)

print(cosine_aggregated.to_string(index=False))


# Separate binary and weighted cosine similarity data.

binary_cosine_aggregated = cosine_aggregated[
    cosine_aggregated["similarity_type"] == "binary"
].copy()

weighted_cosine_aggregated = cosine_aggregated[
    cosine_aggregated["similarity_type"] == "weighted"
].copy()



# ============================================================
# 8. HELPER FUNCTION FOR PLOTTING METRICS
# ============================================================

def plot_metric(
    aggregated_data,
    metric,
    ylabel,
    filename,
    ylim=None,
    show_error_bars=False
):
    """
    Plots an aggregated metric for each experiment type.
    Each figure contains one plot for each experiment type.
    Figures are displayed and saved as PDF files.
    """

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

        plt.xlabel(r"$N_s$", fontsize=14)
        plt.ylabel(ylabel, fontsize=12)

        # plt.title(
        #     experiment_type,
        #     fontsize=14
        # )

        sensor_counts = sorted(
            experiment_data["n_sensors"].unique()
        )

        plt.xticks(sensor_counts)

        if ylim is not None:
            plt.ylim(*ylim)

        plt.grid(
            True,
            linestyle="--",
            alpha=0.5
        )

        # Place the legend outside the plot, horizontally.
        plt.legend(
            title="Method",
            loc="upper center",
            bbox_to_anchor=(0.5, -0.12),
            ncol=len(METHODS_ORDER),
            frameon=False
        )

        plt.tight_layout(rect=[0, 0.08, 1, 1])

        # Save the figure as a PDF in the script directory.

        output_file = PLOTS_DIR / (
            f"{filename}_{experiment_type}.pdf"
        )

        plt.savefig(
            output_file,
            format="pdf",
            bbox_inches="tight"
        )

        plt.show()
        plt.close()

        print(f"Plot saved to: {output_file}")


# ============================================================
# 9. HELPER FUNCTION FOR SIMILARITY PLOTS
# ============================================================


def plot_similarity(
    aggregated_data,
    similarity_column,
    ylabel,
    title,
    filename,
    show_error_bars=False
):
    """
    Plots the mean similarity for each method pair.
    Each subplot represents a method pair, with separate curves
    for each experiment type.

    The subplots are arranged in 3 rows:
    4 subplots in the first row,
    4 subplots in the second row,
    2 centered subplots in the third row.

    A shared legend is placed below all subplots.
    The figure is displayed and saved as a PDF file.
    """

    method_pairs = sorted(
        aggregated_data["method_pair"].unique()
    )

    experiment_types = sorted(
        aggregated_data["experiment_type"].unique()
    )

    n_pairs = len(method_pairs)

    if n_pairs == 0:
        print(f"No data available for {title}.")
        return

    # --------------------------------------------------------
    # Define subplot layout
    # --------------------------------------------------------

    n_cols = 4
    n_rows = 3

    fig, axes = plt.subplots(
        n_rows,
        n_cols,
        figsize=(20, 12),
        squeeze=False
    )

    axes = axes.flatten()

    # Define the subplot positions.
    # For 10 method pairs, the last two plots are centered.

    if n_pairs == 10:

        plot_positions = [
            0, 1, 2, 3,
            4, 5, 6, 7,
            9, 10
        ]

    else:

        plot_positions = list(range(n_pairs))

    # --------------------------------------------------------
    # Plot each method pair
    # --------------------------------------------------------

    for i, method_pair in enumerate(method_pairs):

        ax = axes[plot_positions[i]]

        pair_data = aggregated_data[
            aggregated_data["method_pair"] == method_pair
        ]

        for experiment_type in experiment_types:

            experiment_data = pair_data[
                pair_data["experiment_type"] == experiment_type
            ].sort_values("n_sensors")

            if experiment_data.empty:
                continue

            x = experiment_data["n_sensors"].to_numpy()

            y = experiment_data[
                f"{similarity_column}_mean"
            ].to_numpy()

            errors = experiment_data[
                f"{similarity_column}_std"
            ].to_numpy()

            if show_error_bars:

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
            ylabel,
            fontsize=12
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

    # --------------------------------------------------------
    # Hide unused subplots
    # --------------------------------------------------------

    for i in range(len(axes)):

        if i not in plot_positions:
            axes[i].set_visible(False)

    # --------------------------------------------------------
    # Shared legend below all subplots
    # --------------------------------------------------------

    handles, labels = axes[plot_positions[0]].get_legend_handles_labels()

    fig.legend(
        handles,
        labels,
        title="Experiment",
        loc="lower center",
        bbox_to_anchor=(0.5, 0.01),
        ncol=len(experiment_types),
        frameon=False
    )

    # --------------------------------------------------------
    # Figure layout
    # --------------------------------------------------------

    fig.suptitle(
        title,
        fontsize=16,
        y=0.98
    )

    plt.tight_layout(
        rect=[0, 0.10, 1, 0.95]
    )

    # --------------------------------------------------------
    # Save PDF
    # --------------------------------------------------------

    output_file = PLOTS_DIR / f"{filename}.pdf"

    plt.savefig(
        output_file,
        format="pdf",
        bbox_inches="tight"
    )

    plt.show()
    plt.close()

    print(f"Plot saved to: {output_file}")

# ============================================================
# 10. NORMALIZED HAMMING DISTANCE
# ============================================================

def plot_hamming_distance(
    df,
    show_error_bars=False
):
    """
    Plots the mean normalized Hamming distance for HITL.
    Each curve represents a different experiment type.
    The figure is displayed and saved as a PDF file.
    """

    hir_data = df[
        df["method"] == "HITL"
    ].copy()

    hir_data = hir_data.dropna(
        subset=["normalized_hamming_distance"]
    )

    if hir_data.empty:
        print("No HITL normalized Hamming distance data found.")
        return

    hir_statistics = (
        hir_data
        .groupby([
            "experiment_type",
            "n_sensors"
        ])["normalized_hamming_distance"]
        .agg(["mean", "std", "count"])
        .reset_index()
        .fillna(0)
    )

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

    plt.xlabel(r"$N_s$", fontsize=14)
    plt.ylabel(r"$\overline{HIR}$", fontsize=14)

    plt.xticks(
        sorted(hir_statistics["n_sensors"].unique())
    )

    plt.ylim(0, 1.05)

    plt.grid(
        True,
        linestyle="--",
        alpha=0.5
    )

    plt.legend(
        title="Experiment",
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=len(experiment_types),
        frameon=False
    )

    plt.tight_layout(rect=[0, 0.08, 1, 1])

    # Save the figure as a PDF in the script directory.

    output_file = PLOTS_DIR / "normalized_hamming_distance.pdf"

    plt.savefig(
        output_file,
        format="pdf",
        bbox_inches="tight"
    )

    plt.show()
    plt.close()

    print(f"Plot saved to: {output_file}")


# ============================================================
# 11. AGGREGATED METRICS TABLES - MEAN ± STD_DEV
# ============================================================

def print_metrics_tables(aggregated):

    metrics_to_display = {
        "success_rate": "Success Rate",
        "efficiency": "Inspection Efficiency"
    }

    for experiment_type in sorted(
        aggregated["experiment_type"].unique()
    ):

        experiment_data = aggregated[
            aggregated["experiment_type"] == experiment_type
        ]

        for metric, metric_label in metrics_to_display.items():

            print("\n" + "=" * 100)
            print(f"{metric_label} - {experiment_type}")
            print("=" * 100)

            table = {}

            for n_sensors in sorted(
                experiment_data["n_sensors"].unique()
            ):

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

            table_df = table_df.reindex(
                columns=METHODS_ORDER
            )

            print(table_df.to_string())


# ============================================================
# 12. MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # Success Rate
    # --------------------------------------------------------

    plot_metric(
        aggregated_data=aggregated,
        metric="success_rate",
        ylabel=r"$\overline{SR}$",
        filename="success_rate",
        ylim=(0, 1.05),
        show_error_bars=SHOW_ERROR_BARS
    )

    # --------------------------------------------------------
    # Inspection Efficiency
    # --------------------------------------------------------

    plot_metric(
        aggregated_data=aggregated,
        metric="efficiency",
        ylabel=r"$\overline{\eta}$",
        filename="inspection_efficiency",
        ylim=(0, 1.05),
        show_error_bars=SHOW_ERROR_BARS
    )

    # --------------------------------------------------------
    # Jaccard Similarity
    # --------------------------------------------------------

    plot_similarity(
        aggregated_data=jaccard_aggregated,
        similarity_column="jaccard_similarity",
        ylabel=r"$\overline{JS}$",
        title="Mean Jaccard Similarity",
        filename="jaccard_similarity",
        show_error_bars=SHOW_ERROR_BARS
    )

    # --------------------------------------------------------
    # Binary Cosine Similarity
    # --------------------------------------------------------

    plot_similarity(
        aggregated_data=binary_cosine_aggregated,
        similarity_column="cosine_similarity",
        ylabel=r"$\overline{CS}_{binary}$",
        title="Mean Binary Cosine Similarity",
        filename="binary_cosine_similarity",
        show_error_bars=SHOW_ERROR_BARS
    )

    # --------------------------------------------------------
    # Weighted Cosine Similarity
    # --------------------------------------------------------

    plot_similarity(
        aggregated_data=weighted_cosine_aggregated,
        similarity_column="cosine_similarity",
        ylabel=r"$\overline{CS}_{weighted}$",
        title="Mean Weighted Cosine Similarity",
        filename="weighted_cosine_similarity",
        show_error_bars=SHOW_ERROR_BARS
    )

    # --------------------------------------------------------
    # Normalized Hamming Distance
    # --------------------------------------------------------

    plot_hamming_distance(
        df=df,
        show_error_bars=SHOW_ERROR_BARS
    )

    # --------------------------------------------------------
    # Aggregated Metrics Tables
    # --------------------------------------------------------

    print_metrics_tables(aggregated)

    # --------------------------------------------------------
    # Processing Summary
    # --------------------------------------------------------

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


# ============================================================
# SCRIPT EXECUTION
# ============================================================

if __name__ == "__main__":
    main()
