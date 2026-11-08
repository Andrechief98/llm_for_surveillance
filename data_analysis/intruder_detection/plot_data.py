
import json
import re
from pathlib import Path

import numpy as np
import pandas as pd
import streamlit as st
import plotly.graph_objects as go

from plotly.subplots import make_subplots


# ============================================================
# CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

EXPERIMENTS_PATHS = {
    methodology: BASE_DIR / f"experiments_{methodology}.json"
    for methodology in ["HITL", "AUTO", "ILP", "HUMAN"]
}

EVALUATIONS_PATH = BASE_DIR / "evaluations.json"

EXPERIMENT_TYPES = ["deterministic", "uncertain"]

INSPECTION_MATRICES_PATH = BASE_DIR / "inspection_matrices.json"


METHODOLOGIES = [
    "HITL",
    "AUTO",
    "HUMAN",
    "ILP-30",
    "ILP-60",
    "ILP-120"
]

LLM_METHODOLOGIES = ["HITL", "AUTO"]

VARIANCE_DDOF = 1

WORLD_ORDER = ["world_1", "world_2"]


# ============================================================
# HELPERS
# ============================================================

def create_success_latex_table(
    summary,
    experiment_type,
    world,
    selected_methods=None,
    caption=None,
    label=None
):
    """
    Generate a LaTeX table for success rate statistics.

    Rows: number of sensors.
    Columns: methodologies.
    Values: mean success rate +/- standard deviation.
    """

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if selected_methods is not None:
        filtered = filtered[
            filtered["methodology"].isin(selected_methods)
        ]

    if filtered.empty:
        return None

    # Preserve the predefined methodology order.
    methods = [
        method for method in METHODOLOGIES
        if method in filtered["methodology"].unique()
    ]

    sensors = sorted(filtered["n_sensors"].unique())

    # Create LaTeX table header.
    columns = "c " + " ".join(["c"] * len(methods))

    latex = [
        r"\begin{table*}[h!]",
        r"\centering",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{3pt}",
        ""
    ]

    if caption:
        latex.append(
            rf"\caption{{{caption}}}"
        )

    if label:
        latex.append(
            rf"\label{{{label}}}"
        )

    latex.extend([
        rf"\begin{{tabular}}{{{columns}}}",
        r"\toprule"
    ])

    # Header.
    header = [
        r"\textbf{$\boldsymbol{N_s}$}"
    ]

    header.extend([
        rf"\textbf{{{method}}}"
        for method in methods
    ])

    latex.append(" & ".join(header) + r" \\")
    latex.append(r"\midrule")

    # Table rows.
    pivot = filtered.pivot(
        index="n_sensors",
        columns="methodology",
        values=["mean_success_rate", "std_dev"]
    )

    for sensor in sensors:

        row = [str(sensor)]

        for method in methods:

            try:
                mean = pivot.loc[
                    sensor,
                    ("mean_success_rate", method)
                ]

                std = pivot.loc[
                    sensor,
                    ("std_dev", method)
                ]

                if pd.isna(mean):
                    value = "--"
                elif pd.isna(std):
                    value = f"{mean * 100:.2f}\\% $\\pm$ N/A"
                else:
                    value = (
                        f"{mean * 100:.2f}\\% "
                        f"$\\pm$ {std * 100:.2f}\\%"
                    )

            except KeyError:
                value = "--"

            row.append(value)

        latex.append(" & ".join(row) + r" \\")

    latex.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table*}"
    ])

    return "\n".join(latex)

def create_efficiency_latex_table(
    summary,
    experiment_type,
    world,
    selected_methods=None,
    caption=None,
    label=None
):
    """
    Generate a LaTeX table for inspection efficiency statistics.

    Expected input structure:
        One row per experiment_type, world, methodology
        and number of sensors.

    Columns:
        methodology
        n_sensors
        mean_efficiency
        std_dev
        total_runs

    Rows represent sensor counts.
    Columns represent methodologies.
    Values are mean efficiency +/- standard deviation.
    """

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if selected_methods is not None:
        filtered = filtered[
            filtered["methodology"].isin(selected_methods)
        ]

    if filtered.empty:
        return None

    methods = [
        method for method in METHODOLOGIES
        if method in filtered["methodology"].unique()
    ]

    sensors = sorted(filtered["n_sensors"].unique())

    columns = "c " + " ".join(["c"] * len(methods))

    latex = [
        r"\begin{table*}[h!]",
        r"\centering",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{4pt}",
        ""
    ]

    if caption:
        latex.append(rf"\caption{{{caption}}}")

    if label:
        latex.append(rf"\label{{{label}}}")

    latex.extend([
        rf"\begin{{tabular}}{{{columns}}}",
        r"\toprule"
    ])

    header = [r"\textbf{$\boldsymbol{N_s}$}"]

    header.extend([
        rf"\textbf{{{method}}}"
        for method in methods
    ])

    latex.append(" & ".join(header) + r" \\")
    latex.append(r"\midrule")

    pivot = filtered.pivot(
        index="n_sensors",
        columns="methodology",
        values=["mean_efficiency", "std_dev"]
    )

    for sensor in sensors:

        row = [str(sensor)]

        for method in methods:

            try:
                mean = pivot.loc[
                    sensor,
                    ("mean_efficiency", method)
                ]

                std = pivot.loc[
                    sensor,
                    ("std_dev", method)
                ]

                if pd.isna(mean):
                    value = "--"

                elif pd.isna(std):
                    value = (
                        f"{mean:.3f} $\\pm$ N/A"
                    )

                else:
                    value = (
                        f"{mean:.3f} "
                        f"$\\pm$ {std:.3f}"
                    )

            except KeyError:
                value = "--"

            row.append(value)

        latex.append(" & ".join(row) + r" \\")

    latex.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table*}"
    ])

    return "\n".join(latex)

def render_success_plot(
    summary,
    experiment_type,
    world,
    key_suffix,
    selected_methods=None,
    selected_sensors=None
):
    """Render and export a success rate comparison plot."""

    filtered_summary = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if selected_methods is not None:
        filtered_summary = filtered_summary[
            filtered_summary["methodology"].isin(selected_methods)
        ]

    if selected_sensors is not None:
        filtered_summary = filtered_summary[
            filtered_summary["n_sensors"].isin(selected_sensors)
        ]

    fig = create_success_figure(
        filtered_summary,
        experiment_type,
        world
    )

    if fig is not None:

        st.plotly_chart(
            fig,
            use_container_width=True,
            config={
                "displaylogo": False,
                "responsive": True
            },
            key=f"success_plot_{key_suffix}"
        )

        try:
            jpeg_image = fig.to_image(
                format="jpeg",
                width=1600,
                height=500,
                scale=2
            )

            st.download_button(
                label="Download plot (JPEG)",
                data=jpeg_image,
                file_name=f"success_rate_{key_suffix}.jpg",
                mime="image/jpeg",
                key=f"download_jpeg_{key_suffix}"
            )

        except Exception as exc:
            st.warning(
                f"Unable to generate JPEG image: {exc}"
            )

    else:
        st.info(
            f"No data available for {experiment_type}."
        )

def natural_key(value):
    """Sort strings while treating numeric portions as integers."""

    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", str(value))
    ]


@st.cache_data
def load_json(filepath, file_mtime):
    """Load a JSON file and cache its contents."""

    with open(filepath, "r", encoding="utf-8") as file:
        return json.load(file)

def load_all_experiments(experiment_paths):
    """
    Load and merge available methodology-specific experiment files.

    Missing files are skipped.
    """

    combined_data = {
        experiment_type: {}
        for experiment_type in EXPERIMENT_TYPES
    }

    loaded_methodologies = []

    for methodology, filepath in experiment_paths.items():

        # Skip missing files.
        if not filepath.exists():
            print(f"Warning: {filepath.name} not found. Skipping.")
            continue

        try:
            raw_data = load_json(
                str(filepath),
                filepath.stat().st_mtime
            )

        except (json.JSONDecodeError, OSError) as exc:
            print(f"Warning: Unable to load {filepath.name}: {exc}")
            continue

        if not isinstance(raw_data, dict):
            continue

        loaded = False

        for experiment_type in EXPERIMENT_TYPES:

            experiment_data = raw_data.get(
                experiment_type,
                {}
            )

            if not isinstance(experiment_data, dict):
                continue

            methodology_data = experiment_data.get(
                methodology
            )

            if methodology_data is None:
                methodology_data = experiment_data

            if isinstance(methodology_data, dict) and methodology_data:

                combined_data[experiment_type][methodology] = (
                    methodology_data
                )

                loaded = True

        if loaded:
            loaded_methodologies.append(methodology)

    if not loaded_methodologies:
        raise ValueError(
            "No valid experiment files were found."
        )

    print(
        f"Loaded methodologies: {loaded_methodologies}"
    )

    return combined_data



def render_success_tables(
    summary,
    trial_stats,
    experiment_type,
    world,
    selected_methods,
    selected_sensors,
    key_suffix
):
    """Render summary statistics and individual trial data."""

    filtered_summary = summary[
        (summary["experiment_type"] == experiment_type)
        & summary["methodology"].isin(selected_methods)
        & summary["n_sensors"].isin(selected_sensors)
    ].copy()

    filtered_trials = trial_stats[
        (trial_stats["experiment_type"] == experiment_type)
        & trial_stats["methodology"].isin(selected_methods)
        & trial_stats["n_sensors"].isin(selected_sensors)
    ].copy()

    if filtered_summary.empty:
        st.info(f"No statistics available for {experiment_type}.")
        return

    # Summary statistics.
    display = filtered_summary[
        [
            "methodology",
            "n_sensors",
            "mean_success_rate",
            "std_dev",
            "n_trials",
            "successful_runs",
            "total_runs"
        ]
    ].copy()

    display.columns = [
        "Methodology",
        "Sensors",
        "Mean success (%)",
        "Std dev (%)",
        "Trials",
        "Successful",
        "Total"
    ]

    st.dataframe(
        display.style.format({
            "Mean success (%)": "{:.2f}",
            "Std dev (%)": "{:.2f}"
        }, na_rep="N/A"),
        use_container_width=True,
        hide_index=True
    )

    # Comparison table.
    pivot = create_success_table(
        filtered_summary,
        experiment_type,
        world
    )

    if not pivot.empty:
        st.markdown("**Methodology comparison**")
        st.dataframe(
            pivot,
            use_container_width=True
        )

    st.download_button(
        "Download statistics (CSV)",
        data=filtered_summary.to_csv(
            index=False
        ).encode("utf-8"),
        file_name=f"success_statistics_{key_suffix}.csv",
        mime="text/csv",
        key=f"download_stats_{key_suffix}"
    )

    # --------------------------------------------------------
    # LATEX TABLE EXPORT
    # --------------------------------------------------------

    latex_table = create_success_latex_table(
        summary=filtered_summary,
        experiment_type=experiment_type,
        world=world,
        selected_methods=selected_methods,
        caption=(
            f"Success rate comparison: "
            f"{experiment_type}, {world}."
        ),
        label=f"tab:success_{experiment_type}_{world}"
    )

    if latex_table is not None:

        st.download_button(
            label="Download LaTeX table",
            data=latex_table,
            file_name=f"success_rate_{experiment_type}_{world}.tex",
            mime="text/plain",
            key=f"download_latex_{key_suffix}"
        )

        with st.expander("Preview LaTeX table"):
            st.code(latex_table, language="latex")


    # Individual trials.
    with st.expander("Show individual trial statistics"):

        st.dataframe(
            filtered_trials.sort_values(
                ["methodology", "n_sensors", "trial"]
            ),
            use_container_width=True,
            hide_index=True
        )

        st.download_button(
            "Download trial statistics (CSV)",
            data=filtered_trials.to_csv(
                index=False
            ).encode("utf-8"),
            file_name=f"trial_statistics_{key_suffix}.csv",
            mime="text/csv",
            key=f"download_trials_{key_suffix}"
        )
        
def render_world_dashboard(
    summary,
    trial_stats,
    world,
    selected_methods,
    selected_sensors,
    key_suffix,
    efficiency_summary
):
    """
    Render deterministic and uncertain analyses side by side.

    Layout:
        Row 1: Two comparison plots.
        Row 2: Two summary statistics sections.
        Row 3: Individual run details.
    """

    # ========================================================
    # ROW 1 — SIDE-BY-SIDE PLOTS
    # ========================================================

    col_det, col_unc = st.columns(2, gap="large")

    with col_det:

        st.subheader("Deterministic - Success Rate")

        render_success_plot(
            summary=summary,
            experiment_type="deterministic",
            world=world,
            key_suffix=f"{key_suffix}_det",
            selected_methods=selected_methods,
            selected_sensors=selected_sensors
        )

    with col_unc:

        st.subheader("Uncertain - Success Rate")

        render_success_plot(
            summary=summary,
            experiment_type="uncertain",
            world=world,
            key_suffix=f"{key_suffix}_unc",
            selected_methods=selected_methods,
            selected_sensors=selected_sensors
        )

    st.divider()

    # ========================================================
    # ROW 2 — SIDE-BY-SIDE STATISTICS
    # ========================================================

    col_det, col_unc = st.columns(2, gap="large")

    with col_det:

        st.subheader("Deterministic — Statistics")

        render_success_tables(
            summary=summary,
            trial_stats=trial_stats,
            experiment_type="deterministic",
            world=world,
            selected_methods=selected_methods,
            selected_sensors=selected_sensors,
            key_suffix=f"{key_suffix}_det"
        )

    with col_unc:

        st.subheader("Uncertain — Statistics")

        render_success_tables(
            summary=summary,
            trial_stats=trial_stats,
            experiment_type="uncertain",
            world=world,
            selected_methods=selected_methods,
            selected_sensors=selected_sensors,
            key_suffix=f"{key_suffix}_unc"
        )

        st.divider()

    # ========================================================
    # ROW 3 — INSPECTION EFFICIENCY PLOTS
    # ========================================================

    col_det, col_unc = st.columns(2, gap="large")

    with col_det:

        st.subheader("Deterministic — Inspection Efficiency")

        render_efficiency_analysis(
            summary=efficiency_summary,
            experiment_type="deterministic",
            world=world,
            selected_methods=selected_methods,
            selected_sensors=selected_sensors,
            key_suffix=f"{key_suffix}_det"
        )

    with col_unc:

        st.subheader("Uncertain — Inspection Efficiency")

        render_efficiency_analysis(
            summary=efficiency_summary,
            experiment_type="uncertain",
            world=world,
            selected_methods=selected_methods,
            selected_sensors=selected_sensors,
            key_suffix=f"{key_suffix}_unc"
        )


# ============================================================
# EXPERIMENT PARSING
# ============================================================
def extract_world_runs(
    records,
    world_data,
    experiment_type,
    methodology,
    world
):

    def recursive_extract(node, path=()):

        if not isinstance(node, dict):
            return

        if (
            "success" in node
            and "sensor_sequence" in node
        ):

            success = node.get("success")
            sensors = node.get("sensor_sequence")

            if (
                success not in (0, 1, 0.0, 1.0)
                or not isinstance(sensors, list)
            ):
                return

            trial_parts = [
                part for part in path
                if str(part).startswith("trial_")
            ]

            trial_id = (
                "/".join(
                    path[:path.index(trial_parts[-1]) + 1]
                )
                if trial_parts
                else "/".join(path[:-1])
            )

            run_id = path[-1] if path else "run_1"

            model = (
                path[0]
                if path and not str(path[0]).startswith("trial_")
                else "default"
            )

            records.append({
                "experiment_type": experiment_type,
                "methodology": methodology,
                "world": world,
                "model": model,
                "trial": trial_id or "trial_1",
                "run": run_id,
                "sensor_sequence": sensors,
                "n_sensors": len(sensors),
                "success": int(success)
            })

            return

        for key, value in node.items():

            if isinstance(value, dict):
                recursive_extract(
                    value,
                    path + (key,)
                )

    recursive_extract(world_data)



def extract_experiment_records(data):

    records = []

    if not isinstance(data, dict):
        return pd.DataFrame()

    for experiment_type in EXPERIMENT_TYPES:

        methodologies = data.get(experiment_type, {})

        if not isinstance(methodologies, dict):
            continue

        for methodology, methodology_data in methodologies.items():

            if not isinstance(methodology_data, dict):
                continue

            # ====================================================
            # ILP
            # ====================================================

            if methodology == "ILP":

                for variant, variant_data in methodology_data.items():

                    match = re.fullmatch(r"B(\d+)", str(variant))

                    if not match:
                        continue

                    if not isinstance(variant_data, dict):
                        continue

                    method_label = f"ILP-{match.group(1)}"

                    for world, world_data in variant_data.items():

                        if not str(world).startswith("world_"):
                            continue

                        if not isinstance(world_data, dict):
                            continue

                        extract_world_runs(
                            records,
                            world_data,
                            experiment_type,
                            method_label,
                            world
                        )

            # ====================================================
            # HITL, AUTO, HUMAN
            # ORIGINAL PARSING
            # ====================================================

            else:

                for world, world_data in methodology_data.items():

                    if not isinstance(world_data, dict):
                        continue

                    extract_world_runs(
                        records,
                        world_data,
                        experiment_type,
                        methodology,
                        world
                    )

    df = pd.DataFrame(records)

    if not df.empty:
        df["n_sensors"] = df["n_sensors"].astype(int)
        df["success"] = df["success"].astype(int)

    return df



# ============================================================
# WORLD DISCOVERY
# ============================================================

def discover_worlds(data):

    worlds = set()

    if not isinstance(data, dict):
        return []

    for experiment_type in EXPERIMENT_TYPES:

        methodologies = data.get(experiment_type, {})

        if not isinstance(methodologies, dict):
            continue

        for methodology, methodology_data in methodologies.items():

            if not isinstance(methodology_data, dict):
                continue

            if methodology == "ILP":

                for variant_data in methodology_data.values():

                    if not isinstance(variant_data, dict):
                        continue

                    for world in variant_data:

                        if str(world).startswith("world_"):
                            worlds.add(world)

            else:

                for world in methodology_data:

                    if str(world).startswith("world_"):
                        worlds.add(world)

    return sorted(worlds, key=natural_key)

# ============================================================
# SUCCESS RATE CALCULATION
# ============================================================


def calculate_success_rates(df, aggregate_worlds=False):
    """
    Calculate success statistics directly from individual runs.

    trial_stats:
        One row per trial, world and sensor configuration.
        Used for inspecting individual trial results.

    summary:
        Statistics calculated directly from ALL individual runs
        sharing the same experimental configuration.

    If aggregate_worlds=True:
        All worlds are combined, while trial_stats still
        preserves the original world information.
    """

    if df.empty:
        return pd.DataFrame(), pd.DataFrame()

    # ========================================================
    # STEP 1 - TRIAL-LEVEL STATISTICS (DIAGNOSTICS)
    # ========================================================

    trial_columns = [
        "experiment_type",
        "methodology",
        "world",
        "trial",
        "n_sensors"
    ]

    trial_stats = (
        df.groupby(trial_columns, as_index=False)
        .agg(
            successful_runs=("success", "sum"),
            total_runs=("success", "count")
        )
    )

    trial_stats["success_rate"] = (
        trial_stats["successful_runs"]
        / trial_stats["total_runs"]
    )

    # ========================================================
    # STEP 2 - DEFINE SUMMARY GROUPING
    # ========================================================

    summary_columns = [
        "experiment_type",
        "methodology",
        "n_sensors"
    ]

    if not aggregate_worlds:
        summary_columns.append("world")

    # ========================================================
    # STEP 3 - CALCULATE STATISTICS FROM INDIVIDUAL RUNS
    # ========================================================

    summary = (
        df.groupby(summary_columns, as_index=False)
        .agg(
            # Mean of all binary run outcomes (0 or 1).
            mean_success_rate=("success", "mean"),

            # Sample standard deviation across all individual runs.
            std_dev=(
                "success",
                lambda values: values.std(ddof=VARIANCE_DDOF)
            ),

            # Number of individual runs.
            total_runs=("success", "count"),

            # Number of successful individual runs.
            successful_runs=("success", "sum")
        )
    )

    # ========================================================
    # STEP 4 - COUNT DISTINCT TRIAL OBSERVATIONS
    # ========================================================

    # Count trial/configuration combinations.
    # In the Overview, trials belonging to different worlds
    # remain distinct observations.
    trial_counts = (
        trial_stats.groupby(
            summary_columns,
            as_index=False
        )
        .size()
        .rename(columns={"size": "n_trials"})
    )

    summary = summary.merge(
        trial_counts,
        on=summary_columns,
        how="left"
    )

    return trial_stats, summary




# ============================================================
# SUCCESS RATE FIGURE
# ============================================================

def create_success_figure(summary, experiment_type, world):
    """
    Create a single comparison plot containing all methodologies.

    Each methodology is represented by a separate line:
        - HITL
        - AUTO
        - HUMAN
        - ILP

    The x-axis represents the number of sensors.
    The y-axis represents the mean success rate.
    Error bars represent the sample standard deviation.
    """

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if filtered.empty:
        return None

    available_methods = [
        method for method in METHODOLOGIES
        if method in filtered["methodology"].unique()
    ]

    if not available_methods:
        return None

    palette = {
        "HITL": "#1f77b4",
        "AUTO": "#ff7f0e",
        "HUMAN": "#2ca02c",
        "ILP-30": "#9467bd",
        "ILP-60": "#d62728",
        "ILP-120": "#17becf"
    }

    fig = go.Figure()

    for method in available_methods:

        method_data = (
            filtered[
                filtered["methodology"] == method
            ]
            .sort_values("n_sensors")
        )

        if method_data.empty:
            continue

        # Replace undefined standard deviations with zero-length
        # error bars, for example when only one trial is available.
        std_values = method_data["std_dev"].fillna(0)

        fig.add_trace(
            go.Scatter(
                x=method_data["n_sensors"],
                y=method_data["mean_success_rate"],
                
                error_y=dict(
                    type="data",
                    array=std_values,
                    visible=True,
                    thickness=1.5,
                    width=5
                ),

                mode="lines+markers",
                name=method,

                line=dict(
                    color=palette.get(method, "#7f7f7f"),
                    width=2.5
                ),

                marker=dict(size=8),

                customdata=method_data[
                    [
                        "std_dev",
                        "n_trials",
                        "total_runs",
                        "successful_runs"
                    ]
                ].fillna(0).values,

                hovertemplate=(
                    "Methodology: " + method +
                    "<br>Number of sensors: %{x}" +
                    "<br>Mean success rate: %{y:.2f}%" +
                    "<br>Std deviation: %{customdata[0]:.2f}%" +
                    "<br>Number of trials: %{customdata[1]}" +
                    "<br>Successful runs: %{customdata[3]}" +
                    "<br>Total runs: %{customdata[2]}" +
                    "<extra></extra>"
                )
            )
        )

    fig.update_layout(
        title=dict(
            text=(
                f"{experiment_type.capitalize()} "
                f"— Success Rate ({world})"
            ),
            font=dict(size=18),
            x=0.5,
            xanchor="center",
            y=0.95,
            yanchor="top"
        ),

        xaxis=dict(
            title="Number of sensors",
            range=[0, 5],
            dtick=1,
            showgrid=True,
            gridcolor="rgba(128,128,128,0.2)"
        ),

        yaxis=dict(
            title="Mean success rate",
            range=[-0.25, 1.25],
            ticksuffix="",
            showgrid=True,
            gridcolor="rgba(128,128,128,0.2)"
        ),

        template="plotly_white",
        height=550,
        autosize=True,

        hovermode="x unified",

        legend=dict(
            orientation="h",
            yanchor="top",
            y=-0.25,
            xanchor="center",
            x=0.5,
            font=dict(size=11),
            bgcolor="rgba(255,255,255,0.8)",
            itemsizing="constant",
            itemwidth=70
        ),

        margin=dict(
            l=70,
            r=40,
            t=100,
            b=65
        )
    )

    return fig



# ============================================================
# SUCCESS RATE SUMMARY TABLE
# ============================================================

def create_success_table(summary, experiment_type, world):
    """
    Create a comparison table containing mean and standard deviation.

    Each row corresponds to one sensor configuration.
    """

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if filtered.empty:
        return pd.DataFrame()

    filtered["mean ± std_dev"] = filtered.apply(
        lambda row: (
            f"{row['mean_success_rate']:.2f}% ± "
            f"{row['std_dev']:.2f}%"
            if pd.notna(row["std_dev"])
            else f"{row['mean_success_rate']:.2f}% ± N/A"
        ),
        axis=1
    )

    pivot = filtered.pivot(
        index="n_sensors",
        columns="methodology",
        values="mean ± std_dev"
    )

    return pivot.reindex(
        columns=[
            method for method in METHODOLOGIES
            if method in pivot.columns
        ]
    ).sort_index()


def create_efficiency_table(summary, experiment_type, world):
    """
    Create a methodology comparison table for inspection efficiency.

    Expected input:
        A DataFrame containing one row per methodology,
        experiment type, world and number of sensors.

    Output:
        A pivot DataFrame where:
            - Rows represent the number of sensors.
            - Columns represent methodologies.
            - Values represent mean efficiency +/- standard deviation.
    """

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if filtered.empty:
        return pd.DataFrame()

    filtered["mean ± std_dev"] = filtered.apply(
        lambda row: (
            f"{row['mean_efficiency']:.2f} ± "
            f"{row['std_dev']:.2f}"
            if pd.notna(row["std_dev"])
            else f"{row['mean_efficiency']:.2f} ± N/A"
        ),
        axis=1
    )

    pivot = filtered.pivot(
        index="n_sensors",
        columns="methodology",
        values="mean ± std_dev"
    )

    return pivot.reindex(
        columns=[
            method for method in METHODOLOGIES
            if method in pivot.columns
        ]
    ).sort_index()


# ============================================================
# SUMMARY STATISTICS TABLE
# ============================================================

def show_success_statistics(summary, experiment_type, world):

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if filtered.empty:
        st.warning("No statistics available.")
        return

    st.subheader("Summary statistics")

    display = filtered[
        [
            "methodology",
            "n_sensors",
            "mean_success_rate",
            "std_dev",
            "n_trials",
            "successful_runs",
            "total_runs"
        ]
    ].copy()

    display.columns = [
        "Methodology",
        "Number of sensors",
        "Mean success rate (%)",
        "Std deviation (%)",
        "Number of trials",
        "Successful runs",
        "Total runs"
    ]

    st.dataframe(
        display.style.format({
            "Mean success rate (%)": "{:.2f}",
            "Std deviation (%)": "{:.2f}"
        }, na_rep="N/A"),
        use_container_width=True
    )

    st.subheader("Comparison table")

    pivot = create_success_table(
        summary,
        experiment_type,
        world
    )

    if not pivot.empty:
        st.dataframe(
            pivot,
            use_container_width=True
        )

    st.download_button(
        "Download statistics (CSV)",
        data=filtered.to_csv(index=False).encode("utf-8"),
        file_name=f"success_statistics_{experiment_type}_{world}.csv",
        mime="text/csv",
        key=f"download_success_{experiment_type}_{world}"
    )


# ============================================================
# INSPECTION EFFICIENCY — DATA EXTRACTION
# ============================================================

def extract_efficiency_records(data):
    """
    Extract individual run results from inspection_matrices.json.

    Expected input hierarchy:
        experiment_type
            -> methodology
                -> world
                    -> model
                        -> trial
                            -> run
                                -> run_result

    ILP models are converted into methodology labels:
        ILP / B30  -> ILP-30
        ILP / B60  -> ILP-60
        ILP / B120 -> ILP-120

    Output:
        A pandas DataFrame containing one row per run.

    Each row contains:
        experiment_type
        methodology
        world
        model
        trial
        run
        sensor_sequence
        n_sensors
        n_inspected_areas
        min_inspected_areas
        n_total_areas
        success_rate
        efficiency

    The original matrix is not expanded into columns because
    it is not needed for the efficiency aggregation.
    """

    records = []

    if not isinstance(data, dict):
        return pd.DataFrame()

    for experiment_type, methodologies in data.items():

        if not isinstance(methodologies, dict):
            continue

        for methodology, worlds in methodologies.items():

            if not isinstance(worlds, dict):
                continue

            for world, models in worlds.items():

                if not isinstance(models, dict):
                    continue

                for model, trials in models.items():

                    if not isinstance(trials, dict):
                        continue

                    # ----------------------------------------
                    # Reconstruct methodology label
                    # ----------------------------------------

                    if methodology == "ILP":

                        match = re.fullmatch(
                            r"B(\d+)",
                            str(model)
                        )

                        if not match:
                            continue

                        method_label = f"ILP-{match.group(1)}"

                    else:
                        method_label = methodology

                    # ----------------------------------------
                    # Extract trials and runs
                    # ----------------------------------------

                    for trial, runs in trials.items():

                        if not isinstance(runs, dict):
                            continue

                        for run, run_data in runs.items():

                            if not isinstance(run_data, dict):
                                continue

                            sensor_sequence = run_data.get(
                                "sensor_sequence"
                            )

                            efficiency = run_data.get(
                                "efficiency"
                            )

                            if not isinstance(sensor_sequence, list):
                                continue

                            if efficiency is None:
                                continue

                            records.append({

                                "experiment_type": experiment_type,

                                "methodology": method_label,

                                "world": world,

                                "model": model,

                                "trial": trial,

                                "run": run,

                                "sensor_sequence": sensor_sequence,

                                "n_sensors": len(sensor_sequence),

                                "n_inspected_areas": run_data.get(
                                    "n_inspected_areas"
                                ),

                                "min_inspected_areas": run_data.get(
                                    "min_inspected_areas"
                                ),

                                "n_total_areas": run_data.get(
                                    "n_total_areas"
                                ),

                                "success_rate": run_data.get(
                                    "success_rate"
                                ),

                                "efficiency": float(efficiency)

                            })

    df = pd.DataFrame(records)

    if not df.empty:

        df["n_sensors"] = df["n_sensors"].astype(int)

        df["efficiency"] = pd.to_numeric(
            df["efficiency"],
            errors="coerce"
        )

        df = df.dropna(
            subset=["efficiency", "n_sensors"]
        )

    return df


# ============================================================
# INSPECTION EFFICIENCY — AGGREGATION
# ============================================================

def calculate_efficiency_statistics(
    df,
    aggregate_worlds=False
):
    """
    Calculate inspection efficiency statistics.

    Aggregation criteria:
        - experiment_type
        - methodology
        - number of sensors
        - world (unless aggregate_worlds=True)

    Every individual run contributes equally to the mean.

    Output:
        summary:
            One row per experimental configuration.

            Columns:
                experiment_type
                methodology
                n_sensors
                world (when aggregate_worlds=False)
                mean_efficiency
                std_dev
                total_runs

        When aggregate_worlds=True, runs from different worlds
        are combined into the same group.

    Standard deviation is calculated across individual runs
    using VARIANCE_DDOF.
    """

    if df.empty:
        return pd.DataFrame()

    grouping_columns = [
        "experiment_type",
        "methodology",
        "n_sensors"
    ]

    if not aggregate_worlds:
        grouping_columns.append("world")

    summary = (
        df.groupby(
            grouping_columns,
            as_index=False
        )
        .agg(
            mean_efficiency=("efficiency", "mean"),

            std_dev=(
                "efficiency",
                lambda values: values.std(
                    ddof=VARIANCE_DDOF
                )
            ),

            total_runs=("efficiency", "count")
        )
    )

    return summary


# ============================================================
# INSPECTION EFFICIENCY — FIGURE
# ============================================================

def create_efficiency_figure(
    summary,
    experiment_type,
    world
):
    """
    Create an inspection efficiency comparison plot.

    X-axis:
        Number of sensors.

    Y-axis:
        Mean inspection efficiency.

    Each methodology has a separate curve.

    Error bars:
        Standard deviation across individual runs.

    The input summary must already be filtered by world
    and experiment type.
    """

    filtered = summary[
        (summary["experiment_type"] == experiment_type)
        & (summary["world"] == world)
    ].copy()

    if filtered.empty:
        return None

    available_methods = [
        method for method in METHODOLOGIES
        if method in filtered["methodology"].unique()
    ]

    if not available_methods:
        return None

    palette = {
        "HITL": "#1f77b4",
        "AUTO": "#ff7f0e",
        "HUMAN": "#2ca02c",
        "ILP-30": "#9467bd",
        "ILP-60": "#d62728",
        "ILP-120": "#17becf"
    }

    fig = go.Figure()

    for method in available_methods:

        method_data = (
            filtered[
                filtered["methodology"] == method
            ]
            .sort_values("n_sensors")
        )

        if method_data.empty:
            continue

        std_values = method_data["std_dev"].fillna(0)

        fig.add_trace(
            go.Scatter(

                x=method_data["n_sensors"],

                y=method_data["mean_efficiency"],

                error_y=dict(
                    type="data",
                    array=std_values,
                    visible=True,
                    thickness=1.5,
                    width=5
                ),

                mode="lines+markers",

                name=method,

                line=dict(
                    color=palette.get(
                        method,
                        "#7f7f7f"
                    ),
                    width=2.5
                ),

                marker=dict(size=8),

                customdata=method_data[
                    ["std_dev", "total_runs"]
                ].fillna(0).values,

                hovertemplate=(
                    "Methodology: " + method +
                    "<br>Number of sensors: %{x}" +
                    "<br>Mean efficiency: %{y:.3f}" +
                    "<br>Std deviation: %{customdata[0]:.3f}" +
                    "<br>Number of runs: %{customdata[1]}" +
                    "<extra></extra>"
                )
            )
        )

    fig.update_layout(

        title=dict(
            text=(
                f"{experiment_type.capitalize()} "
                f"— Inspection Efficiency ({world})"
            ),
            font=dict(size=18),
            x=0.5,
            xanchor="center"
        ),

        xaxis=dict(
            title="Number of sensors",
            showgrid=True,
            gridcolor="rgba(128,128,128,0.2)",
            dtick=1
        ),

        yaxis=dict(
            title="Mean inspection efficiency",
            range=[0, 1.05],
            showgrid=True,
            gridcolor="rgba(128,128,128,0.2)"
        ),

        template="plotly_white",

        height=550,

        hovermode="x unified",

        legend=dict(
            orientation="h",
            yanchor="top",
            y=-0.25,
            xanchor="center",
            x=0.5,
            font=dict(size=11),
            bgcolor="rgba(255,255,255,0.8)",
            itemsizing="constant",
            itemwidth=70
        ),

        margin=dict(
            l=70,
            r=40,
            t=100,
            b=65
        )
    )

    return fig


# ============================================================
# INSPECTION EFFICIENCY — DASHBOARD RENDERING
# ============================================================

def render_efficiency_analysis(
    summary,
    experiment_type,
    world,
    selected_methods,
    selected_sensors,
    key_suffix
):
    """
    Render inspection efficiency analysis.

    Input DataFrame structure:
        experiment_type
        methodology
        world
        n_sensors
        mean_efficiency
        std_dev
        total_runs

    The function:
        1. Filters methodologies and sensor counts.
        2. Generates the efficiency comparison plot.
        3. Provides JPEG and HTML plot downloads.
        4. Displays summary statistics.
        5. Provides CSV and LaTeX table downloads.
        6. Displays a LaTeX preview.
    """

    filtered_summary = summary[
        (summary["experiment_type"] == experiment_type)
        & summary["methodology"].isin(selected_methods)
        & summary["n_sensors"].isin(selected_sensors)
    ].copy()

    if filtered_summary.empty:

        st.info(
            f"No efficiency data available for "
            f"{experiment_type} — {world}."
        )

        return

    # --------------------------------------------------------
    # EFFICIENCY PLOT
    # --------------------------------------------------------

    fig = create_efficiency_figure(
        filtered_summary,
        experiment_type,
        world
    )

    if fig is not None:

        st.plotly_chart(
            fig,
            use_container_width=True,
            config={
                "scrollZoom": True,
                "displaylogo": False,
                "responsive": True
            },
            key=f"efficiency_plot_{key_suffix}"
        )

        col1, col2 = st.columns(2)

        # ----------------------------------------------------
        # JPEG EXPORT
        # ----------------------------------------------------

        with col1:

            try:

                jpeg_image = fig.to_image(
                    format="jpeg",
                    width=1600,
                    height=600,
                    scale=2
                )

                st.download_button(
                    label="Download plot (JPEG)",
                    data=jpeg_image,
                    file_name=(
                        f"inspection_efficiency_"
                        f"{key_suffix}.jpg"
                    ),
                    mime="image/jpeg",
                    key=f"download_efficiency_jpeg_{key_suffix}"
                )

            except Exception as exc:

                st.warning(
                    f"Unable to generate JPEG image: {exc}"
                )


    # --------------------------------------------------------
    # SUMMARY STATISTICS
    # --------------------------------------------------------

    st.subheader("Efficiency statistics")

    display = filtered_summary[
        [
            "methodology",
            "n_sensors",
            "mean_efficiency",
            "std_dev",
            "total_runs"
        ]
    ].copy()

    display.columns = [
        "Methodology",
        "Number of sensors",
        "Mean efficiency",
        "Std deviation",
        "Total runs"
    ]

    st.dataframe(
        display.style.format({
            "Mean efficiency": "{:.3f}",
            "Std deviation": "{:.3f}"
        }, na_rep="N/A"),
        use_container_width=True,
        hide_index=True
    )

    # --------------------------------------------------------
    # CSV EXPORT
    # --------------------------------------------------------

    st.download_button(
        label="Download statistics (CSV)",
        data=filtered_summary.to_csv(
            index=False
        ).encode("utf-8"),
        file_name=(
            f"inspection_efficiency_statistics_"
            f"{key_suffix}.csv"
        ),
        mime="text/csv",
        key=f"download_efficiency_csv_{key_suffix}"
    )

    # --------------------------------------------------------
    # METHODOLOGY COMPARISON
    # --------------------------------------------------------

    pivot = create_efficiency_table(
        filtered_summary,
        experiment_type,
        world
    )

    if not pivot.empty:
        st.markdown("**Methodology comparison**")
        st.dataframe(
            pivot,
            use_container_width=True
        )

    # --------------------------------------------------------
    # LATEX TABLE EXPORT
    # --------------------------------------------------------

    latex_table = create_efficiency_latex_table(
        summary=filtered_summary,
        experiment_type=experiment_type,
        world=world,
        selected_methods=selected_methods,
        caption=(
            f"Inspection efficiency comparison: "
            f"{experiment_type}, {world}."
        ),
        label=(
            f"tab:efficiency_{experiment_type}_{world}"
        )
    )

    if latex_table is not None:

        st.download_button(
            label="Download LaTeX table",
            data=latex_table,
            file_name=(
                f"inspection_efficiency_"
                f"{experiment_type}_{world}.tex"
            ),
            mime="text/plain",
            key=f"download_efficiency_latex_{key_suffix}"
        )

        with st.expander("Preview LaTeX table"):

            st.code(
                latex_table,
                language="latex"
            )

# ============================================================
# STREAMLIT CONFIGURATION
# ============================================================

st.set_page_config(
    page_title="LLM Experiments Analysis",
    page_icon="📊",
    layout="wide"
)

st.markdown(
    """
    <style>
        [data-testid="stSidebar"] {
            min-width: 350px !important;
            max-width: 350px !important;
        }

        [data-testid="stSidebar"] > div:first-child {
            width: 350px !important;
        }

        .block-container {
            padding-top: 2rem;
        }
    </style>
    """,
    unsafe_allow_html=True
)

st.title("LLM Experiments — Interactive Analysis")

st.caption(
    "Success rate and error analysis across experimental configurations."
)


# ============================================================
# LOAD INPUT FILES
# ============================================================

if not EVALUATIONS_PATH.exists():
    st.error(f"File not found: {EVALUATIONS_PATH}")
    st.stop()

try:

    # Load all available methodology files.
    experiments_data = load_all_experiments(
        EXPERIMENTS_PATHS
    )

    evaluations_data = load_json(
        str(EVALUATIONS_PATH),
        EVALUATIONS_PATH.stat().st_mtime
    )

except (json.JSONDecodeError, OSError, ValueError) as exc:

    st.error(f"Unable to load JSON files: {exc}")
    st.stop()


# Load inspection matrices and individual run efficiencies.

if not INSPECTION_MATRICES_PATH.exists():
    st.error(
        f"File not found: {INSPECTION_MATRICES_PATH}"
    )
    st.stop()

inspection_data = load_json(
    str(INSPECTION_MATRICES_PATH),
    INSPECTION_MATRICES_PATH.stat().st_mtime
)

# ============================================================
# DATA PREPARATION
# ============================================================

experiments_df = extract_experiment_records(
    experiments_data
)

if experiments_df.empty:
    st.error(
        "No valid experiment runs were found in experiments.json."
    )
    st.stop()


# Calculate statistics independently for each world.
trial_stats_world, summary_world = calculate_success_rates(
    experiments_df,
    aggregate_worlds=False
)

# Calculate statistics across all worlds.
trial_stats_overview, summary_overview = calculate_success_rates(
    experiments_df,
    aggregate_worlds=True
)

# Explicitly identify the overview in the summary DataFrame.
summary_overview["world"] = "Overview"


# ============================================================
# INSPECTION EFFICIENCY DATA PREPARATION
# ============================================================

efficiency_df = extract_efficiency_records(
    inspection_data
)

if efficiency_df.empty:

    st.warning(
        "No valid inspection efficiency records found."
    )

    st.stop()


# --------------------------------------------
# Statistics per world
# --------------------------------------------

efficiency_summary_world = calculate_efficiency_statistics(
    efficiency_df,
    aggregate_worlds=False
)


# --------------------------------------------
# Statistics across all worlds
# --------------------------------------------

efficiency_summary_overview = calculate_efficiency_statistics(
    efficiency_df,
    aggregate_worlds=True
)

efficiency_summary_overview["world"] = "Overview"

# ============================================================
# WORLD DISCOVERY
# ============================================================

all_worlds = discover_worlds(experiments_data)

# Ensure that world_1 and world_2 are always available
# when they are part of the expected experiment configuration.
for expected_world in WORLD_ORDER:

    if expected_world not in all_worlds:
        all_worlds.append(expected_world)

all_worlds = sorted(
    all_worlds,
    key=natural_key
)


# ============================================================
# SIDEBAR FILTERS
# ============================================================

st.sidebar.header("Data filters")

selected_worlds = st.sidebar.multiselect(
    "Worlds",
    options=all_worlds,
    default=all_worlds
)

available_methods = sorted(
    set(experiments_df["methodology"].unique())
    | set(efficiency_df["methodology"].unique()),
    key=natural_key
)

selected_methods = st.sidebar.multiselect(
    "Methodologies",
    options=available_methods,
    default=available_methods
)


available_sensors = sorted(
    set(experiments_df["n_sensors"].unique())
    | set(efficiency_df["n_sensors"].unique())
)

selected_sensors = st.sidebar.multiselect(
    "Number of sensors",
    options=available_sensors,
    default=available_sensors
)



# ============================================================
# SUCCESS RATE ANALYSIS RENDERING
# ============================================================

def render_success_analysis(
    summary,
    trial_stats,
    experiment_type,
    world,
    selected_methods,
    selected_sensors,
    key_suffix
):
    """
    Render one experiment-type analysis.

    The plot compares all selected methodologies in a single figure.
    Summary statistics and trial-level results are displayed below.
    """

    filtered_summary = summary[
        (summary["experiment_type"] == experiment_type)
        & summary["methodology"].isin(selected_methods)
        & summary["n_sensors"].isin(selected_sensors)
    ].copy()

    filtered_trials = trial_stats[
        (trial_stats["experiment_type"] == experiment_type)
        & trial_stats["methodology"].isin(selected_methods)
        & trial_stats["n_sensors"].isin(selected_sensors)
    ].copy()

    if filtered_summary.empty:
        st.warning(
            f"No valid results for {experiment_type} — {world}."
        )
        return

    # --------------------------------------------------------
    # KEY METRICS
    # --------------------------------------------------------

    col1, col2, col3 = st.columns(3)

    col1.metric(
        "Total runs",
        int(filtered_trials["total_runs"].sum())
    )

    col2.metric(
        "Successful runs",
        int(filtered_trials["successful_runs"].sum())
    )

    col3.metric(
        "Sensor configurations",
        filtered_summary["n_sensors"].nunique()
    )

    # --------------------------------------------------------
    # COMPARISON PLOT
    # --------------------------------------------------------

    st.subheader(
        f"{experiment_type.capitalize()} — Success Rate"
    )

    fig = create_success_figure(
        filtered_summary,
        experiment_type,
        world
    )

    if fig is not None:

        st.plotly_chart(
            fig,
            use_container_width=True,
            config={
                "scrollZoom": True,
                "displaylogo": False,
                "responsive": True
            },
            key=f"success_plot_{key_suffix}"
        )


    # --------------------------------------------------------
    # SUMMARY STATISTICS
    # --------------------------------------------------------

    show_success_statistics(
        filtered_summary,
        experiment_type,
        world
    )

    # --------------------------------------------------------
    # INDIVIDUAL TRIALS
    # --------------------------------------------------------

    with st.expander("Show individual trial statistics"):

        st.dataframe(
            filtered_trials.sort_values(
                ["methodology", "n_sensors", "trial"]
            ),
            use_container_width=True,
            hide_index=True
        )

        st.download_button(
            "Download trial statistics (CSV)",
            data=filtered_trials.to_csv(
                index=False
            ).encode("utf-8"),
            file_name=f"trial_statistics_{key_suffix}.csv",
            mime="text/csv",
            key=f"download_trials_{key_suffix}"
        )


# ============================================================
# MAIN SUCCESS RATE DASHBOARD
# ============================================================

st.header("Intruder Detection Analysis")

world_tabs = st.tabs(
    ["Overview"] + selected_worlds
)



# ========================================================
# OVERVIEW
# ========================================================

with world_tabs[0]:

    st.header("Overview — All selected worlds")

    # Preserve trial-level data for individual trial inspection.
    overview_trials = trial_stats_overview.copy()

    # Apply the selected worlds to trial-level data.
    if selected_worlds:
        overview_trials = overview_trials[
            overview_trials["world"].isin(selected_worlds)
        ]

    # Select the original individual runs belonging to
    # the selected worlds.
    overview_experiments = experiments_df.copy()

    if selected_worlds:
        overview_experiments = overview_experiments[
            overview_experiments["world"].isin(selected_worlds)
        ]

    # Recalculate statistics directly from ALL individual runs.
    _, overview_data = calculate_success_rates(
        overview_experiments,
        aggregate_worlds=True
    )

    overview_data["world"] = "Overview"

    # Select efficiency records belonging to selected worlds.

    overview_efficiency_records = efficiency_df.copy()

    if selected_worlds:

        overview_efficiency_records = overview_efficiency_records[
            overview_efficiency_records["world"].isin(
                selected_worlds
            )
        ]

    # Recalculate efficiency statistics across selected worlds.

    overview_efficiency_summary = calculate_efficiency_statistics(
        overview_efficiency_records,
        aggregate_worlds=True
    )

    overview_efficiency_summary["world"] = "Overview"

    render_world_dashboard(
        summary=overview_data,
        trial_stats=overview_trials,
        world="Overview",
        selected_methods=selected_methods,
        selected_sensors=selected_sensors,
        key_suffix="overview",
        efficiency_summary=overview_efficiency_summary
    )


# ============================================================
# INDIVIDUAL WORLD TABS
# ============================================================

for world_index, world in enumerate(selected_worlds, start=1):

    with world_tabs[world_index]:

        st.header(f"Results — {world}")

        world_summary = summary_world[
            summary_world["world"] == world
        ].copy()

        world_trials = trial_stats_world[
            trial_stats_world["world"] == world
        ].copy()

        render_world_dashboard(
            summary=world_summary,
            trial_stats=world_trials,
            world=world,
            selected_methods=selected_methods,
            selected_sensors=selected_sensors,
            key_suffix=world,
            efficiency_summary=efficiency_summary_world[
                efficiency_summary_world["world"] == world
            ].copy()
        )