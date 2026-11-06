
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

METHODOLOGIES = ["HITL", "AUTO", "HUMAN", "ILP"]

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



def render_success_plot(
    summary,
    experiment_type,
    world,
    key_suffix
):
    """Render and export a success rate comparison plot."""

    fig = create_success_figure(
        summary,
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

        # ----------------------------------------------------
        # JPEG EXPORT
        # ----------------------------------------------------

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
        st.info(f"No data available for {experiment_type}.")



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

def render_success_plot(
    summary,
    experiment_type,
    world,
    key_suffix
):
    """Render a compact success rate comparison plot."""

    fig = create_success_figure(
        summary,
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
    else:
        st.info(f"No data available for {experiment_type}.")


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
    key_suffix
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

        st.subheader("Deterministic")

        render_success_plot(
            summary=summary,
            experiment_type="deterministic",
            world=world,
            key_suffix=f"{key_suffix}_det"
        )

    with col_unc:

        st.subheader("Uncertain")

        render_success_plot(
            summary=summary,
            experiment_type="uncertain",
            world=world,
            key_suffix=f"{key_suffix}_unc"
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
# ============================================================
# EXPERIMENT PARSING
# ============================================================

def extract_experiment_records(data):
    """
    Flatten experiments.json into one row per valid run.

    The parser supports optional intermediate hierarchy levels,
    such as LLM model names between worlds and trials.

    A run is identified by the presence of both:
        - sensor_sequence
        - success
    """

    records = []

    if not isinstance(data, dict):
        return pd.DataFrame()

    for experiment_type in EXPERIMENT_TYPES:

        methodologies = data.get(experiment_type, {})

        if not isinstance(methodologies, dict):
            continue

        for methodology, worlds in methodologies.items():

            if not isinstance(worlds, dict):
                continue

            for world, world_data in worlds.items():

                if not isinstance(world_data, dict):
                    continue

                def recursive_extract(node, path=()):
                    """Recursively search for valid run records."""

                    if not isinstance(node, dict):
                        return

                    # A valid run must contain both required fields.
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

                        # Preserve the complete trial hierarchy.
                        # This prevents identically named trials belonging
                        # to different models from being confused.
                        trial_parts = [
                            part for part in path
                            if str(part).startswith("trial_")
                        ]

                        trial_id = (
                            "/".join(path[:path.index(trial_parts[-1]) + 1])
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

                    # Continue searching through nested dictionaries.
                    for key, value in node.items():

                        if isinstance(value, dict):
                            recursive_extract(
                                value,
                                path + (key,)
                            )

                recursive_extract(world_data)

    df = pd.DataFrame(records)

# The final dataFrame will be a table where
# each row represents one experimental run.
#
#   experiment_type     methodology     world       model       trial           run         sensor_sequence       n_sensors  success
#   deterministic       HITL            world_1     GPT-6       trial_1         run_1           [1, 2]                 2          1
#   deterministic       HITL            world_1     GPT-6       trial_1         run_2           [1, 2]                 2          0
#   deterministic       HITL            world_1     GPT-6       trial_2         run_1           [1, 2]                 2          1
#   deterministic       AUTO            world_1     GPT-6       trial_1         run_1           [1, 2]                 2          1
#   uncertain           HITL            world_2     GPT-6       trial_1         run_1           [1, 2, 3]              3          1

    if not df.empty:
        df["n_sensors"] = df["n_sensors"].astype(int)
        df["success"] = df["success"].astype(int)

    return df


# ============================================================
# WORLD DISCOVERY
# ============================================================

def discover_worlds(data):
    """
    Discover worlds across all experiment types and methodologies.

    This function does not depend on whether a world contains
    valid runs or whether every methodology has data for it.
    """

    worlds = set()

    if not isinstance(data, dict):
        return []

    for experiment_type in EXPERIMENT_TYPES:

        methodologies = data.get(experiment_type, {})

        if not isinstance(methodologies, dict):
            continue

        for methodology_data in methodologies.values():

            if not isinstance(methodology_data, dict):
                continue

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
        "ILP": "#9467bd"
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
    experiments_df["methodology"].unique(),
    key=natural_key
)

selected_methods = st.sidebar.multiselect(
    "Methodologies",
    options=available_methods,
    default=available_methods
)

available_sensors = sorted(
    experiments_df["n_sensors"].unique()
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

        st.download_button(
            "Download plot (HTML)",
            data=fig.to_html(include_plotlyjs=True),
            file_name=f"success_rate_{key_suffix}.html",
            mime="text/html",
            key=f"download_plot_{key_suffix}"
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

st.header("Success Rate Analysis")

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

    render_world_dashboard(
        summary=overview_data,
        trial_stats=overview_trials,
        world="Overview",
        selected_methods=selected_methods,
        selected_sensors=selected_sensors,
        key_suffix="overview"
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
            key_suffix=world
        )