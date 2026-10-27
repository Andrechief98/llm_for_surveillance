
import json
import math
import re
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import streamlit as st


import plotly.graph_objects as go
from plotly.subplots import make_subplots


# CONFIGURATION
# Resolve the directory containing this Python script.
# All input and output paths are defined relative to this directory.
BASE_DIR = Path(__file__).resolve().parent
JSON_PATH = BASE_DIR / "evaluations.json"

FIGURE_SIZE_PER_SUBPLOT = (4.5, 3.5)
DPI = 300

# Statistical configuration.
# VARIANCE_DDOF=1 computes the sample standard deviation (ddof=1).
VARIANCE_DDOF = 1



# HEATMAP CONFIGURATION
# Maps each macro-category used in the heatmap to the original error subcategories
# that belong to it.
#
# Each observed subcategory in the "evaluation.json" can contribute to a macro-category
# only if its name matches one of the entries defined here.

ERROR_CATEGORIES = {
    "Spatial_and_Environment_Understanding": [
        "wrong intruder movement hypothesis",
        "wrong spatial information for area connections",
        "wrong spatial information for sensors coverage",
    ],
    "Resource_Selection_and_Task_Assignment": [
        "wrong area proposed",
        "wrong robot camera inspections assignment",
        "wrong robot deployed",
        "wrong sensor considered",
        "unproposed area to inspect",
        "unproposed camera usage",
        "unoptimal robot deployment",
        "useless proposed camera inspection",
    ],
    "Information_Retrieval_and_Context_Selection": [
        "called get_areas_info with missing areas",
        "called get_areas_info with wrong area",
        "get_areas_info with useless areas",
        "get_sensors_info with useless sensors",
    ],
    "Distance_Computation_and_Decision_Integration": [
        "compute_distance for robot assignment not used",
        "computed_distance_to_area_covered_by_camera",
        "computed_distance_to_wrong_area",
        "compute_distance used but not included for optimal robot deployment",
    ],
    "Function_Calling_and_Parameter_Validity": [
        "called_again_the_same_function",
        "Synthatically wrong parameters generated",
    ],
    "Operational_Constraint_Violations": [
        "Unapproved action execution",
    ],
}

# Human-readable labels used in heatmap axes and dashboard controls.
CATEGORY_LABELS = {
    "Spatial_and_Environment_Understanding":
        "Spatial understanding",
    "Resource_Selection_and_Task_Assignment":
        "Resource selection / assignment",
    "Information_Retrieval_and_Context_Selection":
        "Information retrieval",
    "Distance_Computation_and_Decision_Integration":
        "Distance reasoning",
    "Function_Calling_and_Parameter_Validity":
        "Function calling",
    "Operational_Constraint_Violations":
        "Operational constraints",
}

# Short display labels for retrieval modes.
# The original mode identifiers are retained in the underlying data.
MODE_LABELS = {
    "rag_with_functions": "F-RAG",
    "rag_with_semantics": "S-RAG",
    "image": "Image",
    "image_and_rag_with_functions": "Image-F-RAG",
    "image_and_rag_with_semantics": "Image-S-RAG",

}

# Defines the preferred ordering of retrieval modes in the heatmaps.
# Modes not listed here can be appended in natural sort order.
MODE_ORDER = list(MODE_LABELS)


# HELPER FUNCTIONS

def natural_key(value):
    """
    Build a sorting key that compares numeric portions as integers.

    For example, 'model2' is sorted before 'model10'.

    Args:
        value: Any value that can be converted to a string.

    Returns:
        A list of text and integer components suitable for sorting.
    """
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", str(value))
    ]




def get_assistant_messages(run_data):
    """
    Extract the number of assistant messages from a run record.

    Invalid or missing values are represented by NaN so that
    pandas can exclude them from calculations requiring a valid
    message count.

    Args:
        run_data: Dictionary containing the run metadata.

    Returns:
        A float containing the assistant message count, or NaN.
    """
    value = run_data.get("message_counts", {}).get("assistant")

    if value is None:
        return np.nan

    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


# JSON LOADING

@st.cache_data
def load_runs(json_path, file_mtime):
    """
    Load the nested evaluation JSON and flatten it into one row per run.

    The JSON hierarchy is expected to follow this structure:

        mode -> world -> model -> trial -> run

    Error counts are stored in dictionaries so that individual
    subcategories can be selected later without reloading the JSON.

    Args:
        json_path: Path to the input JSON file.
        file_mtime: File modification timestamp used as a cache key.
                    Changing this value invalidates the cached result.

    Returns:
        A pandas DataFrame with one row per run, including metadata,
        sensor counts, assistant message counts, and error dictionaries.

    Raises:
        ValueError: If no valid run records are found.
        json.JSONDecodeError: If the input file contains invalid JSON.
        OSError: If the file cannot be opened.
    """
    with open(json_path, "r", encoding="utf-8") as file:
        data = json.load(file)

    records = []

    # Traverse the nested JSON hierarchy and ignore malformed nodes.
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

                        raw_errors = run_data.get("error_counts", {})
                        if not isinstance(raw_errors, dict):
                            raw_errors = {}

                        category_totals = {}
                        subcategory_counts = {}

                        # Extract category totals and individual
                        # subcategory counts from the run record.
                        for category, subcategory_details in raw_errors.items():
                            if not isinstance(subcategory_details, dict):
                                continue

                            total = subcategory_details.get("total", 0)

                            if isinstance(total, (int, float)):
                                category_totals[category] = float(total)

                            subcategories = subcategory_details.get("subcategories", {})

                            if isinstance(subcategories, dict):
                                for subcategory, value in subcategories.items():
                                    if isinstance(value, (int, float)):
                                        # The combined key preserves both
                                        # the original category and subcategory.
                                        key = f"{category} :: {subcategory}"
                                        subcategory_counts[key] = float(value)

                        # Store all information required by downstream
                        # calculations and interactive dashboard filters.
                        records.append({
                            "mode": mode,
                            "world": world,
                            "model": model,
                            "trial": trial,
                            "run": run_id,
                            "n_sensors": len(
                                run_data.get("sensor_sequence", [])
                            ),
                            "assistant_messages": get_assistant_messages(
                                run_data
                            ),
                            "error_counts": category_totals,
                            "subcategory_counts": subcategory_counts,
                            "total_errors_json": run_data.get("total_errors"),
                        })

    df = pd.DataFrame(records)

    if df.empty:
        raise ValueError("No valid run records were found in the JSON.")

    return df


# ERROR-RATE CALCULATION

def calculate_metrics(df, selected_errors):
    """
    Calculate the selected error count and error rate for every run.

    The selected errors may refer to individual subcategories or
    complete categories. Missing error counts are treated as zero.

    Error rate is defined as:

        selected error count / number of assistant messages

    Runs with missing or non-positive message counts receive a NaN
    error rate because the denominator is not valid.

    Args:
        df: DataFrame containing run metadata and error dictionaries.
        selected_errors: List of category or subcategory keys selected by the user.

    Returns:
        A copy of the input DataFrame with two additional columns:
        'selected_errors' and 'error_rate'.
    """

    df = df.copy()

    if not selected_errors:
        df["selected_errors"] = np.nan
        df["error_rate"] = np.nan
        return df

    def sum_selected_errors(row):
        """Sum the error counts selected for a single run."""

        total = 0.0

        for error in selected_errors:
            if " :: " in error:
                # Subcategory selection: use its fully qualified key.
                total += row["subcategory_counts"].get(error, 0.0)
            else:
                # Category selection: use the category total.
                total += row["error_counts"].get(error, 0.0)

        return total
    
    # Calculate the selected error count independently for each run.
    df["selected_errors"] = df.apply(sum_selected_errors, axis=1)

    # Only positive, non-missing message counts are valid denominators.
    valid_denominator = (
        df["assistant_messages"].notna()
        & (df["assistant_messages"] > 0)
    )

    df["error_rate"] = np.nan

    df.loc[valid_denominator, "error_rate"] = (
        df.loc[valid_denominator, "selected_errors"]
        / df.loc[valid_denominator, "assistant_messages"]
    )

    return df


# SUMMARY STATISTICS
def compute_statistics(df):
    """
    Aggregate valid run-level error rates by mode, model, and sensor count.

    For each group, calculate:
        - mean_error_rate: arithmetic mean of the run-level rates;
        - std_deviation: sample standard deviation;
        - n_observations: number of valid runs in the group.

    Runs without active sensors or a valid error rate are excluded.

    Args:
        df: DataFrame containing the calculated error_rate column.

    Returns:
        A summary DataFrame with one row per mode/model/sensor-count
        combination. An empty summary is returned if no valid runs exist.
    """

    valid = df[
        (df["n_sensors"] > 0)
        & df["error_rate"].notna()
    ]

    if valid.empty:
        return pd.DataFrame(columns=[
            "mode", "model", "n_sensors",
            "mean_error_rate", "std_deviation",
            "n_observations",
        ])

    return (
        valid.groupby(["mode", "model", "n_sensors"])["error_rate"]
        .agg(
            mean_error_rate="mean",
            std_deviation=lambda x: x.std(ddof=VARIANCE_DDOF),
            n_observations="count",
        )
        .reset_index()
    )




# HEATMAP DATA PREPARATION

def build_heatmap_data(data):
    """
    Convert original subcategory counts into macro-category counts.

    Each input row represents one evaluation run. The function preserves
    the run metadata and adds one numeric column for each macro-category
    defined in ERROR_CATEGORIES.

    Matching is based on the subcategory name, regardless of its original
    JSON category. If a matching name occurs under multiple original
    categories, all corresponding counts are added.

    Args:
        data: Filtered run-level DataFrame produced by load_runs().

    Returns:
        A DataFrame with one row per run and one column per macro-category.
        The original DataFrame is not modified.

    Note:
        Subcategories not listed in ERROR_CATEGORIES are not included
        in the macro-category counts.
    """
    records = []

    for _, row in data.iterrows():
        subcategory_counts = row["subcategory_counts"]

        # Preserve identifiers required for grouping and filtering.
        record = {
            "mode": row["mode"],
            "world": row["world"],
            "model": row["model"],
            "run": row["run"],
            "n_sensors": row["n_sensors"],
        }

        # Aggregate the original subcategory counts into macro-categories.
        for category, expected_errors in ERROR_CATEGORIES.items():
            record[category] = sum(
                count
                for key, count in subcategory_counts.items()
                if key.split(" :: ", 1)[-1] in expected_errors
            )

        records.append(record)

    return pd.DataFrame(records)


# HEATMAP METRICS

def calculate_heatmap_metrics(run_df):
    """
    Calculate two complementary error metrics for each mode/model group.

    Metric 1 — Error prevalence:
        Percentage of runs with at least one error in a macro-category.

        A run contributes either 0% or 1 to the category indicator,
        regardless of how many errors occur in that category.

    Metric 2 — Error distribution:
        Percentage of all mapped error occurrences belonging to each
        macro-category within the same mode/model group.

        Category percentages sum to 100% when the group contains at
        least one mapped error. Groups with zero mapped errors produce
        undefined percentages (NaN).

    Args:
        run_df: Run-level DataFrame returned by build_heatmap_data().

    Returns:
        A tuple (prevalence, distribution). Both DataFrames are indexed
        by mode and model, with macro-categories as columns.
    """
    categories = list(ERROR_CATEGORIES)

    if run_df.empty:
        empty = pd.DataFrame()
        return empty, empty

    # Convert each category count to a binary indicator:
    # 1 means the run contains at least one error; 0 means none.
    binary_df = run_df.copy()

    for category in categories:
        binary_df[category] = (
            binary_df[category] > 0
        ).astype(float)

    # Average the binary indicators to obtain the fraction of affected
    # runs, then multiply by 100 to express the result as a percentage.
    prevalence = (
        binary_df.groupby(["mode", "model"])[categories]
        .mean()
        .mul(100)
    )

    # Sum the actual error occurrences in each category.
    counts = (
        run_df.groupby(["mode", "model"])[categories]
        .sum()
    )

    # Normalize each group by its total number of mapped errors.
    # Replacing zero totals with NaN avoids division by zero.
    totals = counts.sum(axis=1).replace(0, np.nan)

    distribution = counts.div(totals, axis=0).mul(100)

    return prevalence, distribution



# INTERACTIVE LINE CHART

def create_figure(summary, selected_modes, selected_models):
    """
    Create a grid of line charts showing mean error rates.

    Each subplot represents one retrieval mode. Each line represents
    a model, with the number of active sensors on the x-axis and the
    mean error rate on the y-axis. Error bars show the sample standard
    deviation across valid runs.

    Args:
        summary: Aggregated DataFrame returned by compute_statistics().
        selected_modes: Retrieval modes to include.
        selected_models: Models to include.

    Returns:
        A Plotly Figure, or None if there is no data to display.
    """
    modes = sorted(selected_modes, key=natural_key)
    models = sorted(selected_models, key=natural_key)

    if not modes or not models or summary.empty:
        return None

    # Arrange up to three mode-specific plots in each row.
    ncols = min(3, len(modes))
    nrows = math.ceil(len(modes) / ncols)

    fig = make_subplots(
        rows=nrows,
        cols=ncols,
        subplot_titles=[mode.replace("_", " ") for mode in modes],
        horizontal_spacing=0.10,
        vertical_spacing=0.16,
    )

    # Assign a consistent color to each model across all subplots.
    palette = [
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
        "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
        "#bcbd22", "#17becf",
    ]

    colors = {
        model: palette[i % len(palette)]
        for i, model in enumerate(models)
    }

    for i, mode in enumerate(modes):
        row = i // ncols + 1
        col = i % ncols + 1

        mode_data = summary[summary["mode"] == mode]

        for model in models:
            model_data = (
                mode_data[mode_data["model"] == model]
                .sort_values("n_sensors")
            )

            # Do not create empty traces for unavailable combinations.
            if model_data.empty:
                continue

            # A missing standard deviation (for example, a single
            # observation) is displayed with a zero-length error bar.
            yerr = model_data["std_deviation"].fillna(0)

            fig.add_trace(
                go.Scatter(
                    x=model_data["n_sensors"],
                    y=model_data["mean_error_rate"],
                    error_y=dict(
                        type="data",
                        array=yerr,
                        visible=True,
                        thickness=1.2,
                        width=4,
                    ),
                    mode="lines+markers",
                    name=model,
                    legendgroup=model,
                    showlegend=(i == 0),
                    line=dict(color=colors[model], width=2),
                    marker=dict(size=6),
                    hovertemplate=(
                        "Model: " + model +
                        "<br>Sensors: %{x}" +
                        "<br>Mean error rate: %{y:.4f}" +
                        "<extra></extra>"
                    ),
                ),
                row=row,
                col=col,
            )

        # Configure the axes independently for each subplot.
        fig.update_xaxes(
            title_text="Active sensors",
            dtick=1,
            row=row,
            col=col,
        )

        fig.update_yaxes(
            title_text="Mean error rate",
            showgrid=True,
            gridcolor="rgba(128,128,128,0.2)",
            row=row,
            col=col,
        )

    # Configure the overall figure layout and shared legend.
    fig.update_layout(
        title="Spatial Reasoning",
        height=320 * nrows,
        autosize=True,
        margin=dict(l=50, r=30, t=75, b=45),
        template="plotly_white",
        hovermode="closest",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="left",
            x=0,
        ),
    )

    return fig


# INTERACTIVE HEATMAP

def create_heatmap(
    metric_df,
    selected_modes,
    selected_models,
    title,
    colorbar_label,
    selected_categories=None,
):
    """
    Create a heatmap with one subplot per selected model.

    Rows represent retrieval modes, columns represent macro error
    categories, and cell colors represent percentages from 0 to 100.

    Args:
        metric_df: Prevalence or distribution DataFrame indexed by
                   (mode, model).
        selected_modes: Retrieval modes to display.
        selected_models: Models to display.
        title: Figure title.
        colorbar_label: Description of the percentage shown by the scale.
        selected_categories: Macro-categories to display. If None,
                             all configured categories are included.

    Returns:
        A Plotly Figure, or None if no modes, models, or data are available.
    """
    categories = (
        selected_categories
        if selected_categories is not None
        else list(ERROR_CATEGORIES)
    )

    models = sorted(selected_models, key=natural_key)

    # Use the preferred order for known modes, followed by any additional
    # selected modes in natural sort order.
    modes = [m for m in MODE_ORDER if m in selected_modes]
    modes += [
        m for m in sorted(selected_modes, key=natural_key)
        if m not in modes
    ]

    if metric_df.empty or not models or not modes or not categories:
        return None

    fig = make_subplots(
        rows=1,
        cols=len(models),
        subplot_titles=models,
        horizontal_spacing=0.08,
    )

    # Add one heatmap panel per model, keeping the color scale consistent.
    for i, model in enumerate(models, start=1):
        try:
            model_data = metric_df.xs(
                model,
                level="model",
                drop_level=True,
            )
        except KeyError:
            # Preserve the panel layout when a model has no valid data.
            model_data = pd.DataFrame(
                columns=categories,
                index=pd.Index([], name="mode"),
            )

        # Ensure all requested modes and categories appear in the same order.
        model_data = model_data.reindex(
            index=modes,
            columns=categories,
        )

        x_labels = [
            CATEGORY_LABELS.get(category, category)
            for category in categories
        ]

        y_labels = [
            MODE_LABELS.get(mode, mode.replace("_", " "))
            for mode in modes
        ]

        z = model_data.to_numpy(dtype=float)

        # Display the numeric percentage in each cell. NaN values remain
        # blank, allowing unavailable combinations to be distinguished
        # from genuine zero-percent values.
        fig.add_trace(
            go.Heatmap(
                z=z,
                x=x_labels,
                y=y_labels,
                zmin=0,
                zmax=100,
                colorscale="YlOrRd",
                colorbar=(
                    dict(title=colorbar_label)
                    if i == len(models)
                    else None
                ),
                showscale=(i == len(models)),
                text=[
                    [
                        f"{value:.1f}" if pd.notna(value) else ""
                        for value in row
                    ]
                    for row in z
                ],
                texttemplate="%{text}",
                xgap=2,
                ygap=2,
                hovertemplate=(
                    "Mode: %{y}<br>"
                    "Category: %{x}<br>"
                    "Value: %{z:.2f}%"
                    "<extra></extra>"
                ),
            ),
            row=1,
            col=i,
        )

        # Improve readability of the category labels and mode ordering.
        fig.update_xaxes(tickangle=-35, row=1, col=i)
        fig.update_yaxes(autorange="reversed", row=1, col=i)

    fig.update_layout(
        title=title,
        template="plotly_white",
        height=max(350, 90 + 65 * len(modes)),
        autosize=True,
        margin=dict(l=110, r=40, t=90, b=150),
    )

    return fig


# STREAMLIT INTERFACE

st.set_page_config(
    page_title="Spatial Reasoning Analysis",
    page_icon="📊",
    layout="wide",
)

st.markdown(
    """
    <style>
        [data-testid="stSidebar"] {
            min-width: 420px !important;
            max-width: 420px !important;
        }

        [data-testid="stSidebar"] > div:first-child {
            width: 420px !important;
        }

        [data-testid="stSidebar"] label,
        [data-testid="stSidebar"] p {
            font-size: 14px !important;
            line-height: 1.5 !important;
        }

        [data-testid="stSidebar"] [data-testid="stExpander"] {
            width: 100% !important;
        }

        [data-testid="stSidebar"] section {
            padding-left: 1rem;
            padding-right: 1rem;
        }
    </style>
    """,
    unsafe_allow_html=True,
)

st.title("Spatial Reasoning — Interactive Analysis")
st.caption(
    "Choose error categories"
)

if not JSON_PATH.exists():
    st.error(f"JSON file not found: {JSON_PATH}")
    st.stop()

try:
    df = load_runs(str(JSON_PATH), JSON_PATH.stat().st_mtime)
except (json.JSONDecodeError, OSError, ValueError) as exc:
    st.error(f"Impossible to load JSON file: {exc}")
    st.stop()


# SIDEBAR: CATEGORIES AND ERROR FILTERS

st.sidebar.header("Error selection")

ERROR_GROUPS = {
    "Reasoning Errors": "Reasoning_errors",
    "Hallucinations": "Hallucinations",
    "Wrong Function Calls": "Wrong_function_calls",
}

# Retrieving JSON categories.
available_subcategories = {}

for counts in df["subcategory_counts"]:
    for key in counts:
        if " :: " not in key:
            continue

        category, subcategory = key.split(" :: ", 1)

        available_subcategories.setdefault(category, set()).add(
            subcategory
        )

selected_errors = []

for display_name, json_key in ERROR_GROUPS.items():
    subcategories = sorted(
        available_subcategories.get(json_key, set()),
        key=natural_key,
    )

    if not subcategories:
        continue

    with st.sidebar.expander(display_name, expanded=True):
        col_all, col_none = st.columns(2)

        if col_all.button("Select all", key=f"all_{json_key}"):
            for subcategory in subcategories:
                st.session_state[
                    f"error_{json_key}_{subcategory}"
                ] = True

        if col_none.button("Clear", key=f"none_{json_key}"):
            for subcategory in subcategories:
                st.session_state[
                    f"error_{json_key}_{subcategory}"
                ] = False

        for subcategory in subcategories:
            widget_key = f"error_{json_key}_{subcategory}"

            if widget_key not in st.session_state:
                st.session_state[widget_key] = True

            if st.checkbox(subcategory, key=widget_key):
                selected_errors.append(
                    f"{json_key} :: {subcategory}"
                )

st.sidebar.divider()
st.sidebar.header("Data filters")

all_worlds = sorted(df["world"].dropna().unique(), key=natural_key)

selected_worlds = st.sidebar.multiselect(
    "Worlds",
    options=all_worlds,
    default=all_worlds,
)

all_modes = sorted(df["mode"].unique(), key=natural_key)
selected_modes = st.sidebar.multiselect(
    "Modes",
    options=all_modes,
    default=all_modes,
)

all_models = sorted(df["model"].unique(), key=natural_key)
selected_models = st.sidebar.multiselect(
    "Models",
    options=all_models,
    default=all_models,
)

all_sensor_counts = sorted(
    int(n) for n in df["n_sensors"].unique() if n > 0
)
selected_sensors = st.sidebar.multiselect(
    "Number of active sensors",
    options=all_sensor_counts,
    default=all_sensor_counts,
)


# ANALYSIS AND VISUALIZATION 

def render_analysis(
    data,
    selected_errors,
    selected_modes,
    selected_models,
    title_suffix="",
    key_suffix="overview",
):
    """
    Render the complete analysis for a filtered set of evaluation runs.

    The function applies the selected retrieval modes, models, and
    sensor counts, calculates the selected error rate, and displays
    summary statistics, interactive plots, comparison tables, and
    individual run details.

    It also generates two macro-category heatmaps:
        1. Error prevalence across runs.
        2. Distribution of mapped error occurrences.

    The heatmaps use all mapped error subcategories, independently of
    the subcategories selected for the error-rate calculation.

    Args:
        data: Run-level DataFrame, optionally restricted to one world
              or a group of selected worlds.
        selected_errors: Error categories/subcategories used for the
                         error-rate calculation.
        selected_modes: Retrieval modes included in the analysis.
        selected_models: Models included in the analysis.
        title_suffix: Optional text appended to section titles.
        key_suffix: Unique identifier used to prevent Streamlit widget
                    and download-button key collisions.

    Returns:
        None. Results are rendered directly in the Streamlit interface.
    """

    filtered = data[
        data["mode"].isin(selected_modes)
        & data["model"].isin(selected_models)
        & data["n_sensors"].isin(selected_sensors)
    ].copy()

    filtered = calculate_metrics(filtered, selected_errors)

    valid = filtered[
        (filtered["n_sensors"] > 0)
        & filtered["error_rate"].notna()
    ].copy()

    if not selected_errors:
        st.warning("Select at least one error subcategory.")
        return

    if valid.empty:
        st.warning(
            "No valid runs available for the current selection."
        )
        return

    # Statistics
    summary = compute_statistics(valid)

    col1, col2, col3 = st.columns(3)

    col1.metric("Valid runs", len(valid))
    col2.metric("Error categories", len(selected_errors))
    col3.metric("Models", valid["model"].nunique())

    st.subheader(f"Error rate by mode and model{title_suffix}")

    fig = create_figure(
        summary,
        selected_modes,
        selected_models,
    )

    if fig is not None:
        st.plotly_chart(
            fig,
            use_container_width=True,
            config={
                "scrollZoom": True,
                "displaylogo": False,
                "responsive": True,
                "modeBarButtonsToAdd": [
                    "drawrect",
                    "eraseshape",
                ],
            },
            key=f"plot_{key_suffix}",
        )

        st.download_button(
            "Download plot (HTML)",
            data=fig.to_html(include_plotlyjs=True),
            file_name=f"spatial_reasoning_{key_suffix}.html",
            mime="text/html",
            key=f"download_plot_html_{key_suffix}",
        )

        st.download_button(
            "Download plot (JPEG)",
                data=fig.to_image(
                format="jpeg",
                width=1800,
                height=900,
                scale=2,
            ),
            file_name=f"spatial_reasoning_{key_suffix}.jpg",
            mime="image/jpeg",
            key=f"download_plot_jpeg_{key_suffix}",
        )


    
    # Table stats
    
    st.subheader("Summary statistics")

    display_summary = summary.copy()

    display_summary["mean_error_rate"] = (
        display_summary["mean_error_rate"].round(4)
    )
    display_summary["std_deviation"] = (
        display_summary["std_deviation"].round(4)
    )

    st.dataframe(display_summary, use_container_width=True)

    st.download_button(
        "Download statistics (CSV)",
        data=summary.to_csv(index=False).encode("utf-8"),
        file_name=f"error_rate_statistics_{key_suffix}.csv",
        mime="text/csv",
        key=f"download_stats_{key_suffix}",
    )
    

    # Table pivot

    st.subheader("Comparison table")

    pivot_data = summary.copy()

    pivot_data["mean ± std"] = pivot_data.apply(
        lambda row: (
            f"{row['mean_error_rate']:.4f} ± "
            f"{row['std_deviation']:.4f}"
            if pd.notna(row["std_deviation"])
            else f"{row['mean_error_rate']:.4f} ± N/A"
        ),
        axis=1,
    )

    if not pivot_data.empty:
        pivot = pivot_data.pivot(
            index=["n_sensors", "model"],
            columns="mode",
            values="mean ± std",
        ).sort_index()

        st.dataframe(pivot, use_container_width=True)

        st.download_button(
            "Download comparison table (CSV)",
            data=pivot.to_csv().encode("utf-8"),
            file_name=f"error_rate_summary_table_{key_suffix}.csv",
            mime="text/csv",
            key=f"download_pivot_{key_suffix}",
        )

    
    # Details single run
    
    with st.expander("Show individual runs"):
        columns = [
            "mode", "world", "model", "trial", "run",
            "n_sensors", "selected_errors",
            "assistant_messages", "error_rate",
        ]

        st.dataframe(
            valid[columns].sort_values(
                ["mode", "model", "n_sensors"],
                key=lambda col: (
                    col.map(lambda x: str(x).lower())
                    if col.name in ["mode", "model"]
                    else col
                ),
            ),
            use_container_width=True,
        )

        st.download_button(
            "Download individual runs (CSV)",
            data=valid[columns].to_csv(index=False).encode("utf-8"),
            file_name=f"individual_run_error_rates_{key_suffix}.csv",
            mime="text/csv",
            key=f"download_runs_{key_suffix}",
        )


    # HEATMAPS

    st.divider()
    st.subheader(f"Error heatmaps{title_suffix}")

    st.caption(
        "Heatmaps show error macrocategories "
        "and use current filters from dashboard."
    )

    heatmap_categories = list(ERROR_CATEGORIES)

    selected_heatmap_categories = st.multiselect(
        "Macro error categories",
        options=heatmap_categories,
        default=heatmap_categories,
        format_func=lambda category: CATEGORY_LABELS.get(
            category, category
        ),
        key=f"heatmap_categories_{key_suffix}",
    )

    # I conteggi vengono calcolati sulle run filtrate,
    # senza dipendere dalle sottocategorie selezionate
    # per il grafico del tasso di errore.
    heatmap_data = build_heatmap_data(filtered)

    prevalence, distribution = calculate_heatmap_metrics(
        heatmap_data
    )

    if not selected_heatmap_categories:
        st.info("Select at least one macro error category.")
    elif heatmap_data.empty:
        st.warning("No data available for the heatmaps.")
    else:
        # Selecting categories to visualize.
        prevalence = prevalence[
            selected_heatmap_categories
        ]
        distribution = distribution[
            selected_heatmap_categories
        ]

        heatmap_tab1, heatmap_tab2 = st.tabs([
            "Error prevalence (%)",
            "Error distribution (%)",
        ])

        with heatmap_tab1:
            st.markdown(
                "**Percentage of runs with at least one error "
                "for category.**"
            )

            fig_prevalence = create_heatmap(
                metric_df=prevalence,
                selected_modes=selected_modes,
                selected_models=selected_models,
                title="Error prevalence by mode and model",
                colorbar_label="Runs affected (%)",
                selected_categories=selected_heatmap_categories,
            )

            if fig_prevalence is not None:
                st.plotly_chart(
                    fig_prevalence,
                    use_container_width=True,
                    config={"displaylogo": False},
                    key=f"heatmap_prevalence_{key_suffix}",
                )

                st.download_button(
                    "Download prevalence heatmap (HTML)",
                    data=fig_prevalence.to_html(
                        include_plotlyjs=True
                    ),
                    file_name=(
                        f"heatmap_prevalence_{key_suffix}.html"
                    ),
                    mime="text/html",
                    key=f"download_prevalence_{key_suffix}",
                )

                st.download_button(
                    "Download prevalence heatmap (JPEG)",
                    data=fig_prevalence.to_image(
                        format="jpeg",
                        width=1800,
                        height=900,
                        scale=2,
                    ),
                    file_name=f"heatmap_prevalence_{key_suffix}.jpg",
                    mime="image/jpeg",
                    key=f"download_prevalence_jpeg_{key_suffix}",
                )

                st.download_button(
                    "Download prevalence data (CSV)",
                    data=prevalence.to_csv().encode("utf-8"),
                    file_name=(
                        f"heatmap_prevalence_data_{key_suffix}.csv"
                    ),
                    mime="text/csv",
                    key=f"download_prevalence_csv_{key_suffix}",
                )

        with heatmap_tab2:
            st.markdown(
                "**Percentage of error total occurances "
                "for each category.**"
            )

            fig_distribution = create_heatmap(
                metric_df=distribution,
                selected_modes=selected_modes,
                selected_models=selected_models,
                title="Error distribution by mode and model",
                colorbar_label="Error occurrences (%)",
                selected_categories=selected_heatmap_categories,
            )

            if fig_distribution is not None:
                st.plotly_chart(
                    fig_distribution,
                    use_container_width=True,
                    config={"displaylogo": False},
                    key=f"heatmap_distribution_{key_suffix}",
                )

                st.download_button(
                    "Download distribution heatmap (HTML)",
                    data=fig_distribution.to_html(
                        include_plotlyjs=True
                    ),
                    file_name=(
                        f"heatmap_distribution_{key_suffix}.html"
                    ),
                    mime="text/html",
                    key=f"download_distribution_html_{key_suffix}",
                )

                st.download_button(
                    "Download distribution heatmap (JPEG)",
                        data=fig_distribution.to_image(
                        format="jpeg",
                        width=1800,
                        height=900,
                        scale=2,
                    ),
                    file_name=f"heatmap_distribution_{key_suffix}.jpg",
                    mime="image/jpeg",
                    key=f"download_distribution_jpeg_{key_suffix}",
                )

                st.download_button(
                    "Download distribution data (CSV)",
                    data=distribution.to_csv().encode("utf-8"),
                    file_name=(
                        f"heatmap_distribution_data_{key_suffix}.csv"
                    ),
                    mime="text/csv",
                    key=f"download_distribution_csv_{key_suffix}",
                )


# SHARED FILTERS

filtered_worlds = df[
    df["world"].isin(selected_worlds)
].copy()

# TAB: OVERVIEW + UN TAB FOR EACH WORLD

tab_names = ["Overview"] + [
    str(world) for world in selected_worlds
]

tabs = st.tabs(tab_names)

# Overview: all selected worlds
with tabs[0]:
    render_analysis(
        data=filtered_worlds,
        selected_errors=selected_errors,
        selected_modes=selected_modes,
        selected_models=selected_models,
        title_suffix=" — All selected worlds",
        key_suffix="overview",
    )

# One tab for each individual world
for i, world in enumerate(selected_worlds, start=1):
    with tabs[i]:
        world_data = filtered_worlds[
            filtered_worlds["world"] == world
        ].copy()

        render_analysis(
            data=world_data,
            selected_errors=selected_errors,
            selected_modes=selected_modes,
            selected_models=selected_models,
            title_suffix=f" — {world}",
            key_suffix=f"world_{i}",
        )
