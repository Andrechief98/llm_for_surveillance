
import json
from pathlib import Path

import numpy as np
import pandas as pd
import streamlit as st
import plotly.graph_objects as go

from plotly.subplots import make_subplots


# ============================================================
# 1. CONFIGURATION
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

METRICS_INPUT_FILE = BASE_DIR / "metrics/run_metrics.json"
JACCARD_INPUT_FILE = BASE_DIR / "metrics/jaccard_similarity.json"
COSINE_INPUT_FILE = BASE_DIR / "metrics/cosine_similarity.json"

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

COLORS = {
    "HITL": "#1f77b4",
    "AUTO": "#ff7f0e",
    "ILP-B30": "#2ca02c",
    "ILP-B60": "#d62728",
    "ILP-B120": "#9467bd"
}

# Plot each method pair.
EXPERIMENT_COLORS = {
    "deterministic": "#1f77b4",  # Blue
    "uncertain": "#ff7f0e"       # Orange
}

EXPERIMENT_LABELS = {
    "deterministic": "Deterministic",
    "uncertain": "Uncertain"
}

# Maximum width of the dashboard plots.
PLOT_WIDTH = 850



# ============================================================
# PDF CONFIGURATION
# ============================================================

PDF_CONFIG = {

    "metrics": {
        "width": 550,
        "height": 300,
        "scale": 2,

        "xaxis_min": None,
        "xaxis_max": None,
        "yaxis_min": 0,
        "yaxis_max": 1.05,

        "figure_border": True,
        "axis_border": True,

        "border_color": "#000000",
        "border_width": 0.7,

        "marker_border_width": 0.8,

        "top_margin": 10,
        "bottom_margin":10,
        "left_margin": 10,
        "right_margin":10,

        "tick_font_size": 14,
        "axis_title_font_size": 18,
        "title_font_size": 22,
        "subplot_title_font_size": 16,
        "legend_font_size": 14
    },

    "hamming": {
        "width": 550,
        "height": 300,
        "scale": 2,

        "xaxis_min": None,
        "xaxis_max": None,
        "yaxis_min": 0,
        "yaxis_max": 1.05,

        "figure_border": True,
        "axis_border": True,
        "top_margin": 10,
        "bottom_margin":10,
        "left_margin": 10,
        "right_margin":10,

        "border_color": "#000000",
        "border_width": 0.7,

        "marker_border_width": 0.8,

        "tick_font_size": 14,
        "axis_title_font_size": 18,
        "title_font_size": 22,
        "subplot_title_font_size": 16,
        "legend_font_size": 14
    },

    "similarity": {

        "width": 1400,
        "height": 800,
        "scale": 2,

        # Subplot dimensions
        "column_widths": [
            0.25,
            0.25,
            0.25,
            0.25
        ],

        "row_heights": [
            0.30,
            0.30,
            0.31
        ],

        "horizontal_spacing": 0.08,
        "vertical_spacing": 0.13,

        "xaxis_min": None,
        "xaxis_max": None,
        "yaxis_min": 0,
        "yaxis_max": 1.05,

        "figure_border": False,
        "subplot_border": True,
        "axis_border": True,

        "border_color": "#000000",
        "border_width": 0.7,

        "subplot_border_color": "#000000",
        "subplot_border_width": 0.7,

        "axis_border_color": "#000000",
        "axis_border_width": 0.7,

        "marker_border_width": 0.8,

        # Margins
        "top_margin": 55,
        "bottom_margin": 100,
        "left_margin": 65,
        "right_margin": 40,

        # Font sizes
        "tick_font_size": 14,
        "axis_title_font_size": 18,
        "title_font_size": 22,
        "subplot_title_font_size": 16,
        "legend_font_size": 14

}

}


def add_subplot_borders(fig, config):

    """
    Add an independent rectangular border to every subplot
    using the corresponding axis domains.
    """

    layout = fig.layout.to_plotly_json()

    subplot_border_color = config["subplot_border_color"]
    subplot_border_width = config["subplot_border_width"]

    # Retrieve X and Y axes.
    x_axes = {
        name: axis
        for name, axis in layout.items()
        if name == "xaxis" or name.startswith("xaxis")
    }

    y_axes = {
        name: axis
        for name, axis in layout.items()
        if name == "yaxis" or name.startswith("yaxis")
    }

    # Process each subplot.
    for x_name, x_axis in x_axes.items():

        suffix = x_name.replace("xaxis", "")

        y_name = f"yaxis{suffix}"

        y_axis = y_axes.get(y_name)

        if y_axis is None:
            continue

        x_domain = x_axis.get("domain")
        y_domain = y_axis.get("domain")

        if x_domain is None or y_domain is None:
            continue

        # Skip invisible subplots.
        if (
            x_axis.get("visible") is False
            or y_axis.get("visible") is False
        ):
            continue

        fig.add_shape(

            type="rect",

            xref="paper",
            yref="paper",

            x0=x_domain[0],
            x1=x_domain[1],

            y0=y_domain[0],
            y1=y_domain[1],

            line=dict(
                color=subplot_border_color,
                width=subplot_border_width
            ),

            fillcolor="rgba(0,0,0,0)",

            layer="above"

        )
# ============================================================
# LATEX TABLE GENERATOR
# ============================================================

def download_latex_table(
    dataframe,
    filename,
    caption,
    label,
    key,
    index=True
):

    latex = dataframe.to_latex(
        index=index,
        escape=False,
        na_rep="-",
        column_format=None,
        caption=caption,
        label=label,
        bold_rows=False,
        longtable=False
    )

    latex = (
        "\\begin{table}[htbp]\n"
        "\\centering\n"
        "\\small\n"
        + latex +
        "\n\\end{table}"
    )

    st.download_button(
        label="Download LaTeX table",
        data=latex,
        file_name=filename,
        mime="text/plain",
        key=key
    )

# ============================================================
# 2. STREAMLIT CONFIGURATION
# ============================================================

st.set_page_config(
    page_title="Experimental Results Dashboard",
    page_icon="📊",
    layout="wide"
)

st.title("Experimental Results Dashboard")

st.markdown(
    """
    Interactive visualization of experimental metrics,
    inspection performance and method similarities.
    """
)

st.divider()


# ============================================================
# 3. LOAD JSON DATA
# ============================================================

@st.cache_data
def load_json_data():

    with open(
        METRICS_INPUT_FILE,
        "r",
        encoding="utf-8"
    ) as file:

        data = json.load(file)

    with open(
        JACCARD_INPUT_FILE,
        "r",
        encoding="utf-8"
    ) as file:

        jaccard_data = json.load(file)

    with open(
        COSINE_INPUT_FILE,
        "r",
        encoding="utf-8"
    ) as file:

        cosine_data = json.load(file)

    return data, jaccard_data, cosine_data


data, jaccard_data, cosine_data = load_json_data()


# ============================================================
# 4. EXTRACT RUN METRICS
# ============================================================

records = []

for method, method_data in data.items():

    for experiment_type, experiment_data in method_data.items():

        for world, world_data in experiment_data.items():

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

                        "success_rate": run_data.get(
                            "success_rate"
                        ),

                        "efficiency": run_data.get(
                            "efficiency",
                            run_data.get("efficiency_human")
                        ),

                        "normalized_hamming_distance": run_data.get(
                            "normalized_hamming_distance"
                        )

                    })


df = pd.DataFrame(records)

if df.empty:

    st.error("No run metrics found.")

    st.stop()


# ============================================================
# 5. EXTRACT JACCARD SIMILARITY
# ============================================================

jaccard_records = []

for experiment_type, worlds in jaccard_data.items():

    for world, trials in worlds.items():

        for trial, sequences in trials.items():

            for sequence, pairwise_results in sequences.items():

                n_sensors = len([
                    sensor
                    for sensor in sequence.split(",")
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

    st.error("No Jaccard similarity data found.")

    st.stop()


# ============================================================
# 6. EXTRACT COSINE SIMILARITY
# ============================================================

cosine_records = []

for experiment_type, worlds in cosine_data.items():

    for world, trials in worlds.items():

        for trial, sequences in trials.items():

            for sequence, similarity_data in sequences.items():

                n_sensors = len([
                    sensor
                    for sensor in sequence.split(",")
                    if sensor.strip()
                ])

                for similarity_type in ["binary", "weighted"]:

                    pairwise_results = similarity_data.get(
                        similarity_type,
                        {}
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

if cosine_df.empty:

    st.error("No Cosine similarity data found.")

    st.stop()




# ============================================================
# 7. SIDEBAR: METHOD SELECTION
# ============================================================

st.sidebar.title("Dashboard Settings")

st.sidebar.subheader("Methods")

# Initialize selected methods.

if "selected_methods" not in st.session_state:

    st.session_state.selected_methods = METHODS_ORDER.copy()


# Select all methods.

if st.sidebar.button(
    "Select all",
    use_container_width=True
):

    st.session_state.selected_methods = METHODS_ORDER.copy()


# Deselect all methods.

if st.sidebar.button(
    "Deselect all",
    use_container_width=True
):

    st.session_state.selected_methods = []


# Individual method selection.

selected_methods = []

for method in METHODS_ORDER:

    selected = st.sidebar.checkbox(
        method,
        value=method in st.session_state.selected_methods,
        key=f"method_{method}"
    )

    if selected:
        selected_methods.append(method)


st.session_state.selected_methods = selected_methods


# Standard deviation option.

st.sidebar.divider()

SHOW_ERROR_BARS = st.sidebar.checkbox(
    "Show standard deviation",
    value=False
)


# Dataset information.

st.sidebar.divider()

st.sidebar.subheader("Dataset information")

st.sidebar.metric(
    "Total runs",
    len(df)
)

st.sidebar.metric(
    "Jaccard comparisons",
    len(jaccard_df)
)

st.sidebar.metric(
    "Cosine comparisons",
    len(cosine_df)
)


# Warn if no methods are selected.

if not selected_methods:

    st.warning(
        "Select at least one method from the sidebar."
    )

    st.stop()


# ============================================================
# 8. FILTER DATA ACCORDING TO SELECTED METHODS
# ============================================================

def filter_run_metrics(dataframe):

    return dataframe[
        dataframe["method"].isin(selected_methods)
    ].copy()


def filter_similarity(dataframe):

    filtered_pairs = []

    for method_pair in dataframe["method_pair"].unique():

        methods_in_pair = method_pair.split("-")

        if all(
            method in selected_methods
            for method in methods_in_pair
        ):

            filtered_pairs.append(method_pair)

    return dataframe[
        dataframe["method_pair"].isin(filtered_pairs)
    ].copy()


# ============================================================
# 9. AGGREGATE RUN METRICS
# ============================================================

def aggregate_run_metrics(dataframe):

    if dataframe.empty:
        return pd.DataFrame()

    aggregated = (

        dataframe

        .groupby(
            [
                "method",
                "experiment_type",
                "n_sensors"
            ],
            dropna=False
        )[METRICS]

        .agg([
            "mean",
            "std",
            "count"
        ])

        .reset_index()

    )

    # Flatten MultiIndex columns.

    aggregated.columns = [

        f"{metric}_{statistic}" if statistic else metric

        for metric, statistic in aggregated.columns

    ]

    # A standard deviation is undefined for a single observation.
    # Keep it as zero for plotting purposes.

    aggregated = aggregated.fillna(0)

    return aggregated


# ============================================================
# 10. AGGREGATE JACCARD SIMILARITY
# ============================================================

def aggregate_jaccard(dataframe):

    if dataframe.empty:
        return pd.DataFrame()

    aggregated = (

        dataframe

        .groupby(
            [
                "method_pair",
                "experiment_type",
                "n_sensors"
            ]
        )["jaccard_similarity"]

        .agg([
            "mean",
            "std",
            "count"
        ])

        .reset_index()

    )

    aggregated = aggregated.rename(
        columns={
            "mean": "jaccard_similarity_mean",
            "std": "jaccard_similarity_std",
            "count": "jaccard_similarity_count"
        }
    )

    return aggregated.fillna(0)


# ============================================================
# 11. AGGREGATE COSINE SIMILARITY
# ============================================================

def aggregate_cosine(dataframe):

    if dataframe.empty:
        return pd.DataFrame()

    aggregated = (

        dataframe

        .groupby(
            [
                "method_pair",
                "experiment_type",
                "n_sensors",
                "similarity_type"
            ]
        )["cosine_similarity"]

        .agg([
            "mean",
            "std",
            "count"
        ])

        .reset_index()

    )

    aggregated = aggregated.rename(
        columns={
            "mean": "cosine_similarity_mean",
            "std": "cosine_similarity_std",
            "count": "cosine_similarity_count"
        }
    )

    return aggregated.fillna(0)


# ============================================================
# 12. AGGREGATE NORMALIZED HAMMING DISTANCE
# ============================================================

def aggregate_hamming(dataframe):

    hitl_data = dataframe[
        dataframe["method"] == "HITL"
    ].dropna(
        subset=["normalized_hamming_distance"]
    )

    if hitl_data.empty:
        return pd.DataFrame()

    aggregated = (

        hitl_data

        .groupby(
            [
                "experiment_type",
                "n_sensors"
            ]
        )["normalized_hamming_distance"]

        .agg([
            "mean",
            "std",
            "count"
        ])

        .reset_index()

        .fillna(0)

    )

    return aggregated


# ============================================================
# 13. PDF EXPORT
# ============================================================
import copy

def download_pdf(
    fig,
    filename,
    key,
    plot_type="metrics"
):

    try:

        # ----------------------------------------------------
        # GET PDF CONFIGURATION
        # ----------------------------------------------------

        config = PDF_CONFIG[plot_type]

        # ----------------------------------------------------
        # CREATE FIGURE COPY
        # ----------------------------------------------------

        pdf_fig = copy.deepcopy(fig)

        # ----------------------------------------------------
        # FIGURE DIMENSIONS
        # ----------------------------------------------------

        pdf_fig.update_layout(

            width=config["width"],

            height=config["height"],

            font=dict(
                family="Arial",
                size=config["tick_font_size"],
                color="black"
            ),

            title_font=dict(
                size=config["title_font_size"],
                color="black"
            ),

            legend=dict(
                font=dict(
                    size=config["legend_font_size"]
                )
            ),

            margin=dict(

                l=config["left_margin"],
                r=config["right_margin"],
                t=config["top_margin"],
                b=config["bottom_margin"]

            )

        )

        # ----------------------------------------------------
        # FIGURE BORDER
        # ----------------------------------------------------

        if config["figure_border"]:

            pdf_fig.add_shape(

                type="rect",

                xref="paper",
                yref="paper",

                x0=0,
                y0=0,
                x1=1,
                y1=1,

                line=dict(

                    color=config["border_color"],
                    width=config["border_width"]

                ),

                fillcolor="rgba(0,0,0,0)",

                layer="above"

            )

        # ----------------------------------------------------
        # INDIVIDUAL SUBPLOT BORDERS
        # ----------------------------------------------------

        if (
            plot_type == "similarity"
            and config["subplot_border"]
        ):

            add_subplot_borders(
                pdf_fig,
                config
            )

        # ----------------------------------------------------
        # AXIS FORMATTING
        # ----------------------------------------------------

        axis_settings = dict(

            showline=config["axis_border"],

            mirror=config["axis_border"],

            linecolor=config.get(
                "axis_border_color",
                config["border_color"]
            ),

            linewidth=config.get(
                "axis_border_width",
                config["border_width"]
            ),

            tickfont=dict(
                size=config["tick_font_size"],
                color="black"
            ),

            title_font=dict(
                size=config["axis_title_font_size"],
                color="black"
            )

        )

        pdf_fig.update_xaxes(**axis_settings)
        pdf_fig.update_yaxes(**axis_settings)

        # ----------------------------------------------------
        # AXIS RANGE
        # ----------------------------------------------------

        pdf_fig.update_xaxes(

            range=[
                config["xaxis_min"],
                config["xaxis_max"]
            ]

        )

        pdf_fig.update_yaxes(

            range=[
                config["yaxis_min"],
                config["yaxis_max"]
            ]

        )

        # ----------------------------------------------------
        # MARKER WHITE BORDER
        # ----------------------------------------------------

        pdf_fig.update_traces(

            marker=dict(

                line=dict(
                    color="white",
                    width=config["marker_border_width"]
                )

            ),

            selector=dict(type="scatter")

        )

        # ----------------------------------------------------
        # SUBPLOT TITLES
        # ----------------------------------------------------

        if plot_type == "similarity":

            for annotation in pdf_fig.layout.annotations:

                annotation.font = dict(

                    size=config["subplot_title_font_size"],
                    color="black"

                )

        # ----------------------------------------------------
        # PDF GENERATION
        # ----------------------------------------------------

        pdf_bytes = pdf_fig.to_image(

            format="pdf",

            width=config["width"],

            height=config["height"],

            scale=config["scale"]

        )

        # ----------------------------------------------------
        # DOWNLOAD BUTTON
        # ----------------------------------------------------

        st.download_button(

            label="Download PDF",

            data=pdf_bytes,

            file_name=filename,

            mime="application/pdf",

            key=key

        )

    except Exception as error:

        st.warning(

            "Unable to export PDF. "

            "Make sure Kaleido is installed. "

            f"Details: {error}"

        )
# ============================================================
# 14. CENTERED PLOT CONTAINER
# ============================================================

def display_centered_plot(fig, key):

    left, center, right = st.columns([1, 3, 1])

    with center:

        st.plotly_chart(
            fig,
            use_container_width=False,
            key=key
        )



# ============================================================
# 15. PLOT RUN METRICS
# ============================================================


def plot_metric(
    aggregated,
    metric,
    ylabel,
    title,
    filename,
    tab_name
):
    if aggregated.empty:
        st.info("No data available.")
        return

    col_det, col_unc = st.columns(2)

    experiment_columns = [
        ("deterministic", col_det),
        ("uncertain", col_unc)
    ]

    for experiment_type, column in experiment_columns:

        experiment_data = aggregated[
            aggregated["experiment_type"] == experiment_type
        ].copy()

        with column:

            st.subheader(experiment_type.capitalize())

            if experiment_data.empty:

                st.info("No data available.")
                continue

            fig = go.Figure()

            for method in METHODS_ORDER:

                if method not in selected_methods:
                    continue

                method_data = experiment_data[
                    experiment_data["method"] == method
                ].sort_values("n_sensors")

                if method_data.empty:
                    continue

                fig.add_trace(
                    go.Scatter(

                        x=method_data["n_sensors"],

                        y=method_data[f"{metric}_mean"],

                        error_y=dict(
                            type="data",
                            array=method_data[f"{metric}_std"],
                            visible=SHOW_ERROR_BARS
                        ),

                        mode="lines+markers",

                        name=method,

                        line=dict(
                            color=COLORS[method],
                            width=2
                        ),

                        marker=dict(size=7)

                    )
                )

            fig.update_layout(

                # title=dict(
                #     text=title,
                #     x=0.5
                # ),

                width=650,

                height=450,

                template="plotly_white",

                xaxis_title=r"$N_s$",

                yaxis_title=ylabel,

                hovermode="x unified",

                legend=dict(
                    orientation="h",
                    yanchor="top",
                    y=-0.25,
                    xanchor="center",
                    x=0.5
                ),

                margin=dict(
                    l=65,
                    r=20,
                    t=70,
                    b=110
                )

            )

            fig.update_xaxes(
                tickmode="array",
                tickvals=sorted(
                    experiment_data["n_sensors"].unique()
                ),
                showgrid=True,
                gridcolor="lightgray"
            )

            fig.update_yaxes(
                range=[0, 1.05],
                showgrid=True,
                gridcolor="lightgray"
            )

            # PLOT

            st.plotly_chart(
                fig,
                use_container_width=True
            )

            # PDF DOWNLOAD

            download_pdf(
                fig=fig,
                filename=(
                    f"{Path(filename).stem}_"
                    f"{tab_name}_"
                    f"{experiment_type}.pdf"
                ),
                key=f"pdf_metric_{tab_name}_{metric}_{experiment_type}",
                plot_type="metrics"
            )
            # ------------------------------------------------
            # TABLE IMMEDIATELY BELOW THE CORRESPONDING PLOT
            # ------------------------------------------------

            st.markdown("**Mean ± Standard Deviation**")

            table = {}

            for n_sensors in sorted(
                experiment_data["n_sensors"].unique()
            ):

                table[n_sensors] = {}

                for method in METHODS_ORDER:

                    if method not in selected_methods:
                        continue

                    row = experiment_data[
                        (experiment_data["n_sensors"] == n_sensors) &
                        (experiment_data["method"] == method)
                    ]

                    if row.empty:

                        table[n_sensors][method] = "-"

                    else:

                        mean = row.iloc[0][f"{metric}_mean"]

                        std = row.iloc[0][f"{metric}_std"]

                        count = row.iloc[0][f"{metric}_count"]

                        table[n_sensors][method] = (
                            f"{mean:.3f} ± {std:.3f}"
                        )

            table_df = pd.DataFrame.from_dict(
                table,
                orient="index"
            )

            table_df.index.name = r"$N_s$"

            table_df = table_df.reindex(
                columns=[
                    method for method in METHODS_ORDER
                    if method in selected_methods
                ]
            )

            st.dataframe(
                table_df,
                use_container_width=True
            )

            download_latex_table(
                dataframe=table_df,
                filename=f"{Path(filename).stem}_{experiment_type}.tex",
                caption=f"{title} - {experiment_type}",
                label=f"tab:{Path(filename).stem}_{experiment_type}",
                key=f"latex_metric_{tab_name}_{metric}_{experiment_type}"
            )

# ============================================================
# 16. PLOT NORMALIZED HAMMING DISTANCE
# ============================================================


def plot_hamming_distance(dataframe, tab_name):

    statistics = aggregate_hamming(dataframe)

    if statistics.empty:

        st.info(
            "No HITL normalized Hamming distance data available."
        )

        return

    fig = go.Figure()

    experiment_types = [
        "deterministic",
        "uncertain"
    ]

    for experiment_type in experiment_types:

        points = statistics[
            statistics["experiment_type"] == experiment_type
        ].sort_values("n_sensors")

        if points.empty:
            continue

        fig.add_trace(
            go.Scatter(

                x=points["n_sensors"],

                y=points["mean"],

                error_y=dict(
                    type="data",
                    array=points["std"],
                    visible=SHOW_ERROR_BARS
                ),

                mode="lines+markers",

                name=EXPERIMENT_LABELS.get(
                    experiment_type,
                    experiment_type.capitalize()
                ),

                marker=dict(
                    size=8,
                    color=EXPERIMENT_COLORS[experiment_type]
                ),

                line=dict(
                    width=2,
                    color=EXPERIMENT_COLORS[experiment_type]
                )

            )
        )

    fig.update_layout(

        # title=dict(
        #     text="Human Intervention Rate",
        #     x=0.5
        # ),

        width=PLOT_WIDTH,

        height=450,

        template="plotly_white",

        xaxis_title=r"$N_s$",

        yaxis_title=r"$\overline{HIR}$",

        hovermode="x unified",

        legend=dict(
            orientation="h",
            yanchor="top",
            y=-0.22,
            xanchor="center",
            x=0.5
        ),

        margin=dict(
            l=80,
            r=40,
            t=80,
            b=110
        )

    )

    fig.update_xaxes(
        tickmode="array",
        tickvals=sorted(
            statistics["n_sensors"].unique()
        )
    )

    fig.update_yaxes(
        range=[0, 1.05]
    )

    # --------------------------------------------------------
    # DISPLAY PLOT
    # --------------------------------------------------------

    display_centered_plot(
        fig,
        key=f"plot_hamming_{tab_name}"
    )

    # --------------------------------------------------------
    # PDF DOWNLOAD
    # --------------------------------------------------------

    download_pdf(
        fig=fig,
        filename=f"human_intervention_rate_{tab_name}.pdf",
        key=f"pdf_hamming_{tab_name}",
        plot_type="hamming"
    )

    # --------------------------------------------------------
    # STATISTICS TABLES
    # --------------------------------------------------------

    st.markdown("---")

    st.subheader("Human Intervention Rate Statistics")

    for experiment_type in experiment_types:

        experiment_data = statistics[
            statistics["experiment_type"] == experiment_type
        ].sort_values("n_sensors")

        st.markdown(
            f"**{EXPERIMENT_LABELS.get(experiment_type, experiment_type.capitalize())}**"
        )

        if experiment_data.empty:

            st.info(
                f"No {experiment_type} data available."
            )

            continue

        table_df = experiment_data[
            [
                "n_sensors",
                "mean",
                "std",
                "count"
            ]
        ].copy()

        table_df["Mean ± Standard Deviation"] = [
            f"{mean:.3f} ± {std:.3f}"
            for mean, std in zip(
                table_df["mean"],
                table_df["std"]
            )
        ]

        table_df = table_df.rename(
            columns={
                "n_sensors": r"$N_s$",
                "count": "Number of observations"
            }
        )

        table_df = table_df[
            [
                r"$N_s$",
                "Mean ± Standard Deviation",
                "Number of observations"
            ]
        ]

        st.dataframe(
            table_df,
            use_container_width=True,
            hide_index=True
        )

        # ----------------------------------------------------
        # LATEX EXPORT
        # ----------------------------------------------------

        download_latex_table(
            dataframe=table_df,
            filename=f"human_intervention_rate_{experiment_type}.tex",
            caption=(
                "Human Intervention Rate - "
                f"{EXPERIMENT_LABELS.get(experiment_type, experiment_type.capitalize())}"
            ),
            label=f"tab:human_intervention_rate_{experiment_type}",
            key=f"latex_hamming_{tab_name}_{experiment_type}",
            index=False
        )

# ============================================================
# 17. PLOT SIMILARITY
# ============================================================



# ============================================================
# PLOT SIMILARITY - 4, 4, 2 LAYOUT + TABLES
# ============================================================

def plot_similarity(
    aggregated,
    similarity_column,
    ylabel,
    title,
    filename,
    tab_name
):

    if aggregated.empty:

        st.info(f"No data available for {title}.")

        return

    method_pairs = sorted(
        aggregated["method_pair"].unique()
    )

    experiment_types = [
        "deterministic",
        "uncertain"
    ]

    n_pairs = len(method_pairs)

    if n_pairs == 0:
        return

    # --------------------------------------------------------
    # SUBPLOT POSITIONS
    # --------------------------------------------------------

    n_cols = 4
    n_rows = 3

    if n_pairs == 10:

        positions = [
            (1, 1), (1, 2), (1, 3), (1, 4),
            (2, 1), (2, 2), (2, 3), (2, 4),
            (3, 2), (3, 3)
        ]

    else:

        positions = [
            (i // n_cols + 1, i % n_cols + 1)
            for i in range(n_pairs)
        ]

    # Prepare subplot titles in the correct positions.

    subplot_titles = [""] * (n_rows * n_cols)

    for i, method_pair in enumerate(method_pairs):

        row, col = positions[i]

        subplot_index = (row - 1) * n_cols + col - 1

        subplot_titles[subplot_index] = method_pair

    # --------------------------------------------------------
    # CREATE FIGURE
    # --------------------------------------------------------

    fig = make_subplots(
        rows=n_rows,
        cols=n_cols,
        subplot_titles=subplot_titles,
        column_widths=PDF_CONFIG["similarity"]["column_widths"],
        row_heights=PDF_CONFIG["similarity"]["row_heights"],
        horizontal_spacing=PDF_CONFIG["similarity"]["horizontal_spacing"],
        vertical_spacing=PDF_CONFIG["similarity"]["vertical_spacing"]
    )

    # --------------------------------------------------------
    # PLOT EACH METHOD PAIR
    # --------------------------------------------------------

    for i, method_pair in enumerate(method_pairs):

        row, col = positions[i]

        pair_data = aggregated[
            aggregated["method_pair"] == method_pair
        ]

        for experiment_type in experiment_types:

            points = pair_data[
                pair_data["experiment_type"] == experiment_type
            ].sort_values("n_sensors")

            if points.empty:
                continue

            fig.add_trace(

                go.Scatter(

                    x=points["n_sensors"],

                    y=points[f"{similarity_column}_mean"],

                    error_y=dict(

                        type="data",

                        array=points[
                            f"{similarity_column}_std"
                        ],

                        visible=SHOW_ERROR_BARS

                    ),

                    mode="lines+markers",

                    name=EXPERIMENT_LABELS.get(
                        experiment_type,
                        experiment_type.capitalize()
                    ),

                    legendgroup=experiment_type,

                    showlegend=(i == 0),

                    marker=dict(

                        size=6,

                        color=EXPERIMENT_COLORS[experiment_type]

                    ),

                    line=dict(

                        width=2,

                        color=EXPERIMENT_COLORS[experiment_type]

                    )

                ),

                row=row,

                col=col

            )

        # X axis.

        fig.update_xaxes(

            title_text=r"$N_s$",

            tickmode="array",

            tickvals=sorted(
                pair_data["n_sensors"].unique()
            ),

            row=row,

            col=col

        )

        # Y axis.

        fig.update_yaxes(

            title_text=ylabel,

            range=[0, 1.05],

            row=row,

            col=col

        )

    # --------------------------------------------------------
    # HIDE UNUSED SUBPLOTS
    # --------------------------------------------------------

    for row in range(1, n_rows + 1):

        for col in range(1, n_cols + 1):

            if (row, col) not in positions:

                fig.update_xaxes(
                    visible=False,
                    row=row,
                    col=col
                )

                fig.update_yaxes(
                    visible=False,
                    row=row,
                    col=col
                )

    # --------------------------------------------------------
    # FIGURE LAYOUT
    # --------------------------------------------------------

    fig.update_layout(

        # title=dict(
        #     text=title,
        #     x=0.5
        # ),

        width=1600,

        height=1050,

        template="plotly_white",

        hovermode="closest",

        legend=dict(

            orientation="h",

            yanchor="top",

            y=-0.06,

            xanchor="center",

            x=0.5

        ),

        margin=dict(
            l=65,
            r=40,
            t=100,
            b=100
        )

    )

    # --------------------------------------------------------
    # DISPLAY FIGURE
    # --------------------------------------------------------

    st.plotly_chart(

        fig,

        use_container_width=True

    )

    # --------------------------------------------------------
    # DOWNLOAD PDF
    # --------------------------------------------------------

    download_pdf(
        fig=fig,
        filename=(
            f"{Path(filename).stem}_"
            f"{tab_name}_all_experiments.pdf"
        ),
        key=f"pdf_similarity_{tab_name}_{Path(filename).stem}",
        plot_type="similarity"
    )


    # ========================================================
    # TABLES: DETERMINISTIC AND UNCERTAIN
    # ========================================================

    st.markdown("---")

    st.markdown("### Similarity Statistics")

    for experiment_type in experiment_types:

        experiment_data = aggregated[
            aggregated["experiment_type"] == experiment_type
        ].copy()

        st.markdown(
            f"**{experiment_type.capitalize()}**"
        )

        if experiment_data.empty:

            st.info(
                f"No {experiment_type} data available."
            )

            continue

        # Create table.

        similarity_table = {}

        for n_sensors in sorted(
            experiment_data["n_sensors"].unique()
        ):

            similarity_table[n_sensors] = {}

            for method_pair in method_pairs:

                row = experiment_data[

                    (experiment_data["n_sensors"] == n_sensors) &

                    (experiment_data["method_pair"] == method_pair)

                ]

                if row.empty:

                    similarity_table[n_sensors][method_pair] = "-"

                else:

                    mean = row.iloc[0][
                        f"{similarity_column}_mean"
                    ]

                    std = row.iloc[0][
                        f"{similarity_column}_std"
                    ]

                    similarity_table[n_sensors][method_pair] = (

                        f"{mean:.3f} ± {std:.3f}"

                    )

        # Convert dictionary into DataFrame.

        similarity_table_df = pd.DataFrame.from_dict(

            similarity_table,

            orient="index"

        )

        similarity_table_df.index.name = r"$N_s$"

        similarity_table_df = similarity_table_df.reindex(

            columns=method_pairs

        )

        # Display table.

        st.dataframe(

            similarity_table_df,

            use_container_width=True

        )

        # ----------------------------------------------------
        # LATEX EXPORT
        # ----------------------------------------------------

        clean_filename = Path(filename).stem

        download_latex_table(
            dataframe=similarity_table_df,
            filename=f"{Path(filename).stem}_{experiment_type}.tex",
            caption=f"{title} - {experiment_type}",
            label=f"tab:{Path(filename).stem}_{experiment_type}",
            key=f"latex_similarity_{tab_name}_{Path(filename).stem}_{experiment_type}"
        )




# ============================================================
# 18. METRICS TABLES
# ============================================================

def show_metrics_tables(aggregated):

    if aggregated.empty:
        st.info("No metrics available.")
        return

    metrics_to_display = {

        "success_rate": "Success Rate",

        "efficiency": "Inspection Efficiency"

    }

    for metric, label in metrics_to_display.items():

        st.subheader(label)

        table = aggregated.pivot_table(

            index="n_sensors",

            columns="method",

            values=f"{metric}_mean",

            aggfunc="first"

        )

        std_table = aggregated.pivot_table(

            index="n_sensors",

            columns="method",

            values=f"{metric}_std",

            aggfunc="first"

        )

        formatted_table = pd.DataFrame(
            index=table.index
        )

        for method in METHODS_ORDER:

            if method not in selected_methods:
                continue

            if method in table.columns:

                formatted_table[method] = [

                    f"{mean:.3f} ± {std:.3f}"

                    for mean, std in zip(
                        table[method],
                        std_table[method]
                    )

                ]

            else:

                formatted_table[method] = "-"

        formatted_table.index.name = r"$N_s$"

        st.dataframe(

            formatted_table,

            use_container_width=False,

            width=850

        )


# ============================================================
# 19. SIMILARITY TABLES
# ============================================================



# ============================================================
# 20. FILTER METHOD PAIRS
# ============================================================

# Method names such as ILP-B30 contain hyphens.
# Therefore, method pairs must be matched against the
# original method names instead of using string.split("-").

def filter_similarity(dataframe):

    allowed_pairs = []

    for i, method_1 in enumerate(METHODS_ORDER):

        for method_2 in METHODS_ORDER[i + 1:]:

            if (
                method_1 in selected_methods
                and method_2 in selected_methods
            ):

                allowed_pairs.append(
                    f"{method_1}-{method_2}"
                )

                allowed_pairs.append(
                    f"{method_2}-{method_1}"
                )

    return dataframe[
        dataframe["method_pair"].isin(allowed_pairs)
    ].copy()


# ============================================================
# 21. SHOW DASHBOARD
# ============================================================

def show_dashboard(
    run_data,
    jaccard_data,
    cosine_data,
    tab_name
):

    if run_data.empty:

        st.warning("No data available for this selection.")

        return

    # --------------------------------------------------------
    # FILTER DATA
    # --------------------------------------------------------

    run_data = filter_run_metrics(run_data)

    jaccard_data = filter_similarity(jaccard_data)

    cosine_data = filter_similarity(cosine_data)

    # --------------------------------------------------------
    # AGGREGATION
    # --------------------------------------------------------

    aggregated = aggregate_run_metrics(run_data)

    jaccard_aggregated = aggregate_jaccard(
        jaccard_data
    )

    cosine_aggregated = aggregate_cosine(
        cosine_data
    )

    binary_cosine = cosine_aggregated[
        cosine_aggregated["similarity_type"] == "binary"
    ].copy()

    weighted_cosine = cosine_aggregated[
        cosine_aggregated["similarity_type"] == "weighted"
    ].copy()

    # --------------------------------------------------------
    # SUMMARY
    # --------------------------------------------------------

    col1, col2, col3 = st.columns(3)

    with col1:

        st.metric(
            "Total runs",
            len(run_data)
        )

    with col2:

        st.metric(
            "Methods selected",
            len(selected_methods)
        )

    with col3:

        st.metric(
            "Experiment types",
            run_data["experiment_type"].nunique()
        )

    st.divider()

    # --------------------------------------------------------
    # SUCCESS RATE
    # --------------------------------------------------------

    st.header("Experimental Performance")

    st.subheader("Success Rate")

    plot_metric(
        aggregated=aggregated,
        metric="success_rate",
        ylabel=r"$\overline{SR}$",
        title="Mean Success Rate",
        filename="success_rate.pdf",
        tab_name=tab_name
    )

    # --------------------------------------------------------
    # INSPECTION EFFICIENCY
    # --------------------------------------------------------

    st.subheader("Inspection Efficiency")

    plot_metric(
        aggregated=aggregated,
        metric="efficiency",
        ylabel=r"$\overline{\eta}$",
        title="Mean Inspection Efficiency",
        filename="inspection_efficiency.pdf",
        tab_name=tab_name
    )

    # --------------------------------------------------------
    # NORMALIZED HAMMING DISTANCE
    # --------------------------------------------------------

    st.subheader("Human Intervention Rate")

    if "HITL" in selected_methods:

        plot_hamming_distance(run_data, tab_name)

    else:

        st.info(
            "Select HITL in the sidebar to visualize "
            "Human Intervention Distance."
        )

    st.divider()

    # --------------------------------------------------------
    # JACCARD SIMILARITY
    # --------------------------------------------------------

    st.header("Method Similarity")

    st.subheader("Jaccard Similarity")

    if not jaccard_aggregated.empty:

        plot_similarity(
            aggregated=jaccard_aggregated,
            similarity_column="jaccard_similarity",
            ylabel=r"$\overline{JS}$",
            title="Mean Jaccard Similarity",
            filename="jaccard_similarity.pdf",
            tab_name=tab_name
        )

    else:

        st.info(
            "No Jaccard comparisons available "
            "for the selected methods."
        )

    # --------------------------------------------------------
    # BINARY COSINE SIMILARITY
    # --------------------------------------------------------

    st.subheader("Binary Cosine Similarity")

    if not binary_cosine.empty:

        plot_similarity(
            aggregated=binary_cosine,
            similarity_column="cosine_similarity",
            ylabel=r"$\overline{CS}_{binary}$",
            title="Mean Binary Cosine Similarity",
            filename="binary_cosine_similarity.pdf",
            tab_name=tab_name
        )

    else:

        st.info(
            "No binary cosine comparisons available "
            "for the selected methods."
        )

    # --------------------------------------------------------
    # WEIGHTED COSINE SIMILARITY
    # --------------------------------------------------------

    st.subheader("Weighted Cosine Similarity")

    if not weighted_cosine.empty:

        plot_similarity(
            aggregated=weighted_cosine,
            similarity_column="cosine_similarity",
            ylabel=r"$\overline{CS}_{weighted}$",
            title="Mean Weighted Cosine Similarity",
            filename="weighted_cosine_similarity.pdf",
            tab_name=tab_name
        )

    else:

        st.info(
            "No weighted cosine comparisons available "
            "for the selected methods."
        )

    st.divider()


# ============================================================
# 22. DASHBOARD TABS
# ============================================================

tab_overview, tab_world1, tab_world2 = st.tabs([

    "Overview",

    "World 1",

    "World 2"

])


# ============================================================
# 23. OVERVIEW
# ============================================================

with tab_overview:

    st.header("Overview")

    st.caption(
        "Metrics aggregated across all available worlds."
    )

    # No world filtering is applied here.
    # All observations are aggregated directly.
    show_dashboard(
        run_data=df,
        jaccard_data=jaccard_df,
        cosine_data=cosine_df,
        tab_name="overview"
    )


# ============================================================
# 24. WORLD 1
# ============================================================

with tab_world1:

    st.header("World 1")

    world1_df = df[
        df["world"] == "world_1"
    ].copy()

    world1_jaccard = jaccard_df[
        jaccard_df["world"] == "world_1"
    ].copy()

    world1_cosine = cosine_df[
        cosine_df["world"] == "world_1"
    ].copy()

    if world1_df.empty:

        st.warning("No data available for World 1.")

    else:

        show_dashboard(
            run_data=world1_df,
            jaccard_data=world1_jaccard,
            cosine_data=world1_cosine,
            tab_name="world_1"
        )


# ============================================================
# 25. WORLD 2
# ============================================================

with tab_world2:

    st.header("World 2")

    world2_df = df[
        df["world"] == "world_2"
    ].copy()

    world2_jaccard = jaccard_df[
        jaccard_df["world"] == "world_2"
    ].copy()

    world2_cosine = cosine_df[
        cosine_df["world"] == "world_2"
    ].copy()

    if world2_df.empty:

        st.warning("No data available for World 2.")

    else:

        show_dashboard(
            run_data=world2_df,
            jaccard_data=world2_jaccard,
            cosine_data=world2_cosine,
            tab_name="world_2"
        )


# ============================================================
# 26. END
# ============================================================

st.sidebar.divider()

st.sidebar.success(
    "Dashboard loaded successfully."
)
