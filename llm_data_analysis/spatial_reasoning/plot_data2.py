
import json
import math
import re
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import streamlit as st


# ============================================================
# CONFIGURAZIONE
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
JSON_PATH = BASE_DIR / "evaluations.json"

FIGURE_SIZE_PER_SUBPLOT = (4.5, 3.5)
DPI = 300
VARIANCE_DDOF = 1


# ============================================================
# FUNZIONI DI SUPPORTO
# ============================================================

def natural_key(value):
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", str(value))
    ]


def count_sensors(sensor_sequence):
    if not sensor_sequence:
        return 0

    if isinstance(sensor_sequence, str):
        sensor_sequence = [sensor_sequence]

    sensors = []

    for item in sensor_sequence:
        if isinstance(item, str):
            sensors.extend(
                sensor.strip()
                for sensor in item.split(",")
                if sensor.strip()
            )

    return len(set(sensors))


def get_assistant_messages(run_data):
    value = run_data.get("message_counts", {}).get("assistant")

    if value is None:
        return np.nan

    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


# ============================================================
# LETTURA JSON
# ============================================================


@st.cache_data
def load_runs(json_path, file_mtime):
    with open(json_path, "r", encoding="utf-8") as file:
        data = json.load(file)

    records = []

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

                        for category, details in raw_errors.items():
                            if not isinstance(details, dict):
                                continue

                            # Totale della categoria, ad esempio:
                            # Reasoning_errors -> 3
                            total = details.get("total", 0)

                            if isinstance(total, (int, float)):
                                category_totals[category] = float(total)

                            # Conteggi delle singole sottocategorie
                            subcategories = details.get("subcategories", {})

                            if isinstance(subcategories, dict):
                                for subcategory, value in subcategories.items():
                                    if isinstance(value, (int, float)):
                                        subcategory_counts[
                                            f"{category} :: {subcategory}"
                                        ] = float(value)

                        records.append({
                            "mode": mode,
                            "world": world,
                            "model": model,
                            "trial": trial,
                            "run": run_id,
                            "n_sensors": count_sensors(
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
        raise ValueError("Non sono stati trovati dati validi nel JSON.")

    return df

# ============================================================
# RICALCOLO DELLE METRICHE
# ============================================================


def calculate_metrics(df, selected_errors):
    df = df.copy()

    if not selected_errors:
        df["selected_errors"] = np.nan
        df["error_rate"] = np.nan
        return df

    def sum_selected_errors(row):
        total = 0.0

        for error in selected_errors:
            if " :: " in error:
                # Categoria e sottocategoria, ad esempio:
                # Reasoning_errors :: wrong area proposed
                total += row["subcategory_counts"].get(error, 0.0)
            else:
                # Categoria principale
                total += row["error_counts"].get(error, 0.0)

        return total

    df["selected_errors"] = df.apply(sum_selected_errors, axis=1)

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


def compute_statistics(df):
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


# ============================================================
# GRAFICI
# ============================================================

import plotly.graph_objects as go
from plotly.subplots import make_subplots


def create_figure(summary, selected_modes, selected_models):
    modes = sorted(selected_modes, key=natural_key)
    models = sorted(selected_models, key=natural_key)

    if not modes or not models or summary.empty:
        return None

    ncols = min(3, len(modes))
    nrows = math.ceil(len(modes) / ncols)

    fig = make_subplots(
        rows=nrows,
        cols=ncols,
        subplot_titles=[mode.replace("_", " ") for mode in modes],
        horizontal_spacing=0.10,
        vertical_spacing=0.16,
    )

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

            if model_data.empty:
                continue

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
                    line=dict(
                        color=colors[model],
                        width=2,
                    ),
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
# ============================================================
# INTERFACCIA STREAMLIT
# ============================================================

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
    "Seleziona le categorie di errore e osserva "
    "come cambiano le statistiche."
)

if not JSON_PATH.exists():
    st.error(f"File JSON non trovato: {JSON_PATH}")
    st.stop()

try:
    df = load_runs(str(JSON_PATH), JSON_PATH.stat().st_mtime)
except (json.JSONDecodeError, OSError, ValueError) as exc:
    st.error(f"Impossibile caricare il JSON: {exc}")
    st.stop()




# ============================================================
# SIDEBAR: CATEGORIE DI ERRORE E FILTRI
# ============================================================

st.sidebar.header("Error selection")

ERROR_GROUPS = {
    "Reasoning Errors": "Reasoning_errors",
    "Hallucinations": "Hallucinations",
    "Wrong Function Calls": "Wrong_function_calls",
}

# Recupera le sottocategorie dal JSON.
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

# ============================================================
# APPLICAZIONE DEI FILTRI E CALCOLO DELLE METRICHE
# ============================================================

filtered = df[
    df["mode"].isin(selected_modes)
    & df["model"].isin(selected_models)
    & df["n_sensors"].isin(selected_sensors)
].copy()

filtered = calculate_metrics(filtered, selected_errors)

valid = filtered[
    (filtered["n_sensors"] > 0)
    & filtered["error_rate"].notna()
].copy()

if not selected_errors:
    st.warning("Select at least one error subcategory.")

if valid.empty:
    st.warning("No valid runs available for the current selection.")
    st.stop()


# ------------------------------------------------------------
# Statistiche e indicatori
# ------------------------------------------------------------

summary = compute_statistics(valid)

col1, col2, col3 = st.columns(3)

col1.metric("Valid runs", len(valid))
col2.metric("Error categories", len(selected_errors))
col3.metric("Models", valid["model"].nunique())

st.subheader("Error rate by mode and model")

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
    )

    st.download_button(
        "Download plot (HTML interattivo)",
        data=fig.to_html(include_plotlyjs=True),
        file_name="spatial_reasoning.html",
        mime="text/html",
    )

    # Esportazione PNG/PDF tramite Matplotlib non più applicabile
    # direttamente alla figura Plotly.
# ------------------------------------------------------------
# Tabella statistiche
# ------------------------------------------------------------

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
    file_name="error_rate_statistics.csv",
    mime="text/csv",
)


# ------------------------------------------------------------
# Tabella pivot
# ------------------------------------------------------------

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

pivot = pivot_data.pivot(
    index=["n_sensors", "model"],
    columns="mode",
    values="mean ± std",
).sort_index()

st.dataframe(pivot, use_container_width=True)

st.download_button(
    "Download comparison table (CSV)",
    data=pivot.to_csv().encode("utf-8"),
    file_name="error_rate_summary_table.csv",
    mime="text/csv",
)


# ------------------------------------------------------------
# Dettaglio delle singole run
# ------------------------------------------------------------

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
        file_name="individual_run_error_rates.csv",
        mime="text/csv",
    )

