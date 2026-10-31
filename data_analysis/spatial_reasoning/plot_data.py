
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

# ERROR_CATEGORIES = {
#     "Spatial_and_Environment_Understanding": [
#         "wrong intruder movement hypothesis",
#         "wrong spatial information for area connections",
#         "wrong spatial information for sensors coverage",
#     ],
#     "Resource_Selection_and_Task_Assignment": [
#         "wrong area proposed",
#         "wrong robot camera inspections assignment",
#         "wrong robot deployed",
#         "wrong sensor considered",
#         "unproposed area to inspect",
#         "unproposed camera usage",
#         "unoptimal robot deployment",
#         "useless proposed camera inspection",
#     ],
#     "Information_Retrieval_and_Context_Selection": [
#         "called get_areas_info with missing areas",
#         "called get_areas_info with wrong area",
#         "get_areas_info with useless areas",
#         "get_sensors_info with useless sensors",
#     ],
#     "Distance_Computation_and_Decision_Integration": [
#         "compute_distance for robot assignment not used",
#         "computed_distance_to_area_covered_by_camera",
#         "computed_distance_to_wrong_area",
#         "compute_distance used but not included for optimal robot deployment",
#     ],
#     "Function_Calling_and_Parameter_Validity": [
#         "called_again_the_same_function",
#         "Synthatically wrong parameters generated",
#     ],
#     "Operational_Constraint_Violations": [
#         "Unapproved action execution",
#     ],
# }

ERROR_CATEGORIES = {
            "Information_Retrieval_and_Spatial_Understanding": [
                "wrong intruder movement hypothesis",
                "wrong spatial information for area connections",
                "wrong spatial information for sensors coverage",
                "called get_areas_info with missing areas",
                "called get_areas_info with wrong area",
            ],

            "Resource_Selection_and_Task_Assignment": [
                "wrong area proposed",
                "unproposed area to inspect",
                "wrong robot camera inspections assignment",
                "wrong robot deployed",
                "wrong sensor considered",
                "unproposed camera usage",
                "useless proposed camera inspection",
                "unoptimal robot deployment"
            ],

            "Function_Usage_Inefficiency": [
                "get_areas_info with useless areas",
                "get_sensors_info with useless sensors",
                "called_again_the_same_function",
                "Synthatically wrong parameters generated"

            ],

            "Operational_Constraint_Violations": [
                "Unapproved action execution",
                "compute_distance for robot assignment not used",
                "computed_distance_to_area_covered_by_camera",
                "compute_distance used but not included for optimal robot deployment"
            ]
}

# Human-readable labels used in heatmap axes and dashboard controls.
# CATEGORY_LABELS = {
#     "Spatial_and_Environment_Understanding":
#         "Spatial understanding",
#     "Resource_Selection_and_Task_Assignment":
#         "Resource selection / assignment",
#     "Information_Retrieval_and_Context_Selection":
#         "Information retrieval",
#     "Distance_Computation_and_Decision_Integration":
#         "Distance reasoning",
#     "Function_Calling_and_Parameter_Validity":
#         "Function calling",
#     "Operational_Constraint_Violations":
#         "Operational constraints",
# }

CATEGORY_LABELS = {
    "Information_Retrieval_and_Spatial_Understanding":
        "Spatial understanding",
    "Resource_Selection_and_Task_Assignment":
        "Resource selection / assignment",
    "Distance_Computation_and_Decision_Integration":
        "Distance reasoning",
    "Function_Usage_Inefficiency":
        "Function inefficiency",
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

MODEL_LABELS = {
    "gpt-4o": "GPT-4o",
    "gpt-6-luna": "GPT-6-Luna"
}



# HELPER FUNCTIONS

def get_mode_label(mode):
    return MODE_LABELS.get(
        mode,
        mode.replace("_", " ").title()
    )


def get_model_label(model):
    return MODEL_LABELS.get(model, model)

def error_subcategories_to_latex(
    table,
    model,
    caption=None,
    label="tab:error_subcategories",
):
    """
    Generate a LaTeX table containing absolute error counts
    for each error subcategory and retrieval mode.
    """

    if table.empty:
        raise ValueError("Cannot generate LaTeX from an empty table.")

    def escape_latex(value):

        if pd.isna(value):
            return "N/A"

        replacements = {
            "\\": r"\textbackslash{}",
            "&": r"\&",
            "%": r"\%",
            "$": r"\$",
            "#": r"\#",
            "_": r"\_",
            "{": r"\{",
            "}": r"\}",
            "~": r"\textasciitilde{}",
            "^": r"\textasciicircum{}",
        }

        return re.sub(
            r"[\\&%$#_{}~^]",
            lambda match: replacements[match.group()],
            str(value),
        )

    lines = []

    lines.append(r"\begin{table*}[htbp]")
    lines.append(r"\centering")

    if caption:
        lines.append(
            rf"\caption{{{escape_latex(caption)}}}"
        )

    lines.append(rf"\label{{{label}}}")
    lines.append(r"\scriptsize")
    lines.append(r"\setlength{\tabcolsep}{5pt}")
    lines.append("")

    # One left-aligned column for subcategories
    # and one centered column per retrieval mode.
    column_format = "l " + " ".join(
        ["c"] * (len(table.columns) - 1)
    )

    lines.append(
        rf"\begin{{tabular}}{{{column_format}}}"
    )

    lines.append(r"\toprule")

    # Header.
    headers = [
        escape_latex(column)
        for column in table.columns
    ]

    lines.append(
        " & ".join(
            [rf"\textbf{{{header}}}" for header in headers]
        ) + r" \\"
    )

    lines.append(r"\midrule")

    # Table rows.
    for _, row in table.iterrows():

        cells = []

        for value in row:
            if pd.isna(value):
                cells.append("N/A")
            elif isinstance(value, (int, float, np.number)):
                cells.append(str(int(value)))
            else:
                cells.append(escape_latex(value))

        lines.append(" & ".join(cells) + r" \\")

    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")
    lines.append(r"\end{table*}")

    return "\n".join(lines)




def comparison_table_to_latex(
    table,
    caption=None,
    label="tab:normalized_error_results",
):
    """
    Generate a publication-ready LaTeX table.

    Expected DataFrame:
        MultiIndex: n_sensors, model
        Columns: retrieval modes
        Values: mean ± standard deviation

    Features:
        - Fixed retrieval mode ordering.
        - Multirow grouping by number of sensors.
        - Automatic conversion of ± into LaTeX \pm.
        - Escaping of LaTeX special characters.
        - table* environment.
    """

    # Preferred retrieval mode order.
    ordered_modes = [
        "rag_with_functions",
        "rag_with_semantics",
        "image",
        "image_and_rag_with_functions",
        "image_and_rag_with_semantics",
    ]

    # Convert original mode names into display labels.
    available_modes = [
        mode for mode in ordered_modes
        if mode in table.columns
    ]

    # Keep additional modes, if present, after the preferred ones.
    available_modes += [
        mode for mode in table.columns
        if mode not in available_modes
    ]

    # Reorder columns.
    latex_df = table[available_modes].copy()

    # Ensure the expected index structure.
    if not isinstance(latex_df.index, pd.MultiIndex):
        raise ValueError(
            "The comparison table must have a MultiIndex "
            "with n_sensors and model."
        )

    if latex_df.index.nlevels != 2:
        raise ValueError(
            "Expected exactly two index levels: n_sensors and model."
        )

    latex_df.index.names = ["n_sensors", "model"]

    # Escape special LaTeX characters.
    def escape_latex(value):
        if pd.isna(value):
            return "N/A"

        value = str(value)

        replacements = {
            "\\": r"\textbackslash{}",
            "&": r"\&",
            "%": r"\%",
            "$": r"\$",
            "#": r"\#",
            "_": r"\_",
            "{": r"\{",
            "}": r"\}",
            "~": r"\textasciitilde{}",
            "^": r"\textasciicircum{}",
        }

        return re.sub(
            r"[\\&%$#_{}~^]",
            lambda match: replacements[match.group()],
            value,
        )

    # Convert mean ± standard deviation into LaTeX math.
    def format_value(value):

        if pd.isna(value):
            return "N/A"

        value = str(value).strip()

        # Replace ± with the LaTeX command.
        value = value.replace("±", r"\pm")

        if r"\pm" in value:
            return f"${value}$"

        return escape_latex(value)

    # LaTeX column headers.
    headers = [
        MODE_LABELS.get(mode, mode)
        for mode in available_modes
    ]

    # Add labels for any additional modes.
    headers += [
        MODE_LABELS.get(mode, mode)
        for mode in available_modes[len(headers):]
    ]

    ncols = len(available_modes)

    # Build tabular column specification.
    column_format = "c l " + " ".join(["c"] * ncols)

    lines = []

    lines.append(r"\begin{table*}[h!]")
    lines.append(r"\centering")

    if caption:
        lines.append(rf"\caption{{{escape_latex(caption)}}}")

    lines.append(rf"\label{{{label}}}")
    lines.append(r"\scriptsize")
    lines.append(r"\setlength{\tabcolsep}{3pt}")
    lines.append("")
    lines.append(rf"\begin{{tabular}}{{{column_format}}}")
    lines.append(r"\toprule")

    # Header.
    header = [
        r"\textbf{$\boldsymbol{N_s}$}",
        r"\textbf{LLM}",
    ]

    header += [
        rf"\textbf{{{escape_latex(mode)}}}"
        for mode in headers
    ]

    lines.append(" & ".join(header) + r" \\")
    lines.append(r"\midrule")
    lines.append("")

    # Sort rows by number of sensors and model.
    latex_df = latex_df.sort_index(
        level=["n_sensors", "model"]
    )

    
    # Group rows by sensor count.
    sensor_groups = list(
        latex_df.groupby(
            level="n_sensors",
            sort=True,
        )
    )

    for group_index, (sensor_count, group) in enumerate(sensor_groups):

        group_size = len(group)

        for row_number, (index, row) in enumerate(group.iterrows()):

            model = index[1]

            cells = []

            # Multirow only on the first row of each sensor group.
            if row_number == 0:
                cells.append(
                    rf"\multirow{{{group_size}}}{{*}}{{{escape_latex(sensor_count)}}}"
                )
            else:
                cells.append("")

            model_label = MODEL_LABELS.get(model, model)
            cells.append(escape_latex(model_label))

            for mode in available_modes:
                cells.append(format_value(row[mode]))

            lines.append(" & ".join(cells) + r" \\")

        # Separate different sensor groups.
        if group_index < len(sensor_groups) - 1:
            lines.append(r"\midrule")


    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")
    lines.append(r"\end{table*}")

    return "\n".join(lines)


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

    # The final records will be a list of dictionary (each dictionary will be a run):
    # records = [
    #     {
    #         "mode": "rag_with_functions",
    #         "world": "world_1",
    #         "model": "model_A",
    #         "trial": "trial_1",
    #         "run": "run_1",
    #         "n_sensors": 3,
    #         "assistant_messages": 10,
    #         "error_counts": {
    #             "Reasoning_errors": 5,
    #             "Hallucinations": 2
    #         },
    #         "subcategory_counts": {
    #             "Reasoning_errors :: wrong area proposed": 2,
    #             "Reasoning_errors :: wrong robot deployed": 3,
    #             "Hallucinations :: wrong sensor considered": 2
    #         }
    #     }
    #     .....
    # ]


    # By using a DataFrame we can convert the list of dictionary in a table
    # where each key of the single dictionary is a column and each dictionary a row:
    #       mode             world       model      trial       run     n_sensors   .......
    # rag_with_functions    world_1     model_A    trial_1     run_1        3    ......

    df = pd.DataFrame(records)
    

    if df.empty:
        raise ValueError("No valid run records were found in the JSON.")

    return df


# ERROR-RATE CALCULATION

def compute_metrics(df, selected_errors):
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


def build_heatmap_data(data, selected_errors):
    """
    Convert selected subcategory counts into macro-category counts.
    Only selected errors contribute to the heatmaps.
    """

    records = []

    for _, row in data.iterrows():

        subcategory_counts = row["subcategory_counts"]

        record = {
            "mode": row["mode"],
            "world": row["world"],
            "model": row["model"],
            "run": row["run"],
            "n_sensors": row["n_sensors"],
        }

        # Aggregate only selected subcategories.
        for category, expected_errors in ERROR_CATEGORIES.items():

            record[category] = sum(
                count
                for key, count in subcategory_counts.items()
                if (
                    key in selected_errors
                    and key.split(" :: ", 1)[-1] in expected_errors
                )
            )

        records.append(record)

    return pd.DataFrame(records)

# HEATMAP METRICS

def compute_heatmap_metrics(run_df):
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

    # The received dataFrame will be:
    #       mode             world       model      trial       run     n_sensors   .......     Spatial_and_Environment_Understanding   Resource_Selection_and_Task_Assignment  Information_Retrieval_and_Context_Selection     ........
    # rag_with_functions    world_1     model_A    trial_1     run_1        3        ......                     1                                       7                                          0                           ........
    # rag_with_functions    world_1     model_A    trial_1     run_2        2        ......                     0                                       3                                          3                           ........
    # rag_with_functions    world_1     model_A    trial_1     run_3        3        ......                     4                                       0                                          3                           ........
    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # rag_with_functions    world_1     model_B    trial_1     run_1        1        ......                     2                                       5                                          0                           ........
    # rag_with_functions    world_1     model_B    trial_1     run_2        2        ......                     0                                       1                                          4                           ........
    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # rag_with_semantics    world_1     model_A    trial_1     run_1        1        ......                     5                                       2                                          0                           ........
    # rag_with_semantics    world_1     model_A    trial_1     run_2        2        ......                     0                                       2                                          3                           ........
    # rag_with_semantics    world_1     model_A    trial_1     run_3        3        ......                     6                                       0                                          5                           ........
    
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

    # The resulting binary_df will be:
    #       mode             world       model      trial       run     n_sensors   .......     Spatial_and_Environment_Understanding   Resource_Selection_and_Task_Assignment  Information_Retrieval_and_Context_Selection     ........
    # rag_with_functions    world_1     model_A    trial_1     run_1        1        ......                     1                                       1                                          0                           ........
    # rag_with_functions    world_1     model_A    trial_1     run_2        2        ......                     0                                       1                                          1                           ........
    # rag_with_functions    world_1     model_A    trial_1     run_3        3        ......                     1                                       0                                          1                           ........
    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # rag_with_functions    world_1     model_B    trial_1     run_1        1        ......                     1                                       1                                          0                           ........
    # rag_with_functions    world_1     model_B    trial_1     run_2        2        ......                     0                                       1                                          1                           ........
    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # rag_with_semantics    world_1     model_A    trial_1     run_1        1        ......                     1                                       1                                          0                           ........
    # rag_with_semantics    world_1     model_A    trial_1     run_2        2        ......                     0                                       1                                          1                           ........
    # rag_with_semantics    world_1     model_A    trial_1     run_3        3        ......                     1                                       0                                          1                           ........
    

    # Average the binary indicators to obtain the fraction of affected
    # runs, then multiply by 100 to express the result as a percentage.
    prevalence = (
        binary_df.groupby(["mode", "model"])[categories]    # groupby() gather dataFrames rows based on the selected columns "mode" and "model" and selecting only the columns in [categories]
        .mean()                                             # Compute the mean among rows (axis=0) of the same column, for each column in [categories]
        .mul(100)                                           # Multiply by 100
    )

    # The dataFrame will after .groupby(["mode", "model"])[categories] will be a table with each row a unique combination of mode-model and corresponding [categories] values
    #       mode              model      Spatial_and_Environment_Understanding      Resource_Selection_and_Task_Assignment      Information_Retrieval_and_Context_Selection     ........
    # rag_with_functions     model_A                       1                                          1                                             0                           ........
    # rag_with_functions     model_A                       0                                          1                                             0                           ........
    # rag_with_functions     model_A                       1                                          0                                             1                           ........
    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #       mode              model      Spatial_and_Environment_Understanding      Resource_Selection_and_Task_Assignment      Information_Retrieval_and_Context_Selection     ........
    # rag_with_functions     model_B                       0                                          1                                             0                           ........
    # rag_with_functions     model_B                       0                                          0                                             1                           ........
    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #       mode              model      Spatial_and_Environment_Understanding      Resource_Selection_and_Task_Assignment      Information_Retrieval_and_Context_Selection     ........
    # rag_with_semantics     model_A                       1                                          1                                             0                           ........
    # rag_with_semantics     model_A                       0                                          1                                             1                           ........
    # rag_with_semantics     model_A                       1                                          0                                             1                           ........
        


    # The final dataFrame  "prevalence" after the mean will be:
    #       mode              model      Spatial_and_Environment_Understanding      Resource_Selection_and_Task_Assignment      Information_Retrieval_and_Context_Selection     ........
    # rag_with_functions     model_A                      0.67                                      0.67                                           0.33                         ........
    # rag_with_functions     model_B                      0.00                                      0.50                                           0.50                         ........
    # rag_with_semantics     model_A                      0.67                                      0.67                                           0.67                         ........



    # Sum the actual error occurrences in each category.
    counts = (
        run_df.groupby(["mode", "model"])[categories]   # It sums along the columns in [categories]
        .sum()
    )


    # The final dataFrame "counts" after the mean will be:
    #       mode              model      Spatial_and_Environment_Understanding      Resource_Selection_and_Task_Assignment      Information_Retrieval_and_Context_Selection     ........
    # rag_with_functions     model_A                      5                                         10                                           6                              ........
    # rag_with_functions     model_B                      2                                         6                                            4                              ........
    # rag_with_semantics     model_A                      11                                        4                                            8                              ........

    # Normalize each group by its total number of mapped errors.
    # Replacing zero totals with NaN avoids division by zero.
    totals = counts.sum(axis=1).replace(0, np.nan)          # We compute the total errors for a given (mode-model) combination among all the columns for a single row (axis = 1), 

    #       mode              model      Spatial_and_Environment_Understanding      Resource_Selection_and_Task_Assignment      Information_Retrieval_and_Context_Selection     Total
    # rag_with_functions     model_A                      5                                         10                                           6                               21
    # rag_with_functions     model_B                      2                                         6                                            4                               12
    # rag_with_semantics     model_A                      11                                        4                                            8                               23

    distribution = counts.div(totals, axis=0).mul(100)  # divide each value in the single row for its corresponding total (obtaining the % of a given error in the total errors) for the (mode-model) combination

    return prevalence, distribution




def build_error_comparison_tables(data, selected_errors,
                                  selected_modes, selected_models):
    """
    Build one comparison table per model.

    Rows: selected error subcategories.
    Columns: retrieval modes.
    Values: absolute error occurrences.
    """

    tables = {}

    for model in selected_models:

        model_data = data[data["model"] == model]

        records = []

        for error_key in selected_errors:

            if " :: " not in error_key:
                continue

            _, subcategory = error_key.split(" :: ", 1)

            record = {
                "Error subcategory": subcategory
            }

            
            ordered_modes = [
                mode for mode in MODE_ORDER
                if mode in selected_modes
            ]

            for mode in ordered_modes:

                mode_data = model_data[
                    model_data["mode"] == mode
                ]

                total = sum(
                    row["subcategory_counts"].get(error_key, 0)
                    for _, row in mode_data.iterrows()
                )

                record[MODE_LABELS.get(mode, mode)] = int(total)

            records.append(record)

        tables[model] = pd.DataFrame(records)

    return tables




def render_error_comparison_tables(
    data,
    selected_errors,
    selected_modes,
    selected_models,
    key_suffix,
):
    st.subheader("Absolute error counts by subcategory")

    tables = build_error_comparison_tables(
        data,
        selected_errors,
        selected_modes,
        selected_models,
    )

    for model, table in tables.items():

        st.markdown(f"**{MODEL_LABELS.get(model, model)}**")

        if table.empty:
            st.info(f"No error data available for {model}.")
            continue

        st.dataframe(
            table,
            use_container_width=True,
            hide_index=True,
        )

        # CSV DOWNLOAD
        st.download_button(
            f"Download {model} error counts (CSV)",
            data=table.to_csv(index=False).encode("utf-8"),
            file_name=f"error_counts_{model}_{key_suffix}.csv",
            mime="text/csv",
            key=f"download_error_counts_{model}_{key_suffix}",
        )

        # LATEX DOWNLOAD
        latex_code = error_subcategories_to_latex(
            table=table,
            model=model,
            caption=(
                f"Absolute error counts by subcategory "
                f"and retrieval mode for "
                f"{MODEL_LABELS.get(model, model)}."
            ),
            label=f"tab:error_counts_{model}_{key_suffix}",
        )

        st.download_button(
            label=f"Download {model} error counts (LaTeX)",
            data=latex_code.encode("utf-8"),
            file_name=f"error_counts_{model}_{key_suffix}.tex",
            mime="text/plain",
            key=f"download_error_counts_latex_{model}_{key_suffix}",
        )



# INTERACTIVE LINE CHART

# def create_figure(summary, selected_modes, selected_models):
#     """
#     Create a grid of line charts showing mean error rates.

#     Each subplot represents one retrieval mode. Each line represents
#     a model, with the number of active sensors on the x-axis and the
#     mean error rate on the y-axis. Error bars show the sample standard
#     deviation across valid runs.

#     Args:
#         summary: Aggregated DataFrame returned by compute_statistics().
#         selected_modes: Retrieval modes to include.
#         selected_models: Models to include.

#     Returns:
#         A Plotly Figure, or None if there is no data to display.
#     """
#     modes = sorted(selected_modes, key=natural_key)
#     models = sorted(selected_models, key=natural_key)

#     if not modes or not models or summary.empty:
#         return None

#     # Arrange up to three mode-specific plots in each row.
#     ncols = min(3, len(modes))
#     nrows = math.ceil(len(modes) / ncols)

#     fig = make_subplots(
#         rows=nrows,
#         cols=ncols,
#         subplot_titles=[mode.replace("_", " ") for mode in modes],
#         horizontal_spacing=0.10,
#         vertical_spacing=0.16,
#     )

#     # Assign a consistent color to each model across all subplots.
#     palette = [
#         "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
#         "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
#         "#bcbd22", "#17becf",
#     ]

#     colors = {
#         model: palette[i % len(palette)]
#         for i, model in enumerate(models)
#     }

#     for i, mode in enumerate(modes):
#         row = i // ncols + 1
#         col = i % ncols + 1

#         mode_data = summary[summary["mode"] == mode]

#         for model in models:
#             model_data = (
#                 mode_data[mode_data["model"] == model]
#                 .sort_values("n_sensors")
#             )

#             # Do not create empty traces for unavailable combinations.
#             if model_data.empty:
#                 continue

#             # A missing standard deviation (for example, a single
#             # observation) is displayed with a zero-length error bar.
#             yerr = model_data["std_deviation"].fillna(0)

#             fig.add_trace(
#                 go.Scatter(
#                     x=model_data["n_sensors"],
#                     y=model_data["mean_error_rate"],
#                     error_y=dict(
#                         type="data",
#                         array=yerr,
#                         visible=True,
#                         thickness=1.2,
#                         width=4,
#                     ),
#                     mode="lines+markers",
#                     name=model,
#                     legendgroup=model,
#                     showlegend=(i == 0),
#                     line=dict(color=colors[model], width=2),
#                     marker=dict(size=6),
#                     hovertemplate=(
#                         "Model: " + model +
#                         "<br>Sensors: %{x}" +
#                         "<br>Mean error rate: %{y:.4f}" +
#                         "<extra></extra>"
#                     ),
#                 ),
#                 row=row,
#                 col=col,
#             )

#         # Configure the axes independently for each subplot.
#         fig.update_xaxes(
#             title_text="Active sensors",
#             dtick=1,
#             row=row,
#             col=col,
#         )

#         fig.update_yaxes(
#             title_text="Mean error rate",
#             showgrid=True,
#             gridcolor="rgba(128,128,128,0.2)",
#             row=row,
#             col=col,
#         )

#     # Configure the overall figure layout and shared legend.
#     fig.update_layout(
#         title="Spatial Reasoning",
#         height=320 * nrows,
#         autosize=True,
#         margin=dict(l=50, r=30, t=75, b=45),
#         template="plotly_white",
#         hovermode="closest",
#         legend=dict(
#             orientation="h",
#             yanchor="bottom",
#             y=1.02,
#             xanchor="left",
#             x=0,
#         ),
#     )

#     return fig


def create_figure(summary, selected_modes, selected_models):

    modes = [
        mode for mode in MODE_ORDER
        if mode in selected_modes
    ]

    modes += [
        mode for mode in sorted(selected_modes, key=natural_key)
        if mode not in modes
    ]

    models = sorted(selected_models, key=natural_key)

    if not modes or not models or summary.empty:
        return None

    # --------------------------------------------------
    # SUBPLOT LAYOUT
    # --------------------------------------------------

    if len(modes) == 5:

        # Six-column grid.
        # Each plot occupies two columns.
        #
        # Top:       1       2       3
        # Bottom:       4       5

        specs = [
            [
                {"colspan": 2}, None,
                {"colspan": 2}, None,
                {"colspan": 2}, None
            ],
            [
                None,
                {"colspan": 2}, None,
                {"colspan": 2}, None,
                None
            ]
        ]

        positions = [
            (1, 1),
            (1, 3),
            (1, 5),
            (2, 2),
            (2, 4),
        ]

        fig = make_subplots(
            rows=2,
            cols=6,
            specs=specs,
            subplot_titles=[
                get_mode_label(mode)
                for mode in modes
            ],
            horizontal_spacing=0.025,
            vertical_spacing=0.20,
        )

        nrows = 2

    else:

        ncols = min(3, len(modes))
        nrows = math.ceil(len(modes) / ncols)

        positions = [
            (i // ncols + 1, i % ncols + 1)
            for i in range(len(modes))
        ]

        fig = make_subplots(
            rows=nrows,
            cols=ncols,
            subplot_titles=[
                get_mode_label(mode)
                for mode in modes
            ],
            horizontal_spacing=0.10,
            vertical_spacing=0.16,
        )

    # --------------------------------------------------
    # MODEL COLORS
    # --------------------------------------------------

    palette = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
        "#bcbd22",
        "#17becf",
    ]

    colors = {
        model: palette[i % len(palette)]
        for i, model in enumerate(models)
    }

    # --------------------------------------------------
    # COMMON Y AXIS RANGE
    # --------------------------------------------------

    visible_data = summary[
        summary["mode"].isin(modes)
        & summary["model"].isin(models)
    ]

    if visible_data.empty:
        return None

    upper_values = (
        visible_data["mean_error_rate"]
        + visible_data["std_deviation"].fillna(0)
    )

    y_max = upper_values.max()

    # Add some space above the highest error bar.
    y_max = max(y_max * 1.10, 0.01)

    # --------------------------------------------------
    # ADD SUBPLOTS
    # --------------------------------------------------

    for i, mode in enumerate(modes):

        row, col = positions[i]

        mode_data = summary[
            summary["mode"] == mode
        ]

        for model in models:

            model_data = (
                mode_data[
                    mode_data["model"] == model
                ]
                .sort_values("n_sensors")
            )

            if model_data.empty:
                continue

            yerr = model_data[
                "std_deviation"
            ].fillna(0)

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

                    name=get_model_label(model),

                    legendgroup=model,

                    showlegend=(i == 0),

                    line=dict(
                        color=colors[model],
                        width=2.5,
                    ),

                    marker=dict(
                        size=7,
                    ),

                    hovertemplate=(
                        f"Model: {get_model_label(model)}"
                        "<br>Sensors: %{x}"
                        "<br>Mean error rate: %{y:.4f}"
                        "<extra></extra>"
                    ),
                ),
                row=row,
                col=col,
            )

        # X AXIS
        fig.update_xaxes(
            title_text="Active sensors",
            dtick=1,
            showgrid=True,
            gridcolor="rgba(128,128,128,0.15)",
            row=row,
            col=col,
        )

        # Y AXIS: SAME RANGE FOR EVERY SUBPLOT
        fig.update_yaxes(
            title_text="Mean error rate",
            range=[0, y_max],
            autorange=False,
            showgrid=True,
            gridcolor="rgba(128,128,128,0.2)",
            zeroline=True,
            row=row,
            col=col,
        )

    # --------------------------------------------------
    # GLOBAL LAYOUT
    # --------------------------------------------------

    fig.update_layout(
        title=dict(
            text="Spatial Reasoning",
            x=0.5,
            xanchor="center",
        ),

        height=750 if len(modes) == 5 else 350 * nrows,

        autosize=True,

        margin=dict(
            l=65,
            r=40,
            t=110,
            b=65,
        ),

        template="plotly_white",

        hovermode="closest",

        font=dict(
            family="Arial",
            size=12,
        ),

        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="center",
            x=0.5,
            font=dict(size=12),
        ),
    )

    # Improve subplot title appearance.
    fig.update_annotations(
        font=dict(
            size=14,
            family="Arial",
        )
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
        subplot_titles=[
            get_model_label(model)
            for model in models
        ],
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
            get_mode_label(mode)
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

    filtered = compute_metrics(filtered, selected_errors)

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

        # Generate Overleaf LaTeX code
        latex_code = comparison_table_to_latex(
            table=pivot,
            caption="Normalized error results by number of sensors and LLM.",
            label=f"tab:normalized_error_results_{key_suffix}",
        )

        st.download_button(
            label="Download comparison table (LaTeX)",
            data=latex_code.encode("utf-8"),
            file_name=f"normalized_error_results_{key_suffix}.tex",
            mime="text/plain",
            key=f"download_pivot_latex_{key_suffix}",
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
    # dipendendo dalle sottocategorie selezionate.
    heatmap_data = build_heatmap_data(
        filtered,
        selected_errors
    )

    prevalence, distribution = compute_heatmap_metrics(
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
                title="Error prevalence",
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
                        width=1600,
                        height=500,
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



            render_error_comparison_tables(
                data=filtered,
                selected_errors=selected_errors,
                selected_modes=selected_modes,
                selected_models=selected_models,
                key_suffix=f"prevalence_{key_suffix}",
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
                title="Error distribution",
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
                        width=1600,
                        height=500,
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
                
            render_error_comparison_tables(
                data=filtered,
                selected_errors=selected_errors,
                selected_modes=selected_modes,
                selected_models=selected_models,
                key_suffix=f"distribution_{key_suffix}",
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
