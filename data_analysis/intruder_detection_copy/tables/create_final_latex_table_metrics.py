
import re
from pathlib import Path

# ==================================================
# 1. INPUT FILES: SPECIFY EACH FILE EXPLICITLY
# ==================================================

SCRIPT_DIR = Path(__file__).resolve().parent


INPUT_FILES = {
    ("world_1", "deterministic", "success_rate"):
        SCRIPT_DIR / "success_rate_world_1_deterministic.tex",

    ("world_1", "deterministic", "eta"):
        SCRIPT_DIR / "inspection_efficiency_world_1_deterministic.tex",

    ("world_1", "uncertain", "success_rate"):
        SCRIPT_DIR / "success_rate_world_1_uncertain.tex",

    ("world_1", "uncertain", "eta"):
        SCRIPT_DIR / "inspection_efficiency_world_1_uncertain.tex",

    ("world_2", "deterministic", "success_rate"):
        SCRIPT_DIR / "success_rate_world_2_deterministic.tex",

    ("world_2", "deterministic", "eta"):
        SCRIPT_DIR / "inspection_efficiency_world_2_deterministic.tex",

    ("world_2", "uncertain", "success_rate"):
        SCRIPT_DIR / "success_rate_world_2_uncertain.tex",

    ("world_2", "uncertain", "eta"):
        SCRIPT_DIR / "inspection_efficiency_world_2_uncertain.tex",
}

OUTPUT_FILE = SCRIPT_DIR / "final_table_metrics.tex"


# ==================================================
# 2. CONFIGURATION
# ==================================================

WORLDS = ["world_1", "world_2"]
SCENARIOS = ["deterministic", "uncertain"]
METRICS = ["success_rate", "eta"]

NS_VALUES = [1, 2, 3, 4]

SOURCE_METHODS = [
    "HITL", "AUTO", "ILP-B30", "ILP-B60", "ILP-B120"
]

METHOD_MAP = {
    "HITL": "HITL",
    "AUTO": "LLM",
    "ILP-B30": "ILP-30",
    "ILP-B60": "ILP-60",
    "ILP-B120": "ILP-120",
}

METHODS = ["LLM", "HITL", "ILP-30", "ILP-60", "ILP-120"]


# ==================================================
# 3. PARSE SOURCE LATEX TABLE
# ==================================================

def format_value(value):
    """
    Round mean and standard deviation to 2 decimal places.
    Example: 1.000 ± 0.000 -> 1.00 ± 0.00
    """

    match = re.search(
        r"([-+]?\d*\.?\d+)\s*±\s*([-+]?\d*\.?\d+)",
        value
    )

    if match:
        mean = float(match.group(1))
        std = float(match.group(2))

        return f"{mean:.2f} ± {std:.2f}"

    return value

def parse_latex_file(filename):

    path = Path(filename)

    if not path.exists():
        raise FileNotFoundError(
            f"File not found: {path.resolve()}"
        )

    content = path.read_text(encoding="utf-8")

    # Extract tabular environment
    match = re.search(
        r"\\begin\{tabular\}.*?\}(.*?)\\end\{tabular\}",
        content,
        re.DOTALL
    )

    if not match:
        raise ValueError(
            f"No tabular environment found in {filename}"
        )

    body = match.group(1)

    # Split into LaTeX rows
    rows = re.split(r"\\\\", body)

    results = {}

    for row in rows:

        # Remove LaTeX rules
        row = re.sub(
            r"\\(toprule|midrule|bottomrule|hline)\b",
            "",
            row
        )

        row = row.strip()

        if "&" not in row:
            continue

        cells = [cell.strip() for cell in row.split("&")]

        # First cell must contain Ns
        ns_match = re.match(r"^\s*(\d+)\s*$", cells[0])

        if not ns_match:
            continue

        ns = int(ns_match.group(1))

        if len(cells) < 6:
            raise ValueError(
                f"Invalid number of columns in {filename}, Ns={ns}"
            )

        values = {}

        for source_method, value in zip(
            SOURCE_METHODS, cells[1:6]
        ):

            # Remove trailing whitespace and comments
            value = value.split("%")[0].strip()
            value = format_value(value)

            values[METHOD_MAP[source_method]] = value

        results[ns] = values

    if not results:
        raise ValueError(
            f"No data rows extracted from {filename}"
        )

    return results


# ==================================================
# 4. LOAD ALL EIGHT TABLES
# ==================================================

data = {}

print("\nREADING SOURCE FILES")
print("=" * 60)

for key, filename in INPUT_FILES.items():

    print(f"Reading: {filename}")

    parsed = parse_latex_file(filename)

    data[key] = parsed

    print(f"  Extracted Ns: {list(parsed.keys())}")


# ==================================================
# 5. RETRIEVE VALUES
# ==================================================

def get_value(world, scenario, metric, ns, method):

    key = (world, scenario, metric)

    if key not in data:
        return ""

    return data[key].get(ns, {}).get(method, "")


def extract_mean(value):
    """
    Extract the mean from values such as:
    '1.000 ± 0.000'
    '0.850'
    """

    if not value:
        return None

    # Remove LaTeX formatting if present
    value = re.sub(r"\\cellcolor\{[^}]*\}", "", value)
    value = re.sub(r"\\textbf\{(.*?)\}", r"\1", value)

    # Accept both ± and +/-
    match = re.search(
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)",
        value
    )

    if match:
        return float(match.group(1))

    return None


def get_best_method(world, scenario, metric, ns):
    """
    Return the best method only if HITL is among
    the methods with the highest mean.

    If there is a tie:
    - HITL is bold if it shares the maximum.
    - Otherwise, no method is bold.
    """

    values = {}

    for method in METHODS:

        value = get_value(
            world, scenario, metric, ns, method
        )

        mean = extract_mean(value)

        if mean is not None:
            values[method] = mean

    if not values:
        return None

    best_value = max(values.values())

    best_methods = {
        method
        for method, value in values.items()
        if value == best_value
    }

    if "HITL" in best_methods:
        return {"HITL"}

    # Bold only HITL in case of a tie;
    # otherwise, no value is bold.
    return set()
# ==================================================
# 6. GENERATE FINAL TABLE
# ==================================================

def generate_table():

    lines = []

    lines.append(r"\setlength{\tabcolsep}{2.4pt}")
    lines.append(r"\begin{table*}[t]")
    lines.append(r"\centering")
    lines.append(r"\scriptsize")

    lines.append(
        r"\begin{tabular}{cl|cc|cc|cc|cc}"
    )

    lines.append(r"\toprule")

    lines.append(
        r"\multirow{3}{*}{\textbf{$\boldsymbol{N_s}$}}"
        r" & \multirow{3}{*}{\textbf{Method}}"
        r" & \multicolumn{4}{c|}{\textbf{World 1}}"
        r" & \multicolumn{4}{c}{\textbf{World 2}} \\"
    )

    lines.append(
        r"\cmidrule(lr){3-6} \cmidrule(lr){7-10}"
    )

    lines.append(
        r" & & \multicolumn{2}{c|}{\textbf{Deterministic}}"
        r" & \multicolumn{2}{c|}{\textbf{Uncertain}}"
        r" & \multicolumn{2}{c|}{\textbf{Deterministic}}"
        r" & \multicolumn{2}{c}{\textbf{Uncertain}} \\"
    )

    lines.append(
        r"\cmidrule(lr){3-4} \cmidrule(lr){5-6}"
        r" \cmidrule(lr){7-8} \cmidrule(lr){9-10}"
    )

    metric_headers = [
        r"$\mathbf{\overline{SR}\uparrow}$",
        r"$\mathbf{\overline{\eta}\uparrow}$"
    ]

    lines.append(
        " & & " + " & ".join(metric_headers * 4) + r" \\"
    )

    lines.append(r"\midrule")

    # Generate data rows
    for ns in NS_VALUES:

        for method in METHODS:

            row = []

            # Ns column
            if method == METHODS[0]:
                row.append(rf"\multirow{{5}}{{*}}{{{ns}}}")
            else:
                row.append("")

            # Method column
            if method == "HITL":
                row.append(r"\cellcolor{green!20}HITL")
            else:
                row.append(method)
            
            # Eight metric values
            for world in WORLDS:

                for scenario in SCENARIOS:

                    for metric in METRICS:

                        value = get_value(
                            world,
                            scenario,
                            metric,
                            ns,
                            method
                        )

                        # Find the best method for this combination
                        best_methods = get_best_method(
                            world, scenario, metric, ns
                        )

                        # Highlight HITL background
                        if method == "HITL":
                            value = (
                                rf"\cellcolor{{green!20}}{value}"
                            )

                        # Bold the highest mean value
                        if best_methods and method in best_methods:
                            value = rf"\textbf{{{value}}}"

                        row.append(value)


            lines.append(" & ".join(row) + r" \\")

        if ns != NS_VALUES[-1]:
            lines.append(r"\hline")

    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")

    lines.append(
        r"\caption{Performance metrics for deterministic "
        r"and uncertain scenarios across the two tested worlds.}"
    )

    lines.append(r"\label{tab:metrics}")
    lines.append(r"\end{table*}")

    return "\n".join(lines)


# ==================================================
# 7. SAVE AND PRINT
# ==================================================

latex_table = generate_table()

Path(OUTPUT_FILE).write_text(
    latex_table,
    encoding="utf-8"
)

print("\n" + "=" * 60)
print("FINAL LATEX TABLE")
print("=" * 60)

print(latex_table)

print(f"\nTable saved to: {OUTPUT_FILE}")
