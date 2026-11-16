
import re
from pathlib import Path

# ==================================================
# 1. INPUT FILES
# ==================================================

SCRIPT_DIR = Path(__file__).resolve().parent

INPUT_FILES = {
    ("jaccard", "deterministic"):
        SCRIPT_DIR / "jaccard_similarity_overview_deterministic.tex",

    ("jaccard", "uncertain"):
        SCRIPT_DIR / "jaccard_similarity_overview_uncertain.tex",

    ("intervention", "deterministic"):
        SCRIPT_DIR / "human_intervention_rate_overview_deterministic.tex",

    ("intervention", "uncertain"):
        SCRIPT_DIR / "human_intervention_rate_overview_uncertain.tex",
}

OUTPUT_FILES = {
    "jaccard": SCRIPT_DIR / "final_jaccard_similarity.tex",
    "intervention": SCRIPT_DIR / "final_human_intervention_rate.tex",
}


# ==================================================
# 2. CONFIGURATION
# ==================================================

NS_VALUES = [1, 2, 3, 4]

SCENARIOS = ["deterministic", "uncertain"]

JACCARD_METHODS = [
    "AUTO-ILP-B120",
    "AUTO-ILP-B30",
    "AUTO-ILP-B60",
    "HITL-AUTO",
    "HITL-ILP-B120",
    "HITL-ILP-B30",
    "HITL-ILP-B60",
    "ILP-B30-ILP-B120",
    "ILP-B30-ILP-B60",
    "ILP-B60-ILP-B120",
]


# ==================================================
# 3. COMMON FUNCTIONS
# ==================================================

def format_value(value):
    """
    Format mean and standard deviation to 2 decimal places.
    Example: 0.495 ± 0.251 -> 0.50 ± 0.25
    """

    match = re.search(
        r"([-+]?\d*\.?\d+)\s*±\s*([-+]?\d*\.?\d+)",
        value
    )

    if match:
        mean = float(match.group(1))
        std = float(match.group(2))

        return f"{mean:.2f} ± {std:.2f}"

    return value.strip()


def extract_tabular(content, filename):
    """
    Extract the body of the first tabular environment.
    """

    match = re.search(
        r"\\begin\{tabular\}\{[^}]*\}(.*?)\\end\{tabular\}",
        content,
        re.DOTALL
    )

    if not match:
        raise ValueError(
            f"No tabular environment found in {filename}"
        )

    return match.group(1)


def read_file(filename):
    path = Path(filename)

    if not path.exists():
        raise FileNotFoundError(
            f"File not found: {path.resolve()}"
        )

    return path.read_text(encoding="utf-8")


def split_rows(body):
    """
    Split LaTeX tabular body into rows.
    """

    rows = re.split(r"\\\\", body)

    clean_rows = []

    for row in rows:

        row = re.sub(
            r"\\(toprule|midrule|bottomrule|hline)\b",
            "",
            row
        )

        row = row.strip()

        if "&" in row:
            clean_rows.append(row)

    return clean_rows


# ==================================================
# 4. PARSE JACCARD SIMILARITY TABLE
# ==================================================

def parse_jaccard_file(filename):

    content = read_file(filename)

    body = extract_tabular(content, filename)

    rows = split_rows(body)

    results = {}

    for row in rows:

        cells = [
            cell.strip()
            for cell in row.split("&")
        ]

        # First cell must contain Ns
        ns_match = re.match(r"^\s*(\d+)\s*$", cells[0])

        if not ns_match:
            continue

        ns = int(ns_match.group(1))

        if len(cells) < len(JACCARD_METHODS) + 1:
            raise ValueError(
                f"Invalid number of columns in {filename}, Ns={ns}"
            )

        values = {}

        for method, value in zip(
            JACCARD_METHODS,
            cells[1:len(JACCARD_METHODS) + 1]
        ):

            value = value.split("%")[0].strip()

            values[method] = format_value(value)

        results[ns] = values

    if not results:
        raise ValueError(
            f"No Jaccard data extracted from {filename}"
        )

    return results


# ==================================================
# 5. PARSE HUMAN INTERVENTION RATE TABLE
# ==================================================

def parse_intervention_file(filename):

    content = read_file(filename)

    body = extract_tabular(content, filename)

    rows = split_rows(body)

    results = {}

    for row in rows:

        cells = [
            cell.strip()
            for cell in row.split("&")
        ]

        # First cell must contain Ns
        ns_match = re.match(r"^\s*(\d+)\s*$", cells[0])

        if not ns_match:
            continue

        ns = int(ns_match.group(1))

        if len(cells) < 3:
            raise ValueError(
                f"Invalid number of columns in {filename}, Ns={ns}"
            )

        mean_std = cells[1].split("%")[0].strip()
        observations = cells[2].split("%")[0].strip()

        results[ns] = {
            "mean_std": format_value(mean_std),
            "observations": observations
        }

    if not results:
        raise ValueError(
            f"No intervention data extracted from {filename}"
        )

    return results


# ==================================================
# 6. LOAD ALL FOUR TABLES
# ==================================================

jaccard_data = {}
intervention_data = {}

print("\nREADING SOURCE FILES")
print("=" * 60)

for scenario in SCENARIOS:

    # Jaccard
    filename = INPUT_FILES[("jaccard", scenario)]

    print(f"Reading: {filename}")

    jaccard_data[scenario] = parse_jaccard_file(filename)

    print(
        f"  Extracted Ns: "
        f"{list(jaccard_data[scenario].keys())}"
    )

    # Human intervention
    filename = INPUT_FILES[("intervention", scenario)]

    print(f"Reading: {filename}")

    intervention_data[scenario] = parse_intervention_file(
        filename
    )

    print(
        f"  Extracted Ns: "
        f"{list(intervention_data[scenario].keys())}"
    )


# ==================================================
# 7. GENERATE JACCARD SIMILARITY TABLE
# ==================================================

def get_jaccard_value(scenario, ns, method):

    return (
        jaccard_data
        .get(scenario, {})
        .get(ns, {})
        .get(method, "")
    )



# ==================================================
# 7. GENERATE JACCARD SIMILARITY TABLE
# ==================================================

def generate_jaccard_table():

    lines = []

    lines.append(r"\begin{table*}[t]")
    lines.append(r"\centering")
    lines.append(r"\fontsize{6pt}{7.2pt}\selectfont ")
    lines.append(r"\setlength{\tabcolsep}{2pt}")

    lines.append(
        r"\begin{tabular}{cl|cccccccccc}"
    )

    lines.append(r"\toprule")

    # Single header row
    bold_methods = [
        rf"\textbf{{{method}}}"
        for method in JACCARD_METHODS
    ]

    lines.append(
        r"$\boldsymbol{N_s}$ & \textbf{Scenario} & "
        + " & ".join(bold_methods)
        + r" \\"
    )

    lines.append(r"\midrule")

    # Data rows
    for ns in NS_VALUES:

        for scenario in SCENARIOS:

            row = []

            # Ns column
            if scenario == SCENARIOS[0]:
                row.append(rf"\multirow{{2}}{{*}}{{{ns}}}")
            else:
                row.append("")

            # Scenario column
            if scenario == "deterministic":
                row.append("Deterministic")
            else:
                row.append("Uncertain")

            # Jaccard values
            for method in JACCARD_METHODS:

                value = get_jaccard_value(
                    scenario, ns, method
                )

                row.append(value)

            lines.append(" & ".join(row) + r" \\")

        if ns != NS_VALUES[-1]:
            lines.append(r"\midrule")

    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")

    lines.append(
        r"\caption{Jaccard similarity between the different "
        r"methods for deterministic and uncertain scenarios, "
        r"across different numbers of sensors.}"
    )

    lines.append(r"\label{tab:jaccard_similarity}")
    lines.append(r"\end{table*}")

    return "\n".join(lines)


# ==================================================
# 8. GENERATE HUMAN INTERVENTION RATE TABLE
# ==================================================

def get_intervention_value(scenario, ns):

    return (
        intervention_data
        .get(scenario, {})
        .get(ns, {})
        .get("mean_std", "")
    )


def generate_intervention_table():

    lines = []

    lines.append(r"\begin{table}[t]")
    lines.append(r"\centering")
    lines.append(r"\small")

    lines.append(
        r"\begin{tabular}{c|cc}"
    )

    lines.append(r"\toprule")

    lines.append(
        r"$\boldsymbol{N_s}$"
        r" & \textbf{Deterministic}"
        r" & \textbf{Uncertain} \\"
    )

    lines.append(r"\midrule")

    for ns in NS_VALUES:

        det_value = get_intervention_value(
            "deterministic", ns
        )

        unc_value = get_intervention_value(
            "uncertain", ns
        )

        row = [
            str(ns),
            det_value,
            unc_value
        ]

        lines.append(" & ".join(row) + r" \\")

    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")

    lines.append(
        r"\caption{Human intervention rate (HIR) for "
        r"deterministic and uncertain scenarios.}"
    )

    lines.append(r"\label{tab:human_intervention_rate}")
    lines.append(r"\end{table}")

    return "\n".join(lines)



# ==================================================
# 9. SAVE AND PRINT FINAL TABLES
# ==================================================

final_jaccard = generate_jaccard_table()

final_intervention = generate_intervention_table()


# Save Jaccard table
OUTPUT_FILES["jaccard"].write_text(
    final_jaccard,
    encoding="utf-8"
)

# Save intervention table
OUTPUT_FILES["intervention"].write_text(
    final_intervention,
    encoding="utf-8"
)


print("\n" + "=" * 60)
print("FINAL JACCARD SIMILARITY TABLE")
print("=" * 60)

print(final_jaccard)

print("\n" + "=" * 60)
print("FINAL HUMAN INTERVENTION RATE TABLE")
print("=" * 60)

print(final_intervention)


print("\n" + "=" * 60)
print("OUTPUT FILES")
print("=" * 60)

for filename in OUTPUT_FILES.values():
    print(f"Saved: {filename}")
