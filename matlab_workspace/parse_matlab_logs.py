"""
Parse MATLAB Command Window printouts of the intruder-detection simulation
into CSV files.

Usage:
    python parse_matlab_logs.py log1.txt log2.txt ... --output_directory parsed_csv

Outputs (one set of CSV files covering all input logs; every row carries the
source file name):
    tests.csv            one row per test
    monitored_areas.csv  one row per (test, area) for "Areas monitored:" lines
    assignments.csv      one row per (test, iteration, robot)
    summary_tasks.csv    final Task / Trials / Successes table
    success_rates.csv    final success_rate_* values
    dropped_rows.csv     identifiers of rows dropped because of missing values
"""

import argparse
import re
from pathlib import Path

import pandas as pd

# ---------------------------------------------------------------- patterns
NUMBER = r"[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|NaN|Inf)"

HTML_TAG_PATTERN = re.compile(r"</?strong>")
SIMULATION_PATTERN = re.compile(r"Simulation in environment (\d+), mode (\d+)")
TRIAL_PATTERN = re.compile(r"TRIAL (\d+) / (\d+)")
TEST_PATTERN = re.compile(r"^TEST (\d+) / (\d+)")
SENSORS_PATTERN = re.compile(r"^Tested sensors \[(.*)\]")
ITERATION_PATTERN = re.compile(r"^Iteration (\d+) / (\d+)")
ASSIGNMENT_ROW_PATTERN = re.compile(
    rf"^\s*(\d+)\s+\{{'(\w+)'\}}\s+({NUMBER})\s+({NUMBER})\s+({NUMBER})\s*$"
)
SUCCESS_PATTERN = re.compile(r"^Success:")
FALSE_POSITIVE_PATTERN = re.compile(r"^False positive:")
INTRUDER_PATTERN = re.compile(r"^Intruder position:\s*(\S+)")
AREAS_MONITORED_PATTERN = re.compile(r"^Areas monitored:\s*(.*)$")
SUMMARY_HEADER_PATTERN = re.compile(r"^\s*Task\s+Trials\s+Successes")
SUMMARY_ROW_PATTERN = re.compile(r"^\s*(\d+)\s+(\d+)\s+(\d+)\s*$")
SUCCESS_RATE_NAME_PATTERN = re.compile(r"^(success_rate_\w+)\s*=\s*$")
SUCCESS_RATE_VALUE_PATTERN = re.compile(rf"^\s*({NUMBER})\s*$")


# ---------------------------------------------------------------- helpers
def read_clean_lines(log_path):
    """Read the log and remove MATLAB's <strong> display tags."""
    raw_text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    clean_text = HTML_TAG_PATTERN.sub("", raw_text)
    return clean_text.splitlines()


def new_test_record(source_file, environment, mode, trial, test_number, total_tests):
    return {
        "source_file": source_file,
        "environment": environment,
        "mode": mode,
        "trial": trial,
        "test": test_number,
        "total_tests": total_tests,
        "tested_sensors": None,
        "n_sensors": None,
        "last_iteration": None,
        "max_iterations": None,
        "outcome": None,
        "intruder_position": None,
    }


def parse_log(log_path):
    """Walk the log line by line and collect all records."""
    source_file = Path(log_path).name
    lines = read_clean_lines(log_path)

    test_records = []
    monitored_area_records = []
    assignment_records = []
    summary_task_records = []
    success_rate_records = []

    environment = None
    mode = None
    trial = None
    current_test = None
    current_iteration = None
    inside_summary_table = False
    pending_success_rate_name = None

    for line in lines:
        # success_rate value is printed on a later line than its name
        if pending_success_rate_name is not None:
            value_match = SUCCESS_RATE_VALUE_PATTERN.match(line)
            if value_match:
                success_rate_records.append({
                    "source_file": source_file,
                    "name": pending_success_rate_name,
                    "value": float(value_match.group(1)),
                })
                pending_success_rate_name = None
            continue

        simulation_match = SIMULATION_PATTERN.search(line)
        if simulation_match:
            environment = int(simulation_match.group(1))
            mode = int(simulation_match.group(2))
            continue

        trial_match = TRIAL_PATTERN.search(line)
        if trial_match:
            trial = int(trial_match.group(1))
            continue

        test_match = TEST_PATTERN.match(line)
        if test_match:
            current_test = new_test_record(
                source_file, environment, mode, trial,
                int(test_match.group(1)), int(test_match.group(2)),
            )
            test_records.append(current_test)
            current_iteration = None
            continue

        sensors_match = SENSORS_PATTERN.match(line)
        if sensors_match and current_test is not None:
            sensor_names = [name.strip() for name in sensors_match.group(1).split(",") if name.strip()]
            current_test["tested_sensors"] = ";".join(sensor_names)
            current_test["n_sensors"] = len(sensor_names)
            continue

        iteration_match = ITERATION_PATTERN.match(line)
        if iteration_match and current_test is not None:
            current_iteration = int(iteration_match.group(1))
            current_test["last_iteration"] = current_iteration
            current_test["max_iterations"] = int(iteration_match.group(2))
            continue

        assignment_match = ASSIGNMENT_ROW_PATTERN.match(line)
        if assignment_match and current_test is not None and not inside_summary_table:
            assignment_records.append({
                "source_file": source_file,
                "trial": current_test["trial"],
                "test": current_test["test"],
                "iteration": current_iteration,
                "robot": int(assignment_match.group(1)),
                "area": assignment_match.group(2),
                "distance": float(assignment_match.group(3)),
                "frequency": float(assignment_match.group(4)),
                "cost": float(assignment_match.group(5)),
            })
            continue

        if SUCCESS_PATTERN.match(line) and current_test is not None:
            current_test["outcome"] = "success"
            continue

        if FALSE_POSITIVE_PATTERN.match(line) and current_test is not None:
            current_test["outcome"] = "false_positive"
            continue

        intruder_match = INTRUDER_PATTERN.match(line)
        if intruder_match and current_test is not None:
            current_test["intruder_position"] = intruder_match.group(1)
            continue

        areas_monitored_match = AREAS_MONITORED_PATTERN.match(line)
        if areas_monitored_match and current_test is not None:
            area_names = [name.strip() for name in areas_monitored_match.group(1).split(",") if name.strip()]
            for area_order, area_name in enumerate(area_names, start=1):
                monitored_area_records.append({
                    "source_file": source_file,
                    "trial": current_test["trial"],
                    "test": current_test["test"],
                    "area_order": area_order,
                    "area": area_name,
                })
            continue

        if SUMMARY_HEADER_PATTERN.match(line):
            inside_summary_table = True
            current_test = None
            continue

        summary_row_match = SUMMARY_ROW_PATTERN.match(line)
        if summary_row_match and inside_summary_table:
            summary_task_records.append({
                "source_file": source_file,
                "task": int(summary_row_match.group(1)),
                "trials": int(summary_row_match.group(2)),
                "successes": int(summary_row_match.group(3)),
            })
            continue

        success_rate_name_match = SUCCESS_RATE_NAME_PATTERN.match(line)
        if success_rate_name_match:
            inside_summary_table = False
            pending_success_rate_name = success_rate_name_match.group(1)
            continue

    return {
        "tests": test_records,
        "monitored_areas": monitored_area_records,
        "assignments": assignment_records,
        "summary_tasks": summary_task_records,
        "success_rates": success_rate_records,
    }


def drop_incomplete_rows(table_name, data_frame, identifier_columns):
    """Drop rows with any missing value; return the cleaned frame and the dropped identifiers."""
    incomplete_mask = data_frame.isna().any(axis=1)
    dropped_identifiers = data_frame.loc[incomplete_mask, identifier_columns].copy()
    dropped_identifiers.insert(0, "table", table_name)
    missing_value_flags = data_frame.loc[incomplete_mask].isna()
    dropped_identifiers["missing_columns"] = [
        ";".join(missing_value_flags.columns[row_flags])
        for row_flags in missing_value_flags.to_numpy()
    ]
    return data_frame.loc[~incomplete_mask].reset_index(drop=True), dropped_identifiers


def main():
    argument_parser = argparse.ArgumentParser(description="Parse MATLAB simulation logs into CSV files.")
    argument_parser.add_argument("log_paths", nargs="+", help="MATLAB log .txt files")
    argument_parser.add_argument("--output_directory", default="parsed_csv")
    arguments = argument_parser.parse_args()

    output_directory = Path(arguments.output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)

    collected_records = {
        "tests": [], "monitored_areas": [], "assignments": [],
        "summary_tasks": [], "success_rates": [],
    }
    for log_path in arguments.log_paths:
        parsed_log = parse_log(log_path)
        for table_name, records in parsed_log.items():
            collected_records[table_name].extend(records)

    identifier_columns_by_table = {
        "tests": ["source_file", "trial", "test"],
        "monitored_areas": ["source_file", "trial", "test", "area_order"],
        "assignments": ["source_file", "trial", "test", "iteration", "robot"],
        "summary_tasks": ["source_file", "task"],
        "success_rates": ["source_file", "name"],
    }

    all_dropped_identifiers = []
    for table_name, records in collected_records.items():
        data_frame = pd.DataFrame(records)
        if data_frame.empty:
            print(f"{table_name}: no rows found")
            continue
        cleaned_frame, dropped_identifiers = drop_incomplete_rows(
            table_name, data_frame, identifier_columns_by_table[table_name]
        )
        cleaned_frame.to_csv(output_directory / f"{table_name}.csv", index=False)
        all_dropped_identifiers.append(dropped_identifiers)
        print(f"{table_name}: {len(cleaned_frame)} rows written, {len(dropped_identifiers)} dropped")

    dropped_frame = pd.concat(all_dropped_identifiers, ignore_index=True) if all_dropped_identifiers else pd.DataFrame()
    dropped_frame.to_csv(output_directory / "dropped_rows.csv", index=False)
    if not dropped_frame.empty:
        print("\nDropped rows (missing values):")
        print(dropped_frame.to_string(index=False))


if __name__ == "__main__":
    main()
