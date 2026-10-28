"""
Parse MATLAB Command Window printouts of the intruder-detection simulation
into ONE CSV file, with one block per log file.

Usage (from the folder containing this script):
    python parse_matlab_logs_v2.py LOGS_FOLDER/matlab_log_env2_*.txt --output_file results.csv

Block layout for each log file:
    Source file | <name> | Mode | <mode> | B | <B or "not in log">
    Mode | Test | Trial 1 |  |  | Trial 2 |  |  | ... | Total Successes | SuccessRate
         |      | Nr of iterations | Area visited | Success | ...
    <one row per test>
    <empty row>

Definitions:
    Mode            "Logical" for the first half of the tests, "With uncertainty" for the rest
    Nr of iterations  last printed iteration; if the number of iterations that produced
                    assignments differs, it is added in brackets: "2 (1)"
    Area visited    assigned areas in printed order (iteration 1 robot 1, robot 2, iteration 2 ...),
                    followed by the "Areas monitored:" areas when they are printed
    Success         1 if "Success:" was printed, or if no intruder was identified and the
                    intruder position is "-"; 0 otherwise
    Total Successes sum of Success over all trials of the test
    SuccessRate     sum of Success over all tests with the same Mode and number of sensors,
                    divided by (number of those tests x number of trials)
"""

import argparse
import csv
import re
from pathlib import Path

# ---------------------------------------------------------------- patterns
NUMBER = r"[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|NaN|Inf)"

HTML_TAG_PATTERN = re.compile(r"</?strong>")
SIMULATION_PATTERN = re.compile(
    rf"Simulation in environment (\d+), mode (\d+)(?:, B = ({NUMBER}))?"
)
TRIAL_PATTERN = re.compile(r"TRIAL (\d+) / (\d+)")
TEST_PATTERN = re.compile(r"^TEST (\d+) / (\d+)")
SENSORS_PATTERN = re.compile(r"^Tested sensors \[(.*)\]")
ITERATION_PATTERN = re.compile(r"^Iteration (\d+) / (\d+)")
ASSIGNMENT_ROW_PATTERN = re.compile(
    rf"^\s*(\d+)\s+\{{'(\w+)'\}}\s+({NUMBER})\s+({NUMBER})\s+({NUMBER})\s*$"
)
SUCCESS_PATTERN = re.compile(r"^Success:")
NO_INTRUDER_IDENTIFIED_PATTERN = re.compile(r"no intruder identified")
INTRUDER_PATTERN = re.compile(r"^Intruder position:\s*(\S+)")
AREAS_MONITORED_PATTERN = re.compile(r"^Areas monitored:\s*(.*)$")
SUMMARY_HEADER_PATTERN = re.compile(r"^\s*Task\s+Trials\s+Successes")

NO_INTRUDER_SYMBOL = "-"
SEQUENCE_SEPARATOR = ", "


# ---------------------------------------------------------------- parsing
def read_clean_lines(log_path):
    """Read the log and remove MATLAB's <strong> display tags."""
    raw_text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    return HTML_TAG_PATTERN.sub("", raw_text).splitlines()


def new_run_record(trial, test_number, total_tests):
    """One run = one test in one trial."""
    return {
        "trial": trial,
        "test": test_number,
        "total_tests": total_tests,
        "n_sensors": None,
        "last_printed_iteration": None,
        "assignment_iterations": set(),
        "assigned_areas": [],
        "monitored_areas": [],
        "success_printed": False,
        "no_intruder_identified": False,
        "intruder_position": None,
    }


def parse_log(log_path):
    """Walk the log line by line; return file-level information and one record per run."""
    file_information = {"mode": None, "b_value": None, "n_trials": None}
    run_records = []
    trial = None
    current_run = None
    current_iteration = None

    for line in read_clean_lines(log_path):
        simulation_match = SIMULATION_PATTERN.search(line)
        if simulation_match:
            file_information["mode"] = simulation_match.group(2)
            file_information["b_value"] = simulation_match.group(3)  # None in older logs
            continue

        trial_match = TRIAL_PATTERN.search(line)
        if trial_match:
            trial = int(trial_match.group(1))
            file_information["n_trials"] = int(trial_match.group(2))
            continue

        if SUMMARY_HEADER_PATTERN.match(line):
            current_run = None  # stop attributing lines to the last test
            continue

        test_match = TEST_PATTERN.match(line)
        if test_match:
            current_run = new_run_record(trial, int(test_match.group(1)), int(test_match.group(2)))
            run_records.append(current_run)
            current_iteration = None
            continue

        if current_run is None:
            continue

        sensors_match = SENSORS_PATTERN.match(line)
        if sensors_match:
            sensor_names = [name for name in sensors_match.group(1).split(",") if name.strip()]
            current_run["n_sensors"] = len(sensor_names)
            continue

        iteration_match = ITERATION_PATTERN.match(line)
        if iteration_match:
            current_iteration = int(iteration_match.group(1))
            current_run["last_printed_iteration"] = current_iteration
            continue

        assignment_match = ASSIGNMENT_ROW_PATTERN.match(line)
        if assignment_match:
            current_run["assignment_iterations"].add(current_iteration)
            current_run["assigned_areas"].append(assignment_match.group(2))
            continue

        if SUCCESS_PATTERN.match(line):
            current_run["success_printed"] = True
            continue

        if NO_INTRUDER_IDENTIFIED_PATTERN.search(line):
            current_run["no_intruder_identified"] = True
            continue

        intruder_match = INTRUDER_PATTERN.match(line)
        if intruder_match:
            current_run["intruder_position"] = intruder_match.group(1)
            continue

        areas_monitored_match = AREAS_MONITORED_PATTERN.match(line)
        if areas_monitored_match:
            current_run["monitored_areas"] = [
                name.strip() for name in areas_monitored_match.group(1).split(",") if name.strip()
            ]
            continue

    return file_information, run_records


# ---------------------------------------------------------------- per-run values
def run_success(run_record):
    if run_record["success_printed"]:
        return 1
    if run_record["no_intruder_identified"] and run_record["intruder_position"] == NO_INTRUDER_SYMBOL:
        return 1
    return 0


def format_iterations(run_record):
    printed_iterations = run_record["last_printed_iteration"]
    iterations_with_assignments = len(run_record["assignment_iterations"])
    if printed_iterations == iterations_with_assignments:
        return str(printed_iterations)
    return f"{printed_iterations} ({iterations_with_assignments})"


def format_areas_visited(run_record):
    return SEQUENCE_SEPARATOR.join(run_record["assigned_areas"] + run_record["monitored_areas"])


def mode_label(test_number, total_tests):
    return "Logical" if test_number <= total_tests / 2 else "With uncertainty"


def run_is_complete(run_record):
    return (
        run_record["n_sensors"] is not None
        and run_record["last_printed_iteration"] is not None
        and run_record["intruder_position"] is not None
    )


# ---------------------------------------------------------------- table building
def build_test_rows(file_information, run_records):
    """Return (rows, dropped_tests). A test is dropped if any of its runs is missing or incomplete."""
    n_trials = file_information["n_trials"]
    runs_by_test = {}
    for run_record in run_records:
        runs_by_test.setdefault(run_record["test"], {})[run_record["trial"]] = run_record

    complete_tests = {}
    dropped_tests = []
    for test_number, runs_by_trial in sorted(runs_by_test.items()):
        missing_trials = [trial for trial in range(1, n_trials + 1) if trial not in runs_by_trial]
        incomplete_trials = [trial for trial, run in runs_by_trial.items() if not run_is_complete(run)]
        if missing_trials or incomplete_trials:
            dropped_tests.append((test_number, missing_trials, incomplete_trials))
            continue
        complete_tests[test_number] = runs_by_trial

    # success counts per (mode, number of sensors) for SuccessRate
    successes_by_group = {}
    runs_by_group = {}
    for test_number, runs_by_trial in complete_tests.items():
        first_run = runs_by_trial[1]
        group_key = (mode_label(test_number, first_run["total_tests"]), first_run["n_sensors"])
        successes_by_group[group_key] = successes_by_group.get(group_key, 0) + sum(
            run_success(run) for run in runs_by_trial.values()
        )
        runs_by_group[group_key] = runs_by_group.get(group_key, 0) + len(runs_by_trial)

    rows = []
    groups_with_rate_written = set()
    for test_number, runs_by_trial in complete_tests.items():
        first_run = runs_by_trial[1]
        test_mode = mode_label(test_number, first_run["total_tests"])
        group_key = (test_mode, first_run["n_sensors"])
        row = [test_mode, test_number]
        for trial in range(1, n_trials + 1):
            run_record = runs_by_trial[trial]
            row += [format_iterations(run_record), format_areas_visited(run_record), run_success(run_record)]
        total_successes = sum(run_success(run) for run in runs_by_trial.values())
        if group_key in groups_with_rate_written:
            success_rate_cell = ""
        else:
            success_rate = successes_by_group[group_key] / runs_by_group[group_key]
            success_rate_cell = round(success_rate, 4)
            groups_with_rate_written.add(group_key)
        row += [total_successes, success_rate_cell]
        rows.append(row)
    return rows, dropped_tests


def build_header_rows(source_file, file_information):
    n_trials = file_information["n_trials"]
    b_value = file_information["b_value"] if file_information["b_value"] is not None else "not in log"
    title_row = ["Source file", source_file, "Mode", file_information["mode"], "B", b_value]
    trial_row = ["Mode", "Test"]
    column_row = ["", ""]
    for trial in range(1, n_trials + 1):
        trial_row += [f"Trial {trial}", "", ""]
        column_row += ["Nr of iterations", "Area visited", "Success"]
    trial_row += ["Total Successes", "SuccessRate"]
    column_row += ["", ""]
    return [title_row, trial_row, column_row]


# ---------------------------------------------------------------- main
def main():
    argument_parser = argparse.ArgumentParser(description="Parse MATLAB simulation logs into one CSV.")
    argument_parser.add_argument("log_paths", nargs="+", help="MATLAB log .txt files")
    argument_parser.add_argument("--output_file", default="results.csv")
    arguments = argument_parser.parse_args()

    all_dropped_tests = []
    with open(arguments.output_file, "w", newline="", encoding="utf-8") as output_handle:
        csv_writer = csv.writer(output_handle)
        for log_path in sorted(arguments.log_paths):
            source_file = Path(log_path).name
            file_information, run_records = parse_log(log_path)
            if file_information["n_trials"] is None:
                print(f"{source_file}: no 'TRIAL k / N' line found, file skipped")
                continue
            test_rows, dropped_tests = build_test_rows(file_information, run_records)
            csv_writer.writerows(build_header_rows(source_file, file_information))
            csv_writer.writerows(test_rows)
            csv_writer.writerow([])
            print(f"{source_file}: {file_information['n_trials']} trials, "
                  f"{len(test_rows)} tests written, {len(dropped_tests)} dropped")
            for test_number, missing_trials, incomplete_trials in dropped_tests:
                all_dropped_tests.append((source_file, test_number, missing_trials, incomplete_trials))

    if all_dropped_tests:
        print("\nDropped tests (source file, test, missing trials, incomplete trials):")
        for dropped_test in all_dropped_tests:
            print(*dropped_test, sep=" | ")


if __name__ == "__main__":
    main()
