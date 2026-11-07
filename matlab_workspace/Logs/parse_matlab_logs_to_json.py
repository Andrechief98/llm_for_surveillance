"""
Parse MATLAB Command Window printouts of the intruder-detection simulation
into JSON files: two files per value of B, each containing all environments (worlds).

Usage (from the folder containing this script):
    python parse_matlab_logs_to_json.py LOGS_FOLDER/matlab_log_env*.txt --output_folder JSON_FOLDER

Output files, for each value of B found in the logs:
    results_B<B>_deterministic.json   tests 1 .. half of the tests     (logical sequences)
    results_B<B>_uncertain.json       tests half + 1 .. last test      (FP & HI sequences)

JSON layout:
    {"ILP": {"world_<environment>": {"trial_<k>": {"run_<j>": {
        "sensor_sequence": [...],
        "actions": [{"deploy": {"robot_1": "<area>", "robot_2": "<area>"}}, ...],
        "final_intruder_area": "<area or ->",
        "success": 0 or 1
    }}}}}

Definitions:
    world_<environment>  environment number from "Simulation in environment N, mode M, B = X"
    trial_<k>            k from "TRIAL k / N"
    run_<j>              progressive number inside each file: j = test number in the deterministic
                         file, j = test number - half of the tests in the uncertain file
    sensor_sequence      sensor names from "Tested sensors [...]", in printed order
    actions              one "deploy" per iteration with an "Assignments:" table; it contains only
                         the robots listed in that table; iterations without assignments add nothing
    final_intruder_area  value of "Intruder position:" ("-" when there is no intruder)
    success              1 if "Success:" was printed, or if no intruder was identified and the
                         intruder position is "-"; 0 otherwise

The program stops with an error, before writing anything, if two logs have the same environment and B.
Runs with missing information are not written; they are listed in the console output.
"""

import argparse
import json
import re
import sys
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
ASSIGNMENTS_HEADER_PATTERN = re.compile(r"^\s*Assignments:")
ASSIGNMENT_ROW_PATTERN = re.compile(
    rf"^\s*(\d+)\s+\{{'(\w+)'\}}\s+({NUMBER})\s+({NUMBER})\s+({NUMBER})\s*$"
)
TABLE_ROW_CANDIDATE_PATTERN = re.compile(r"^\s*\d+\s+\S")
SUCCESS_PATTERN = re.compile(r"^Success:")
NO_INTRUDER_IDENTIFIED_PATTERN = re.compile(r"no intruder identified")
INTRUDER_PATTERN = re.compile(r"^Intruder position:\s*(\S+)")
SUMMARY_HEADER_PATTERN = re.compile(r"^\s*Task\s+Trials\s+Successes")

NO_INTRUDER_SYMBOL = "-"
TOP_LEVEL_KEY = "ILP"
DETERMINISTIC_LABEL = "deterministic"
UNCERTAIN_LABEL = "uncertain"


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
        "sensor_sequence": None,
        "last_printed_iteration": None,
        "deployments_by_iteration": {},
        "unparsed_assignment_lines": [],
        "success_printed": False,
        "no_intruder_identified": False,
        "intruder_position": None,
    }


def parse_log(log_path):
    """Walk the log line by line; return file-level information and one record per run."""
    file_information = {"environment": None, "b_value": None, "n_trials": None}
    run_records = []
    trial = None
    current_run = None
    current_iteration = None
    inside_assignment_table = False

    for line in read_clean_lines(log_path):
        simulation_match = SIMULATION_PATTERN.search(line)
        if simulation_match:
            file_information["environment"] = int(simulation_match.group(1))
            file_information["b_value"] = simulation_match.group(3)  # None in older logs
            continue

        trial_match = TRIAL_PATTERN.search(line)
        if trial_match:
            trial = int(trial_match.group(1))
            file_information["n_trials"] = int(trial_match.group(2))
            continue

        if SUMMARY_HEADER_PATTERN.match(line):
            current_run = None  # stop attributing lines to the last test
            inside_assignment_table = False
            continue

        test_match = TEST_PATTERN.match(line)
        if test_match:
            current_run = new_run_record(trial, int(test_match.group(1)), int(test_match.group(2)))
            run_records.append(current_run)
            current_iteration = None
            inside_assignment_table = False
            continue

        if current_run is None:
            continue

        sensors_match = SENSORS_PATTERN.match(line)
        if sensors_match:
            current_run["sensor_sequence"] = [
                name.strip() for name in sensors_match.group(1).split(",") if name.strip()
            ]
            continue

        iteration_match = ITERATION_PATTERN.match(line)
        if iteration_match:
            current_iteration = int(iteration_match.group(1))
            current_run["last_printed_iteration"] = current_iteration
            inside_assignment_table = False
            continue

        if ASSIGNMENTS_HEADER_PATTERN.match(line):
            inside_assignment_table = True
            continue

        assignment_match = ASSIGNMENT_ROW_PATTERN.match(line)
        if assignment_match:
            robot_key = f"robot_{assignment_match.group(1)}"
            assigned_area = assignment_match.group(2)
            current_run["deployments_by_iteration"].setdefault(current_iteration, {})[robot_key] = assigned_area
            continue

        if inside_assignment_table and TABLE_ROW_CANDIDATE_PATTERN.match(line):
            current_run["unparsed_assignment_lines"].append((current_iteration, line.strip()))
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

    return file_information, run_records


# ---------------------------------------------------------------- per-run values
def run_success(run_record):
    if run_record["success_printed"]:
        return 1
    if run_record["no_intruder_identified"] and run_record["intruder_position"] == NO_INTRUDER_SYMBOL:
        return 1
    return 0


def missing_run_fields(run_record):
    """Names of the fields that were not found in the log for this run."""
    required_fields = ["sensor_sequence", "last_printed_iteration", "intruder_position"]
    return [field for field in required_fields if run_record[field] is None]


def build_run_entry(run_record):
    actions = [
        {"deploy": run_record["deployments_by_iteration"][iteration]}
        for iteration in sorted(run_record["deployments_by_iteration"])
    ]
    return {
        "sensor_sequence": run_record["sensor_sequence"],
        "actions": actions,
        "final_intruder_area": run_record["intruder_position"],
        "success": run_success(run_record),
    }


def sequence_type_and_run_number(run_record):
    """First half of the tests -> deterministic, second half -> uncertain; numbering restarts at 1."""
    half_of_tests = run_record["total_tests"] // 2
    if run_record["test"] <= half_of_tests:
        return DETERMINISTIC_LABEL, run_record["test"]
    return UNCERTAIN_LABEL, run_record["test"] - half_of_tests


def find_missing_runs(file_information, run_records):
    """(trial, test) pairs that never appear in the log, e.g. because the log was cut."""
    if not run_records:
        return []
    total_tests = max(run_record["total_tests"] for run_record in run_records)
    present_runs = {(run_record["trial"], run_record["test"]) for run_record in run_records}
    return [
        (trial, test_number)
        for trial in range(1, file_information["n_trials"] + 1)
        for test_number in range(1, total_tests + 1)
        if (trial, test_number) not in present_runs
    ]


# ---------------------------------------------------------------- grouping logs by B
def group_logs_by_b(parsed_logs):
    """Return {B: {environment: source file}}; stop with an error on duplicated (environment, B)."""
    logs_by_b = {}
    duplicate_messages = []
    for source_file, file_information, _ in parsed_logs:
        b_value = file_information["b_value"]
        environment = file_information["environment"]
        logs_for_b = logs_by_b.setdefault(b_value, {})
        if environment in logs_for_b:
            duplicate_messages.append(
                f"B = {b_value}, environment {environment}: {logs_for_b[environment]} and {source_file}"
            )
            continue
        logs_for_b[environment] = source_file
    if duplicate_messages:
        sys.exit("Error: more than one log with the same environment and B, nothing written.\n"
                 + "\n".join(duplicate_messages))
    return logs_by_b


def world_number(world_key):
    """"world_12" -> 12, so that worlds are sorted as numbers, not as text."""
    return int(world_key.split("_")[1])


# ---------------------------------------------------------------- main
def main():
    argument_parser = argparse.ArgumentParser(description="Parse MATLAB simulation logs into JSON files.")
    argument_parser.add_argument("log_paths", nargs="+", help="MATLAB log .txt files")
    argument_parser.add_argument("--output_folder", default=".")
    arguments = argument_parser.parse_args()

    parsed_logs = []
    for log_path in sorted(arguments.log_paths):
        source_file = Path(log_path).name
        file_information, run_records = parse_log(log_path)
        if file_information["environment"] is None or file_information["b_value"] is None:
            print(f"{source_file}: no environment or no B in the 'Simulation in environment' line, file skipped")
            continue
        if file_information["n_trials"] is None:
            print(f"{source_file}: no 'TRIAL k / N' line found, file skipped")
            continue
        parsed_logs.append((source_file, file_information, run_records))

    logs_by_b = group_logs_by_b(parsed_logs)

    # json_contents[(B, sequence type)][world][trial][run] = run entry
    json_contents = {}
    dropped_runs = []
    unparsed_rows = []
    for source_file, file_information, run_records in parsed_logs:
        b_value = file_information["b_value"]
        world_key = f"world_{file_information['environment']}"
        for trial, test_number in find_missing_runs(file_information, run_records):
            dropped_runs.append((source_file, f"trial {trial}", f"test {test_number}", "not in log"))
        for run_record in run_records:
            for iteration, unparsed_line in run_record["unparsed_assignment_lines"]:
                unparsed_rows.append((source_file, f"trial {run_record['trial']}",
                                      f"test {run_record['test']}", f"iteration {iteration}", unparsed_line))
            missing_fields = missing_run_fields(run_record)
            if missing_fields:
                dropped_runs.append((source_file, f"trial {run_record['trial']}",
                                     f"test {run_record['test']}", "missing " + ", ".join(missing_fields)))
                continue
            sequence_type, run_number = sequence_type_and_run_number(run_record)
            trial_runs = (
                json_contents.setdefault((b_value, sequence_type), {})
                .setdefault(world_key, {})
                .setdefault(f"trial_{run_record['trial']}", {})
            )
            trial_runs[f"run_{run_number}"] = build_run_entry(run_record)

    output_folder = Path(arguments.output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)
    for (b_value, sequence_type), worlds in sorted(json_contents.items()):
        output_path = output_folder / f"results_B{b_value}_{sequence_type}.json"
        sorted_worlds = {world: worlds[world] for world in sorted(worlds, key=world_number)}
        with open(output_path, "w", encoding="utf-8") as output_handle:
            json.dump({TOP_LEVEL_KEY: sorted_worlds}, output_handle, indent=2)
        n_runs = sum(len(trial_runs) for trials in worlds.values() for trial_runs in trials.values())
        world_sources = ", ".join(
            f"{world} <- {logs_by_b[b_value][world_number(world)]}" for world in sorted_worlds
        )
        print(f"{output_path.name}: {n_runs} runs written | {world_sources}")

    if dropped_runs:
        print("\nDropped runs (source file, trial, test, reason):")
        for dropped_run in dropped_runs:
            print(*dropped_run, sep=" | ")
    else:
        print("\nNo dropped runs")

    if unparsed_rows:
        print("\nUnparsed assignment rows (source file, trial, test, iteration, line):")
        for unparsed_row in unparsed_rows:
            print(*unparsed_row, sep=" | ")
    else:
        print("No unparsed assignment rows")


if __name__ == "__main__":
    main()
