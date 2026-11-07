"""
Merge the JSON files written by parse_matlab_logs_to_json.py into ONE JSON file.

Usage (from the folder containing this script):
    python merge_json_results_v2.py JSON_FOLDER/results_B*.json --output_file results_all.json

Input files must be named results_B<B>_<sequence type>.json, with <sequence type> equal to
"deterministic" or "uncertain", and must contain {"ILP": {"world_<N>": ...}}.

Output layout:
    {
        "deterministic": {"ILP": {"B<B>": {"world_<N>": {"trial_<k>": {"run_<j>": {...}}}}, ...}},
        "uncertain":     {"ILP": {"B<B>": {...}, ...}}
    }
Sequence types are written deterministic first, then uncertain; B values in increasing numeric order.
The content of each world is copied unchanged, except that in every run the key
"final_intruder_area" is renamed "intruder_final_area" (same value, same position in the run).

The program stops with an error, before writing anything, if two input files have the same
B and sequence type, or if a file does not contain the "ILP" key.
"""

import argparse
import json
import re
import sys
from pathlib import Path

NUMBER = r"[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|NaN|Inf)"
INPUT_NAME_PATTERN = re.compile(rf"^results_B({NUMBER})_(deterministic|uncertain)\.json$")

TOP_LEVEL_KEY = "ILP"
OLD_INTRUDER_KEY = "final_intruder_area"
NEW_INTRUDER_KEY = "intruder_final_area"
SEQUENCE_TYPE_ORDER = ["deterministic", "uncertain"]


def read_input_files(json_paths):
    """Return {(sequence type, B as text): (file path, worlds)}; stop with an error on problems."""
    worlds_by_group = {}
    error_messages = []
    for json_path in sorted(json_paths):
        source_file = Path(json_path).name
        name_match = INPUT_NAME_PATTERN.match(source_file)
        if not name_match:
            print(f"{source_file}: name is not results_B<B>_<deterministic|uncertain>.json, file skipped")
            continue
        b_value, sequence_type = name_match.group(1), name_match.group(2)
        with open(json_path, encoding="utf-8") as input_handle:
            file_content = json.load(input_handle)
        if TOP_LEVEL_KEY not in file_content:
            error_messages.append(f"{source_file}: no '{TOP_LEVEL_KEY}' key")
            continue
        group_key = (sequence_type, b_value)
        if group_key in worlds_by_group:
            error_messages.append(
                f"B = {b_value}, {sequence_type}: {worlds_by_group[group_key][0]} and {json_path}"
            )
            continue
        worlds_by_group[group_key] = (json_path, file_content[TOP_LEVEL_KEY])
    if error_messages:
        sys.exit("Error: nothing written.\n" + "\n".join(error_messages))
    return worlds_by_group


def rename_intruder_key(run_entry):
    """Return a copy of the run with OLD_INTRUDER_KEY renamed NEW_INTRUDER_KEY, keeping the key order."""
    return {
        (NEW_INTRUDER_KEY if key == OLD_INTRUDER_KEY else key): value
        for key, value in run_entry.items()
    }


def rename_intruder_key_in_worlds(worlds):
    """Apply rename_intruder_key to every run; return (renamed worlds, runs without either key)."""
    renamed_worlds = {}
    runs_without_intruder_key = []
    for world_key, trials in worlds.items():
        for trial_key, trial_runs in trials.items():
            for run_key, run_entry in trial_runs.items():
                renamed_run = rename_intruder_key(run_entry)
                if NEW_INTRUDER_KEY not in renamed_run:
                    runs_without_intruder_key.append(f"{world_key} | {trial_key} | {run_key}")
                renamed_worlds.setdefault(world_key, {}).setdefault(trial_key, {})[run_key] = renamed_run
    return renamed_worlds, runs_without_intruder_key


def count_runs(worlds):
    return sum(len(trial_runs) for trials in worlds.values() for trial_runs in trials.values())


def main():
    argument_parser = argparse.ArgumentParser(description="Merge per-B JSON result files into one.")
    argument_parser.add_argument("json_paths", nargs="+", help="results_B<B>_<type>.json files")
    argument_parser.add_argument("--output_file", default="results_all.json")
    arguments = argument_parser.parse_args()

    worlds_by_group = read_input_files(arguments.json_paths)

    merged_content = {}
    all_runs_without_intruder_key = []
    for sequence_type in SEQUENCE_TYPE_ORDER:
        b_values = sorted(
            (b_value for group_type, b_value in worlds_by_group if group_type == sequence_type),
            key=float,
        )
        for b_value in b_values:
            json_path, original_worlds = worlds_by_group[(sequence_type, b_value)]
            worlds, runs_without_intruder_key = rename_intruder_key_in_worlds(original_worlds)
            merged_content.setdefault(sequence_type, {}).setdefault(TOP_LEVEL_KEY, {})[f"B{b_value}"] = worlds
            all_runs_without_intruder_key += [
                f"{Path(json_path).name} | {run_label}" for run_label in runs_without_intruder_key
            ]
            print(f"{sequence_type} | B{b_value} | {', '.join(worlds)} | "
                  f"{count_runs(worlds)} runs <- {Path(json_path).name}")

    with open(arguments.output_file, "w", encoding="utf-8") as output_handle:
        json.dump(merged_content, output_handle, indent=2)
    print(f"\nWritten: {arguments.output_file}")

    if all_runs_without_intruder_key:
        print(f"\nRuns without '{OLD_INTRUDER_KEY}' or '{NEW_INTRUDER_KEY}' (source file, world, trial, run):")
        for run_label in all_runs_without_intruder_key:
            print(run_label)
    else:
        print(f"All runs have '{NEW_INTRUDER_KEY}'")


if __name__ == "__main__":
    main()
