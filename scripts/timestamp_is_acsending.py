#!/usr/bin/env bash

def is_strictly_ascending_timestamps(file_path):
    previous_timestamp = None

    with open(file_path, 'r') as file:
        for line_num, line in enumerate(file, 1):
            parts = line.split()

            if len(parts) < 2:
                print(f"Warning: Line {line_num} does not have at least two columns.")
                continue

            try:
                timestamp = int(parts[1])
            except ValueError:
                print(f"\033[91mERROR\033[0m: Line {line_num} - failed to parse timestamp.")
                return False

            if previous_timestamp is not None and timestamp < previous_timestamp:
                print(f"\033[91mERROR\033[0m: Line {line_num} - timestamp {timestamp} is not ascending.")
                return False

            previous_timestamp = timestamp

    return True

log_file_path = 'timestamps'
if is_strictly_ascending_timestamps(log_file_path):
    print("\033[92mPASSED\033[0m: timestamps are ascending.")
else:
    print("\033[91mERROR\033[0m: timestamps are not ascending.")
