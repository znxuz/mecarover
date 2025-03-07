#!/usr/bin/env bash

def is_strictly_ascending_timestamps(file_path):
    previous_timestamp = None

    # Open the file and read line by line
    with open(file_path, 'r') as file:
        for line_num, line in enumerate(file, 1):
            # Split the line to extract columns
            parts = line.split()

            # Ensure we have at least two columns per line
            if len(parts) < 2:
                print(f"Warning: Line {line_num} does not have at least two columns.")
                continue

            # Extract the timestamp as an integer
            timestamp = int(parts[1])

            if previous_timestamp is not None and timestamp < previous_timestamp:
                print(f"\033[91mERROR\033[0m: Line {line_num} - timestamp {timestamp} is not strictly ascending.")
                return False

            # Update the previous timestamp
            previous_timestamp = timestamp

    return True

log_file_path = 'timestamps'
if is_strictly_ascending_timestamps(log_file_path):
    print("\033[92mPASSED\033[0m: timestamps strictly ascending.")
else:
    print("\033[91mERROR\033[0m: timestamps not strictly ascending.")
