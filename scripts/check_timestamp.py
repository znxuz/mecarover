#!/usr/bin/env python3

import sys

def is_valid_timestamps(file_path):
    with open(file_path, 'r') as file:
        for line_num, line in enumerate(file, 1):
            parts = line.split()

            if len(parts) < 2:
                print(f"Warning: Line {line_num} does not have at least two columns.")
                continue

            try:
                int(parts[1])
            except ValueError:
                print(f"\033[91mERROR\033[0m: Line {line_num} - failed to parse timestamp.")
                return False

    return True

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

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 script.py <ascending|valid> <file_path>")
        sys.exit(1)

    mode = sys.argv[1]
    file_path = sys.argv[2]

    if mode == 'ascending':
        if is_strictly_ascending_timestamps(file_path):
            print("\033[92mPASSED\033[0m: timestamps are ascending.")
        else:
            print("\033[91mERROR\033[0m: timestamps are not ascending.")
    elif mode == 'valid':
        if is_valid_timestamps(file_path):
            print("\033[92mPASSED\033[0m: timestamps are valid.")
        else:
            print("\033[91mERROR\033[0m: timestamps are not valid.")
    else:
        print("Invalid mode. Use 'ascending' or 'valid'.")
