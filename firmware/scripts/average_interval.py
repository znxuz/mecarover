#!/usr/bin/env python3

import sys
from collections import defaultdict

def calculate_average_distances(data):
    entries = []
    for line in data.split('\n'):
        if not line.strip():
            continue
        try:
            parts = line.split()
            name = parts[0]
            timestamp = int(parts[1])
            state = int(parts[2])
            entries.append((name, timestamp, state))
        except Exception as e:
            print(f"Error parsing line: '{line.strip()}' - {str(e)}")
            raise

    group_data = defaultdict(list)
    prev_timestamps = {}

    for name, timestamp, state in entries:
        if state == 1:
            if name in prev_timestamps:
                distance = timestamp - prev_timestamps[name]
                group_data[name].append(distance)
            prev_timestamps[name] = timestamp

    averages = {}
    for name, distances in group_data.items():
        if distances:
            averages[name] = sum(distances) / len(distances)

    return averages

# Read data from file (replace with your actual file path)
if len(sys.argv) < 2:
    print("Usage: python <script_name> <filename>")
    sys.exit(1)

filename = sys.argv[1]
with open(filename, 'r') as f:
    data = f.read()

averages = calculate_average_distances(data)

# Print results
for name, avg in sorted(averages.items()):
    print(f"{name}: {avg:.2f} µs = {1_000_000/avg:.2f} Hz")
