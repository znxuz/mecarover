#!/usr/bin/env python3

import sys
from collections import defaultdict

def process_timestamps(file_path):
    tasks = defaultdict(list)

    with open(file_path, 'r') as file:
        for line in file:
            if not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            name, timestamp, flag = parts[0], int(parts[1]), int(parts[2])
            if flag == 1:
                tasks[name].append(timestamp)
            elif flag == 0 and tasks[name]:
                start_time = tasks[name].pop()
                duration = timestamp - start_time
                tasks[name].append(duration)

    results = []
    total_sum = 0

    # First calculate totals for percentage calculation
    temp_results = {}
    for name, durations in tasks.items():
        durations = [d for d in durations if isinstance(d, int)]
        if durations:
            total = sum(durations)
            temp_results[name] = (total / len(durations), total)
            total_sum += total

    # Now prepare sorted results with percentages
    for name, (avg, local_sum) in sorted(temp_results.items(), key=lambda x: x[1][0]):
        percentage = (local_sum / total_sum) * 100
        results.append((name, avg, local_sum, percentage))

    return results, total_sum

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <filename>")
        sys.exit(1)

    results, total_sum = process_timestamps(sys.argv[1])
    for name, avg, local_sum, percentage in results:
        print(f"{name}:")
        print(f"  Avg: {avg:.2f} µs")
        print(f"  Sum: {local_sum} µs ({percentage:.2f}%)")
    print(f"\nTotal sum of all durations: {total_sum} µs")
