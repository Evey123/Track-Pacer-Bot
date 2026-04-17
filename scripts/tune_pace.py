#!/usr/bin/env python3
import argparse
import csv
from statistics import mean


def load_rows(path: str):
    rows = []
    with open(path, newline="") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            parsed = {}
            for key, value in row.items():
                try:
                    parsed[key] = float(value)
                except (TypeError, ValueError):
                    parsed[key] = value
            rows.append(parsed)
    return rows


def summarize(rows):
    if not rows:
        raise ValueError("No rows found in log file.")

    schedule_errors = [r["schedule_error_s"] for r in rows if "schedule_error_s" in r]
    speed_errors = [r["speed_error_mps"] for r in rows if "speed_error_mps" in r]
    throttles = [r["throttle_cmd_us"] for r in rows if "throttle_cmd_us" in r]
    distances = [r["distance_m"] for r in rows if "distance_m" in r]
    elapsed = [r["elapsed_s"] for r in rows if "elapsed_s" in r]

    result = {
        "final_distance_m": max(distances) if distances else 0.0,
        "final_elapsed_s": max(elapsed) if elapsed else 0.0,
        "mean_schedule_error_s": mean(schedule_errors) if schedule_errors else 0.0,
        "max_abs_schedule_error_s": max(abs(x) for x in schedule_errors) if schedule_errors else 0.0,
        "mean_speed_error_mps": mean(speed_errors) if speed_errors else 0.0,
        "throttle_span_us": (max(throttles) - min(throttles)) if throttles else 0.0,
    }
    return result


def print_recommendations(summary):
    print("\n=== Pace Run Summary ===")
    for key, value in summary.items():
        print(f"{key}: {value:.4f}")

    print("\n=== Gain Tuning Suggestions ===")
    mean_sched = summary["mean_schedule_error_s"]
    max_sched = summary["max_abs_schedule_error_s"]
    mean_speed = summary["mean_speed_error_mps"]
    throttle_span = summary["throttle_span_us"]

    if mean_sched > 0.5:
        print("- Run is generally LATE (positive schedule error). Increase feedforward table or K_t / K_split slightly.")
    elif mean_sched < -0.5:
        print("- Run is generally EARLY (negative schedule error). Decrease feedforward table or K_t / K_split slightly.")
    else:
        print("- Mean schedule error is small; schedule bias is good.")

    if max_sched > 1.5:
        print("- Large schedule swings. Reduce K_t or K_split, or increase throttle smoothing/rate limit.")
    else:
        print("- Schedule swings are moderate.")

    if abs(mean_speed) > 0.25:
        print("- Non-trivial speed bias. Increase K_p_speed slightly and retest.")
    else:
        print("- Speed bias is small.")

    if throttle_span > 180:
        print("- Throttle is moving a lot. Increase throttle low-pass filtering or lower max throttle rate.")
    else:
        print("- Throttle activity looks reasonably smooth.")

    print("\nTip: change only one gain family at a time and rerun 3-5 repetitions.")


def main():
    parser = argparse.ArgumentParser(description="Analyze pace_run.csv and suggest gain tuning.")
    parser.add_argument("csv_path", nargs="?", default="pace_run.csv", help="Path to pace log CSV")
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    summary = summarize(rows)
    print_recommendations(summary)


if __name__ == "__main__":
    main()
