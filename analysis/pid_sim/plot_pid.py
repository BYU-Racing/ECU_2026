"""
Runs the pid_sim binary and plots target vs PID output.

Data can come from a CAN dump file (--can-dump) or from stdin as a raw
time_ms,target CSV. PID parameters are always provided via --pid.

Usage:
    uv run plot_pid.py --pid 0.2,0.05,0.0,10.0 --can-dump can-dump.txt
    uv run plot_pid.py --pid 0.2,0.05,0.0,10.0 < your_input.csv
"""

import argparse
import io
import subprocess
import sys
from pathlib import Path

import git
import matplotlib.pyplot as plt
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from parse_can import STEP_MS, parse_dump

PROJECT_ROOT = Path(git.Repo(__file__, search_parent_directories=True).working_tree_dir)
BINARY = PROJECT_ROOT / ".pio" / "build" / "pid_sim" / "program"

# Mirrors constants from lib/ecu/constants.hpp at commit 30fdd1f (the last test run).
# FIXME_HACKS_TO_GET_THINGS_WORKING was active, so only throttle1 was used.
THROTTLE1_LOW = 660
THROTTLE1_HIGH = 950
MAX_TORQUE = 1000  # units: 0.1 Nm (Cascadia format)

# Matches PID_PERIOD_MS in lib/ecu/constants.hpp
PID_PERIOD_MS = 10


def throttle1_to_target_nm(t1: int) -> float:
    """Map a raw throttle1 ADC value to a torque target in Nm, matching the
    ECU logic at the time of the dump (saturating map, throttle1 only)."""
    pct = (t1 - THROTTLE1_LOW) * 100 / (THROTTLE1_HIGH - THROTTLE1_LOW)
    pct = max(0.0, min(100.0, pct))
    return pct * MAX_TORQUE / 1000


def can_dump_to_sim_input(path: Path) -> tuple[list[str], list[float]]:
    """Parse a CAN dump and return (time_ms,target rows, actual torques in Nm).

    Actual torques are converted from Cascadia format (0.1 Nm) to Nm.
    """
    pairs = parse_dump(path)
    if not pairs:
        print("error: no paired throttle messages found", file=sys.stderr)
        sys.exit(1)

    ticks_per_message = STEP_MS // PID_PERIOD_MS

    rows = []
    actual_torques = []
    for i, (t1, _t2, _brake, torque_raw) in enumerate(pairs):
        target_nm = throttle1_to_target_nm(t1)
        for tick in range(ticks_per_message):
            rows.append(f"{i * STEP_MS + tick * PID_PERIOD_MS},{target_nm:.4f}")
            actual_torques.append(torque_raw / 10.0)

    return rows, actual_torques


def main():
    parser = argparse.ArgumentParser(description="Plot PID response against a target signal.")
    parser.add_argument("--pid", required=True, metavar="p,i,d,windup_cap",
                        help="PID parameters as a comma-separated list")
    parser.add_argument("--can-dump", type=Path, metavar="FILE",
                        help="CAN dump file to use as input (otherwise reads time_ms,target from stdin)")
    args = parser.parse_args()

    actual_torques = None
    if args.can_dump:
        rows, actual_torques = can_dump_to_sim_input(args.can_dump)
        sim_input = args.pid + "\n" + "\n".join(rows)
    else:
        sim_input = args.pid + "\n" + sys.stdin.read()

    if not BINARY.exists():
        print(f"error: pid_sim binary not found at {BINARY}", file=sys.stderr)
        print("hint: run `pio run -e pid_sim` first", file=sys.stderr)
        sys.exit(1)

    result = subprocess.run([BINARY], input=sim_input, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        sys.exit(result.returncode)

    df = pd.read_csv(io.StringIO(result.stdout))
    time_s = df["time_ms"] / 1000.0

    fig, ax = plt.subplots()
    ax.plot(time_s, df["target"], label="Target", linestyle="--")
    ax.plot(time_s, df["output"], label="PID Output")
    if actual_torques is not None:
        ax.plot(time_s, actual_torques[:len(df)], label="Actual (dump)", linestyle=":")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("PID Response")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
