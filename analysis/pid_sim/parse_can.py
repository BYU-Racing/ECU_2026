"""
Parses a CAN dump and outputs pedal messages as a timestamped CSV.

Since the dump has no timestamps, each throttle pair is assigned a timestamp
30ms after the previous one. Throttle1 and Throttle2 are paired by order of
appearance and share the same timestamp.

Dump format:  can0  ID  [len]  byte0 byte1 ...
Output CSV:   time_ms,throttle1,throttle2,brake,actual_torque_raw

Usage:
    uv run parse_can.py <can-dump.txt>
"""

import struct
import sys
from pathlib import Path

THROTTLE1_ID = 0x001
THROTTLE2_ID = 0x002
BRAKE_ID = 0x003
CONTROL_COMMAND_ID = 0x0C0
STEP_MS = 30


def read_u16_le(data: list[str]) -> int:
    return struct.unpack_from("<H", bytes(int(b, 16) for b in data[:2]))[0]


def read_i16_le(data: list[str]) -> int:
    return struct.unpack_from("<h", bytes(int(b, 16) for b in data[:2]))[0]


def parse_dump(path: Path):
    """Parse the dump, pairing throttle1 and throttle2 in order of appearance.

    If one sensor's message is missing (two of the same type appear in a row),
    the pending unpaired message is dropped to resynchronize. Brake and
    actual_torque use the most recent value seen at the time each pair is emitted.

    Returns a list of (throttle1, throttle2, brake, actual_torque_raw) tuples,
    where actual_torque_raw is in Cascadia format (0.1 Nm units).
    """
    pairs: list[tuple[int, int, int, int]] = []
    pending_t1: int | None = None
    pending_t2: int | None = None
    last_brake: int = 0
    last_torque_raw: int = 0
    dropped = 0

    with open(path) as f:
        for line in f:
            parts = line.split()
            # format: can0  ID  [len]  byte0 byte1 ...
            if len(parts) < 4:
                continue
            msg_id = int(parts[1], 16)
            data = parts[3:]

            if msg_id == THROTTLE1_ID:
                if pending_t1 is not None:
                    dropped += 1
                pending_t1 = read_u16_le(data)
            elif msg_id == THROTTLE2_ID:
                if pending_t2 is not None:
                    dropped += 1
                pending_t2 = read_u16_le(data)
            elif msg_id == BRAKE_ID:
                last_brake = read_u16_le(data)
            elif msg_id == CONTROL_COMMAND_ID:
                last_torque_raw = read_i16_le(data)

            if pending_t1 is not None and pending_t2 is not None:
                pairs.append((pending_t1, pending_t2, last_brake, last_torque_raw))
                pending_t1 = None
                pending_t2 = None

    if dropped > 0:
        print(f"warning: dropped {dropped} unpaired message(s) to resynchronize", file=sys.stderr)

    return pairs


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <can-dump.txt>", file=sys.stderr)
        sys.exit(1)

    pairs = parse_dump(Path(sys.argv[1]))

    if not pairs:
        print("error: no paired throttle messages found", file=sys.stderr)
        sys.exit(1)

    try:
        print("time_ms,throttle1,throttle2,brake,actual_torque_raw")
        for i, (t1, t2, brake, torque_raw) in enumerate(pairs):
            print(f"{i * STEP_MS},{t1},{t2},{brake},{torque_raw}")
    except BrokenPipeError:
        pass


if __name__ == "__main__":
    main()
