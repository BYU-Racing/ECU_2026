"""
Finds optimal PID parameters for the throttle controller using differential
evolution against a real CAN dump.

The PID is simulated directly in Python (matching the C++ implementation in
util.cpp) to avoid subprocess overhead during optimization. The cost function
is a weighted sum of ISE (tracking error) and a rate-of-change penalty
(smoothness):

    cost = ISE + lambda * sum((output[i] - output[i-1])^2)

--smoothness controls lambda. Higher values produce smoother output at the
cost of more lag. Start around 1.0 and adjust based on the plot.

Usage:
    uv run tune_pid.py --can-dump can-dump.txt
    uv run tune_pid.py --can-dump can-dump.txt --smoothness 5.0
    uv run tune_pid.py --can-dump can-dump.txt --windup-cap 10.0 --smoothness 1.0
    uv run tune_pid.py --can-dump can-dump.txt --p-max 2.0 --i-max 20.0 --d-max 0.0
"""

import argparse
import sys
from pathlib import Path

from scipy.optimize import differential_evolution

sys.path.insert(0, str(Path(__file__).parent))
from plot_pid import throttle1_to_target_nm
from parse_can import parse_dump, STEP_MS


def simulate_pid(p: float, i_gain: float, d_gain: float, windup_cap: float,
                 targets: list[float], times_ms: list[int]) -> list[float] | None:
    """Python reimplementation of Pid::nextValue from util.cpp.

    Returns None if the simulation diverges (output grows without bound),
    indicating an unstable parameter combination.
    """
    integral = 0.0
    last_error = 0.0
    last_time = times_ms[0]
    last_output = 0.0
    outputs = []

    for t, target in zip(times_ms, targets):
        dt = (t - last_time) / 1000.0
        error = target - last_output

        integral += error * dt
        integral = max(-windup_cap, min(windup_cap, integral))

        d = (error - last_error) / dt if dt > 0 else 0.0
        last_error = error
        last_time = t

        output = p * error + i_gain * integral + d_gain * d
        if not (-1e6 < output < 1e6):
            return None
        last_output = output
        outputs.append(output)

    return outputs


class CostFn:
    """Callable class (picklable) for use with differential_evolution workers=-1."""
    def __init__(self, targets: list[float], times_ms: list[int],
                 windup_cap: float, smoothness: float):
        self.targets = targets
        self.times_ms = times_ms
        self.windup_cap = windup_cap
        self.smoothness = smoothness

    def __call__(self, params):
        p, i_gain, d_gain = params
        outputs = simulate_pid(p, i_gain, d_gain, self.windup_cap, self.targets, self.times_ms)
        if outputs is None:
            return float("inf")
        ise = sum((o - t) ** 2 for o, t in zip(outputs, self.targets))
        rate_penalty = sum((outputs[i] - outputs[i-1]) ** 2 for i in range(1, len(outputs)))
        return ise + self.smoothness * rate_penalty


def main():
    parser = argparse.ArgumentParser(description="Tune PID parameters against a CAN dump.")
    parser.add_argument("--can-dump", type=Path, required=True, metavar="FILE")
    parser.add_argument("--windup-cap", type=float, default=20.0, metavar="N",
                        help="Fixed windup cap in Nm (default: 20.0 = max torque)")
    parser.add_argument("--smoothness", type=float, default=10.0, metavar="N",
                        help="Rate-of-change penalty weight lambda (default: 10.0). "
                             "Higher = smoother output, more lag.")
    parser.add_argument("--p-max", type=float, default=5.0, metavar="N",
                        help="Upper search bound for P gain (default: 5.0)")
    parser.add_argument("--i-max", type=float, default=10.0, metavar="N",
                        help="Upper search bound for I gain (default: 10.0)")
    parser.add_argument("--d-max", type=float, default=0.5, metavar="N",
                        help="Upper search bound for D gain (default: 0.5)")
    args = parser.parse_args()

    bounds = [(0.0, args.p_max), (0.0, args.i_max), (0.0, args.d_max)]

    pairs = parse_dump(args.can_dump)
    if not pairs:
        print("error: no paired throttle messages found", file=sys.stderr)
        sys.exit(1)

    targets = [throttle1_to_target_nm(t1) for t1, *_ in pairs]
    times_ms = [i * STEP_MS for i in range(len(pairs))]
    windup_cap = args.windup_cap

    cost_fn = CostFn(targets, times_ms, windup_cap, args.smoothness)

    print(f"Optimizing over {len(pairs)} samples with:")
    print(f"  smoothness:  {args.smoothness}")
    print(f"  windup_cap:  {windup_cap} Nm")
    print(f"  p range:     [0, {args.p_max}]")
    print(f"  i range:     [0, {args.i_max}]")
    print(f"  d range:     [0, {args.d_max}]")
    print("This may take a few minutes...")

    result = differential_evolution(
        cost_fn,
        bounds=bounds,
        workers=-1,
        seed=42,
        tol=0.01,
        popsize=10,
        maxiter=300,
        disp=True,
    )

    p, i_gain, d_gain = result.x
    print(f"\nOptimal parameters:")
    print(f"  p={p:.4f}, i={i_gain:.4f}, d={d_gain:.4f}, windup_cap={windup_cap}")
    print(f"  Cost (ISE + {args.smoothness}*rate_penalty): {result.fun:.2f}")

    at_bound = []
    for val, (lo, hi), name in zip(result.x, bounds, ["p", "i", "d"]):
        if abs(val - lo) < 1e-6 or abs(val - hi) < 1e-6:
            at_bound.append(name)
    if at_bound:
        print(f"  warning: {', '.join(at_bound)} hit a search bound — consider widening BOUNDS in tune_pid.py")

    print(f"\nTo plot: uv run plot_pid.py --pid {p:.4f},{i_gain:.4f},{d_gain:.4f},{windup_cap} --can-dump {args.can_dump}")


if __name__ == "__main__":
    main()
