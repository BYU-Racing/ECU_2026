#include <cstdio>
#include <cstdlib>
#include "util.hpp"

/* PID simulator. Reads PID parameters and a sequence of (time, target) pairs
 * from stdin, and outputs the PID response as CSV to stdout.
 *
 * stdin format:
 *   p,i,d,windup_cap          <- PID parameters (first line)
 *   time_ms,target            <- one row per PID tick (remaining lines)
 *
 * stdout format:
 *   time_ms,target,output     <- CSV with header
 *
 * Example usage:
 *   echo "0.2,0.05,0.0,10.0" > input.csv
 *   echo "0,0\n100,50\n200,100" >> input.csv
 *   .pio/build/pid_sim/program < input.csv | uv run analysis/pid_sim/plot_pid.py --pid p,i,d,windup_cap
 */
int main() {
    double p, i_gain, d_gain, windup_cap;
    if (scanf("%lf,%lf,%lf,%lf", &p, &i_gain, &d_gain, &windup_cap) != 4) {
        fprintf(stderr, "error: first line must be: p,i,d,windup_cap\n");
        return 1;
    }

    uint32_t first_time_ms;
    double first_target;
    if (scanf("%u,%lf", &first_time_ms, &first_target) != 2) {
        fprintf(stderr, "error: no data rows provided\n");
        return 1;
    }

    /* Initialize the PID at the first timestamp so the first tick has
     * delta_time = 0 (P-only), rather than a large spurious integral spike. */
    Pid pid(first_time_ms, p, i_gain, d_gain, windup_cap);
    double last_output = 0.0;

    printf("time_ms,target,output\n");

    uint32_t time_ms = first_time_ms;
    double target = first_target;
    do {
        double output = pid.nextValue(time_ms, target, last_output);
        last_output = output;
        printf("%u,%f,%f\n", time_ms, target, output);
    } while (scanf("%u,%lf", &time_ms, &target) == 2);

    return 0;
}
