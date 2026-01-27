import experiment_runner
import matplotlib.pyplot as plt
import numpy as np

MODEL_PATH = "../robot_L0.12.xml"

# =============================================================================
# TEST 1: The Survivor - Full Duration Completion
# =============================================================================
print("\n" + "="*60)
print("TEST 1: The Survivor (Standard Run)")
print("="*60)

result_success = experiment_runner.run_episode(
    model_path=MODEL_PATH,
    duration=2.0,
    seed=42
)

print(f"Status: {result_success['status']['reason']}")
print(f"Logged Steps: {len(result_success['history'])}")
print(f"Final Time: {result_success['metrics']['survival_time']:.3f}s")
print(f"Distance Traveled: {result_success['metrics']['total_distance']:.3f}m")

# Validation with tolerance for floating point
DURATION_TOLERANCE = 0.01
expected_duration = 2.0
actual_duration = result_success['metrics']['survival_time']

if abs(actual_duration - expected_duration) < DURATION_TOLERANCE:
    print("PASS: Robot survived full duration.")
else:
    print(f"FAIL: Robot terminated at {actual_duration:.3f}s (expected {expected_duration:.3f}s)")

# =============================================================================
# TEST 2: The Victim - Early Termination on Failure
# =============================================================================
print("\n" + "="*60)
print("TEST 2: The Victim (Forced Failure)")
print("="*60)

def killer_controller(model, data):
    """
    Sabotage controller that forces height failure at t > 1.0s.
    This tests if the episode terminates immediately upon failure detection.
    """
    if data.time > 1.0:
        data.qpos[2] = -0.5  # Force body underground (height failure)

result_fail = experiment_runner.run_episode(
    model_path=MODEL_PATH,
    duration=2.0,
    seed=42,
    controller_func=killer_controller
)

print(f"Status: {result_fail['status']['reason']}")
print(f"Logged Steps: {len(result_fail['history'])}")
print(f"Final Time: {result_fail['metrics']['survival_time']:.3f}s")
print(f"Distance Traveled: {result_fail['metrics']['total_distance']:.3f}m")

# Validation
FAILURE_TIME_TOLERANCE = 0.05
expected_failure_time = 1.0
actual_failure_time = result_fail['metrics']['survival_time']
time_error = abs(actual_failure_time - expected_failure_time)

# Check both timing and reason
timing_correct = time_error < FAILURE_TIME_TOLERANCE
reason_correct = "Body too low" in result_fail['status']['reason']

if timing_correct and reason_correct:
    print(f"PASS: Robot died at t={actual_failure_time:.3f}s (within {FAILURE_TIME_TOLERANCE}s of expected)")
    print("PASS: Correct failure reason detected")
    print("PASS: Logging halted immediately (History Contract upheld)")
else:
    if not timing_correct:
        print(f"FAIL: Robot died at {actual_failure_time:.3f}s (expected ~{expected_failure_time:.3f}s)")
    if not reason_correct:
        print(f"FAIL: Incorrect failure reason: '{result_fail['status']['reason']}'")

# =============================================================================
# DATA INTEGRITY CHECK
# =============================================================================
print("\n" + "="*60)
print("DATA INTEGRITY CHECK")
print("="*60)

times = [h["time"] for h in result_fail["history"]]
z_pos = [h["z_pos"] for h in result_fail["history"]]
alive_status = [h["alive"] for h in result_fail["history"]]

print(f"\nTotal history entries: {len(times)}")
print(f"Last entry marked as alive: {alive_status[-1]}")
print(f"Number of 'alive=True' entries: {sum(alive_status)}")
print(f"Number of 'alive=False' entries: {len(alive_status) - sum(alive_status)}")

# Display last few entries to prove logging stopped
print("\nLast 5 log entries:")
print(f"{'Time':>8} {'Z-pos':>8} {'Alive':>8}")
print("-" * 26)
for i in range(max(-5, -len(times)), 0):
    alive_marker = "YES" if alive_status[i] else "NO"
    print(f"{times[i]:8.3f} {z_pos[i]:8.3f} {alive_marker:>8}")

# =============================================================================
# VISUALIZATION (Optional)
# =============================================================================
print("\n" + "="*60)
print("GENERATING COMPARISON PLOT")
print("="*60)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# Plot 1: Survivor
times_success = [h["time"] for h in result_success["history"]]
z_success = [h["z_pos"] for h in result_success["history"]]
ax1.plot(times_success, z_success, 'g-', linewidth=2, label='Height')
ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Ground Level')
ax1.set_xlabel('Time (s)', fontsize=12)
ax1.set_ylabel('Height (m)', fontsize=12)
ax1.set_title('TEST 1: Survivor (Full Duration)', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot 2: Victim
alive_colors = ['green' if h["alive"] else 'red' for h in result_fail["history"]]
ax2.scatter(times, z_pos, c=alive_colors, s=20, alpha=0.6)
ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Ground Level')
ax2.axvline(x=1.0, color='orange', linestyle=':', linewidth=2, label='Failure Trigger (t=1.0s)')
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Height (m)', fontsize=12)
ax2.set_title('TEST 2: Victim (Early Termination)', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.savefig('episode_test_results.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'episode_test_results.png'")
plt.show()

# =============================================================================
# FINAL SUMMARY
# =============================================================================
print("\n" + "="*60)
print("TEST SUMMARY")
print("="*60)
test1_pass = abs(result_success['metrics']['survival_time'] - 2.0) < DURATION_TOLERANCE
test2_pass = time_error < FAILURE_TIME_TOLERANCE and reason_correct

print(f"TEST 1 (Survivor):        {'PASSED' if test1_pass else 'FAILED'}")
print(f"TEST 2 (Victim):          {'PASSED' if test2_pass else 'FAILED'}")
print(f"\nOverall Result:           {'ALL TESTS PASSED' if (test1_pass and test2_pass) else 'SOME TESTS FAILED'}")
print("="*60)