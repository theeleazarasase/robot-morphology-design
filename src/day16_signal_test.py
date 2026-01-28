import numpy as np
import matplotlib.pyplot as plt
from controllers import SineWaveController

def verify_signal():
    print("🔬 Day 16: CPG Signal Verification")
    
    # 1. Setup Controller
    # A = 0.5 rad (~28 degrees), f = 1.0 Hz
    cpg = SineWaveController(amplitude=0.5, frequency=1.0)
    
    # 2. Simulate Time
    duration = 2.0  # seconds
    dt = 0.002      # standard timestep
    time_steps = np.arange(0, duration, dt)
    
    left_commands = []
    right_commands = []
    
    print(f"   Simulating {len(time_steps)} steps...")
    
    for t in time_steps:
        angles = cpg.get_action(t)
        left_commands.append(angles[0])
        right_commands.append(angles[1])
        
    # 3. Analyze Phase Relationship
    # At t=0.25s (1/4 cycle), sin(pi/2) = 1. Left should be Max Positive.
    # Right should be Max Negative.
    
    check_idx = int(0.25 / dt)
    l_val = left_commands[check_idx]
    r_val = right_commands[check_idx]
    
    print("-" * 40)
    print(f"   Check at t=0.25s:")
    print(f"   Left Command:  {l_val:.4f} (Expected ~0.5)")
    print(f"   Right Command: {r_val:.4f} (Expected ~ -0.5)")
    
    if l_val > 0 and r_val < 0:
        print("✅ SUCCESS: Antiphase behavior confirmed.")
    else:
        print("❌ FAIL: Phase alignment incorrect.")

    # 4. Plot for Visual Confirmation
    plt.figure(figsize=(10, 4))
    plt.plot(time_steps, left_commands, label='Left Hip', color='blue')
    plt.plot(time_steps, right_commands, label='Right Hip', color='red', linestyle='--')
    plt.title('CPG Output: Antiphase Synchronization')
    plt.xlabel('Time (s)')
    plt.ylabel('Target Angle (rad)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('cpg_verification.png')
    print("✅ Plot saved to 'cpg_verification.png'")

if __name__ == "__main__":
    verify_signal()