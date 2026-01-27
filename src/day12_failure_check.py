import mujoco
import numpy as np
import os
import time

# --- CONFIGURATION ---
MODEL_PATH = "../robot_L0.12.xml"

def check_failure(data):
    """Returns (failed: bool, reason: str)"""
    
    # Check 1: Numerical Stability (NaN OR Infinity)
    # np.isfinite returns False for both NaN and Inf
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        return True, "Unstable state (NaN or Inf detected)"
    
    # Check 2: Height failure (body on ground)
    if data.qpos[2] < 0.05:
        return True, f"Body too low: z={data.qpos[2]:.3f}m"
    
    # Check 3: Orientation failure (tipped over)
    # MuJoCo Quat is [w, x, y, z] -> indices 3, 4, 5, 6
    qx, qy = data.qpos[4], data.qpos[5]
    uprightness = 1.0 - 2.0 * (qx**2 + qy**2)
    
    if uprightness < 0.7:
        return True, f"Tipped over: R33={uprightness:.3f}"
    
    return False, "OK"

def run_test_case(name, setup_func):
    print(f"\n--- TEST CASE: {name} ---")
    
    if not os.path.exists(MODEL_PATH):
        print("Error: Model file not found.")
        return

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    
    # 1. Apply sabotage
    setup_func(data)
    
    # 2. IMMEDIATE CHECK (The Fix)
    # Verify the detector works on the raw corrupted state
    failed, reason = check_failure(data)
    if failed:
        print(f"✅ SUCCESS: Detected failure! Reason: {reason}")
        return

    # 3. Step simulation (Only if immediate check didn't catch it)
    print("  ...Immediate check passed (Unexpected for NaN), stepping sim...")
    for _ in range(10):
        try:
            mujoco.mj_step(model, data)
        except Exception as e:
            print(f"✅ SUCCESS: Simulation crashed as expected ({e})")
            return
            
        failed, reason = check_failure(data)
        if failed:
            print(f"✅ SUCCESS: Detected failure after step! Reason: {reason}")
            return

    print("❌ FAIL: Detector did NOT catch the failure.")
# --- SABOTAGE FUNCTIONS ---

def sabotage_drop(data):
    # Spawn it inside the floor (deep underground)
    print("Action: Spawning robot at z = 0.02m (Below threshold)")
    data.qpos[2] = 0.02

def sabotage_tip(data):
    # Rotate robot 90 degrees (lying on side)
    # Quat for 90 deg rotation around X is [0.707, 0.707, 0, 0]
    print("Action: Spawning robot rotated 90 degrees")
    data.qpos[3] = 0.707
    data.qpos[4] = 0.707
    data.qpos[2] = 0.5 # Keep it high so height check doesn't trigger first

def sabotage_nan(data):
    # Corrupt the state with Not-a-Number
    print("Action: Injecting NaN into velocity")
    data.qvel[0] = float('nan')

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    print("Running Day 12 Failure Detector Validation...")
    
    run_test_case("Height Limit", sabotage_drop)
    run_test_case("Orientation Limit", sabotage_tip)
    run_test_case("Numerical Explosion", sabotage_nan)