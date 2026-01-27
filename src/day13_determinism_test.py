import mujoco
import numpy as np
import os
import simulation_utils  # Importing your new library

MODEL_PATH = "../robot_L0.12.xml"
SIM_STEPS = 500  # 1 second of simulation

def run_episode(seed):
    """Runs one simulation episode and returns the Z-trajectory."""
    
    # Load Model
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Missing: {MODEL_PATH}")
        
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    
    # --- DETERMINISTIC RESET ---
    simulation_utils.reset_simulation(model, data, seed)
    
    trajectory = []
    
    # Run loop
    for _ in range(SIM_STEPS):
        mujoco.mj_step(model, data)
        # Log Z-height
        trajectory.append(data.qpos[2])
        
    return np.array(trajectory)

if __name__ == "__main__":
    print(" Day 13: Determinism Validation")
    print("-" * 40)
    
    # 1. Run Baseline (Seed 42)
    print("Running Episode 1 (Seed 42)...")
    traj_1 = run_episode(seed=42)
    
    # 2. Run Replicate (Seed 42)
    print("Running Episode 2 (Seed 42)...")
    traj_2 = run_episode(seed=42)
    
    # 3. Run Variant (Seed 999)
    print("Running Episode 3 (Seed 999)...")
    traj_3 = run_episode(seed=999)
    
    # --- VERIFICATION ---
    print("-" * 40)
    
    # Check 1: Exact Match?
    # We use tolerance 0.0 because with fixed seed, it should be bitwise identical.
    diff_1_2 = np.abs(traj_1 - traj_2).max()
    
    if diff_1_2 == 0.0:
        print(f" SUCCESS: Seed 42 vs Seed 42 -> Exact Match (Diff: {diff_1_2})")
    else:
        print(f" FAIL: Seed 42 vs Seed 42 -> Drift Detected (Diff: {diff_1_2})")
        print("   (Did you forget mj_resetData or mj_forward?)")

    # Check 2: Variation?
    # If the seed changes, specific random initializations (if any) or numerical noise
    # might cause drift. Note: If your simulation has NO randomness (no noise added),
    # even different seeds might look identical. 
    # But usually, different seeds imply we might eventually add noise. 
    # For now, let's see if they are identical (which they might be if physics is pure).
    
    diff_1_3 = np.abs(traj_1 - traj_3).max()
    print(f"  Info: Seed 42 vs Seed 999 -> Diff: {diff_1_3}")
    
    if diff_1_3 == 0.0:
        print("   (Trajectories are identical. This is normal if you haven't added noise/randomization yet.)")
    else:
        print("   (Trajectories differ. Randomness is active.)")