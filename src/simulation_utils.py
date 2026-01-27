import mujoco
import numpy as np
import random

def reset_simulation(model, data, seed=None):
    """
    Resets MuJoCo simulation to a clean, deterministic state.
    
    Args:
        model: MjModel object
        data: MjData object
        seed: (Optional) Int for deterministic randomness
    """
    # 1. Seed RNGs (Python + NumPy)
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    
    # 2. Wipe MuJoCo Data (Clears previous forces, warm-starts, etc.)
    mujoco.mj_resetData(model, data)
    
    # 3. Set Custom Start State
    # Example: Drop from 0.3m
    data.qpos[2] = 0.3
    
    # 4. Forward Kinematics (CRITICAL)
    # Updates sensors and geometry positions for t=0 before physics starts
    mujoco.mj_forward(model, data)
    
    return