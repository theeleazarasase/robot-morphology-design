import mujoco
import numpy as np
from enum import Enum

# Import our project libraries
import simulation_utils
# We import the check_failure function from Day 12
from day12_failure_check import check_failure

class EpisodeStatus(Enum):
    INITIALIZED = "Initialized"
    RUNNING = "Running"
    COMPLETED = "Completed"          # Survived full duration
    TERMINATED_FAILURE = "Failure"   # Fallen or NaN
    
def run_episode(model_path, duration=5.0, seed=None, controller_func=None):
    """
    Executes a single, atomic simulation episode.
    
    Args:
        model_path (str): Path to XML file.
        duration (float): Max simulation time in seconds.
        seed (int): RNG seed for reproducibility.
        controller_func (callable): Optional f(model, data) -> None to apply control.
        
    Returns:
        dict: A standardized result packet containing metadata, metrics, and history.
    """
    
    # --- 1. SETUP (State: INITIALIZED) ---
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Deterministic Reset (Day 13 Logic)
    simulation_utils.reset_simulation(model, data, seed)
    
    status = EpisodeStatus.RUNNING
    termination_reason = "TimeLimit" # Default assumption
    
    history = []
    
    # Calculate max steps based on dt
    max_steps = int(duration / model.opt.timestep)
    start_x = data.qpos[0]
    
    # --- 2. SIMULATION LOOP (State: RUNNING) ---
    for step in range(max_steps):
        
        # A. Apply Control
        if controller_func:
            controller_func(model, data)
            
        # B. Step Physics
        mujoco.mj_step(model, data)
        
        # C. Check Failure
        failed, reason = check_failure(data)
        
        if failed:
            status = EpisodeStatus.TERMINATED_FAILURE
            termination_reason = reason
            break # <--- THE CONTRACT: Stop immediately.
            
        # D. Log History
        snapshot = {
            "time": float(data.time),
            "x_pos": float(data.qpos[0]),
            "z_pos": float(data.qpos[2]), 
            "alive": True
        }
        history.append(snapshot)
        
    # --- 3. FINALIZE (State Transition) ---
    # CRITICAL FIX: If we exited loop naturally, we succeeded.
    if status == EpisodeStatus.RUNNING:
        status = EpisodeStatus.COMPLETED

    # Calculate summary metrics
    final_time = data.time
    final_x = data.qpos[0]
    dist = final_x - start_x
    
    # Mark last frame as dead if we failed
    if status == EpisodeStatus.TERMINATED_FAILURE:
        if history:
            history[-1]["alive"] = False
    
    result = {
        "metadata": {
            "seed": seed,
            "duration": duration,
            "model": model_path
        },
        "metrics": {
            "total_distance": dist,
            "survival_time": final_time,
            "avg_speed": dist / final_time if final_time > 0 else 0.0
        },
        "status": {
            # REFINED SCHEMA: "success" means we survived the full duration
            "success": (status == EpisodeStatus.COMPLETED),
            "reason": termination_reason
        },
        "history": history
    }
    
    return result 