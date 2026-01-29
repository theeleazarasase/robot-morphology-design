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
    
# src/experiment_runner.py (Partial Update)

def run_episode(model_path, duration=5.0, seed=None, controller=None):
    """
    Runs a single simulation episode.
    
    Args:
        model_path (str): Path to the XML file.
        duration (float): Max time in seconds.
        seed (int): Random seed.
        controller (object): An object with a .get_action(time) method.
    """
    # ... (Setup code remains the same: load model, reset data) ...
    if seed is not None:
        np.random.seed(seed)
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Deterministic Reset
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    history = []
    status = EpisodeStatus.RUNNING
    reason = None
    
    # === THE MAIN LOOP ===
    while data.time < duration:
        
        # 1. GET CONTROL SIGNAL (The New Part)
        if controller is not None:
            # Ask the brain for angles based on current time
            action = controller.get_action(data.time)
            # Apply to motors (assuming action order matches actuator order)
            data.ctrl[:] = action
            
        # 2. STEP PHYSICS
        mujoco.mj_step(model, data)
        
        # 3. CHECK FOR FAILURE (The Referee)
        # Check for NaN (Explosion)
        if np.isnan(data.qpos).any():
            status = EpisodeStatus.TERMINATED_FAILURE
            reason = "NaN detected (Simulation Explosion)"
            break
            
        # Check for Fall (Torso Z < 0.05m roughly)
        # Note: We use body name 'torso' index usually, or just qpos[2] for simple root
        if data.qpos[2] < 0.10: # Threshold: 10cm off ground
            status = EpisodeStatus.TERMINATED_FAILURE
            reason = f"Body too low: z={data.qpos[2]:.3f}m"
            break

        # 4. LOGGING (Record data)
        # ... (Logging code remains the same) ...
        history.append({
            "time": data.time,
            "x_pos": data.qpos[0],
            "z_pos": data.qpos[2],
            "alive": True
        })

    # ... (Cleanup and return code remains the same) ...
    
    # Calculate final metrics
    final_x = data.qpos[0]
    total_distance = final_x  # Assuming start at 0
    
    if status == EpisodeStatus.RUNNING:
        status = EpisodeStatus.COMPLETED
        reason = "Time Limit Reached"
        
    return {
        "metadata": {"model": model_path, "duration": duration, "seed": seed},
        "metrics": {"total_distance": total_distance, "survival_time": data.time},
        "status": {"state": status.name, "reason": reason},
        "history": history
    }# src/experiment_runner.py (Partial Update)

def run_episode(model_path, duration=5.0, seed=None, controller=None):
    """
    Runs a single simulation episode.
    
    Args:
        model_path (str): Path to the XML file.
        duration (float): Max time in seconds.
        seed (int): Random seed.
        controller (object): An object with a .get_action(time) method.
    """
    # ... (Setup code remains the same: load model, reset data) ...
    if seed is not None:
        np.random.seed(seed)
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Deterministic Reset
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    history = []
    status = EpisodeStatus.RUNNING
    reason = None
    
    # === THE MAIN LOOP ===
    while data.time < duration:
        
        # 1. GET CONTROL SIGNAL (The New Part)
        if controller is not None:
            # Ask the brain for angles based on current time
            action = controller.get_action(data.time)
            # Apply to motors (assuming action order matches actuator order)
            data.ctrl[:] = action
            
        # 2. STEP PHYSICS
        mujoco.mj_step(model, data)
        
        # 3. CHECK FOR FAILURE (The Referee)
        # Check for NaN (Explosion)
        if np.isnan(data.qpos).any():
            status = EpisodeStatus.TERMINATED_FAILURE
            reason = "NaN detected (Simulation Explosion)"
            break
            
        # Check for Fall (Torso Z < 0.05m roughly)
        # Note: We use body name 'torso' index usually, or just qpos[2] for simple root
        if data.qpos[2] < 0.10: # Threshold: 10cm off ground
            status = EpisodeStatus.TERMINATED_FAILURE
            reason = f"Body too low: z={data.qpos[2]:.3f}m"
            break

        # 4. LOGGING (Record data)
        # ... (Logging code remains the same) ...
        history.append({
            "time": data.time,
            "x_pos": data.qpos[0],
            "z_pos": data.qpos[2],
            "alive": True
        })

    # ... (Cleanup and return code remains the same) ...
    
    # Calculate final metrics
    final_x = data.qpos[0]
    total_distance = final_x  # Assuming start at 0
    
    if status == EpisodeStatus.RUNNING:
        status = EpisodeStatus.COMPLETED
        reason = "Time Limit Reached"
        
    return {
        "metadata": {"model": model_path, "duration": duration, "seed": seed},
        "metrics": {"total_distance": total_distance, "survival_time": data.time},
        "status": {"state": status.name, "reason": reason},
        "history": history
    }