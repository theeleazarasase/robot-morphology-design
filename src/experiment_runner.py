import mujoco
import numpy as np
from enum import Enum
import os

# --- PRESERVED IMPORTS (Your hard work) ---
# Assuming these files exist in your src/ folder from previous days
try:
    import simulation_utils
    from day12_failure_check import check_failure
except ImportError:
    # Fallback if files are missing, just so the script doesn't crash immediately
    print(" Warning: Helper modules (day12/simulation_utils) not found. Using defaults.")
    def check_failure(data): return False, None

class EpisodeStatus(Enum):
    INITIALIZED = "Initialized"
    RUNNING = "Running"
    COMPLETED = "Completed"          # Survived full duration
    TERMINATED_FAILURE = "Failure"   # Fallen or NaN

def run_episode(model_path, duration=5.0, seed=None, controller=None):
    """
    Runs a single simulation episode with Physics + Energy Tracking.
    
    Args:
        model_path (str): Path to the XML file.
        duration (float): Max time in seconds.
        seed (int): Random seed.
        controller (object): An object with a .get_action(time) method.
    """
    # 1. SETUP
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    if seed is not None:
        np.random.seed(seed)
    
    # Deterministic Reset
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    # --- DAY 19 UPDATE: ENERGY TRACKING INIT ---
    total_mass = np.sum(model.body_mass)
    total_energy = 0.0
    start_x = data.qpos[0]  # Record start X to measure net displacement later
    # -------------------------------------------

    history = []
    status = EpisodeStatus.RUNNING
    reason = None
    
    # === THE MAIN LOOP ===
    while data.time < duration:
        
        # 1. GET CONTROL SIGNAL
        if controller is not None:
            # Ask the brain for angles based on current time
            action = controller.get_action(data.time)
            # Apply to motors (assuming action order matches actuator order)
            data.ctrl[:] = action
            
        # 2. STEP PHYSICS
        mujoco.mj_step(model, data)
        
        # --- DAY 19 UPDATE: CALCULATE ENERGY ---
        # Power P = |Torque * Velocity|
        # data.actuator_force gives the torque (N*m) applied by motors
        torques = data.actuator_force
        
        # data.qvel contains ALL velocities. 
        # Indices 0-5 are the "Root" (Free Joint) - the robot flying through air.
        # Indices 6+ are the "Joints" (Hips/Knees) - the motors.
        # We slice [6:] to match the shape of 'torques'.
        joint_velocities = data.qvel[6:]
        
        # Safety check: Shapes must match to multiply
        if len(torques) == len(joint_velocities):
            # Element-wise multiply, take abs(), sum them up -> Watts (Joules/sec)
            instantaneous_power = np.sum(np.abs(torques * joint_velocities))
            
            # Integrate: Energy = Power * TimeStep
            total_energy += instantaneous_power * model.opt.timestep
        # ---------------------------------------
        
        # 3. CHECK FOR FAILURE (The Referee)
        # Check for NaN (Explosion)
        if np.isnan(data.qpos).any():
            status = EpisodeStatus.TERMINATED_FAILURE
            reason = "NaN detected (Simulation Explosion)"
            break
            
        # Check for Fall using your Day 12 logic if available, or simple height check
        # We assume body Z is at qpos[2]
        if data.qpos[2] < 0.10: # Threshold: 10cm off ground
            status = EpisodeStatus.TERMINATED_FAILURE
            reason = f"Body too low: z={data.qpos[2]:.3f}m"
            break

        # 4. LOGGING (Record data)
        history.append({
            "time": data.time,
            "x_pos": data.qpos[0],
            "z_pos": data.qpos[2],
            "alive": True
        })

    # === POST-SIMULATION ANALYSIS ===
    
    final_x = data.qpos[0]
    
# --- DAY 19 UPDATE: COST OF TRANSPORT ---
    # Net Displacement (Start to Finish)
    displacement = final_x - start_x
    
    # We want to measure cost of moving *anywhere*, even backward.
    # So we use the absolute value of the distance.
    dist_magnitude = abs(displacement) 
    
    # Avoid division by zero
    if dist_magnitude < 0.01:
        cot = float('inf') # Didn't move significantly
    else:
        # CoT = Total Energy / (Mass * Gravity * Distance_Traveled)
        cot = total_energy / (total_mass * 9.81 * dist_magnitude)
    
    if status == EpisodeStatus.RUNNING:
        status = EpisodeStatus.COMPLETED
        reason = "Time Limit Reached"
        
    return {
        "metadata": {"model": model_path, "duration": duration, "seed": seed},
        "metrics": {
            "total_distance": displacement, # Keep sign (+) or (-) to know direction
            "survival_time": data.time,
            "total_energy": total_energy,
            "cot": cot
        },
        "status": {"state": status.name, "reason": reason},
        "history": history
    }