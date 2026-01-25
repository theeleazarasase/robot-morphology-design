import mujoco
import numpy as np
import csv
import os

# --- CONFIGURATION ---
MODEL_PATH = "../robot_L0.12.xml"
OUTPUT_PATH = "../results/day11_friction_sweep.csv"
FRICTION_VALUES = [0.1, 0.5, 1.0, 2.0]  # Ice, Plastic, Rubber, Sticky
INITIAL_VELOCITY = 1.0  # m/s (The "Push")
SIM_STEPS = 1000        # 2 seconds

class FrictionExperiment:
    def __init__(self, model_path):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.results = []

    def run_sweep(self):
        print(f"Starting Friction Sweep on {MODEL_PATH}")
        print(f"Test Condition: Initial Velocity = {INITIAL_VELOCITY} m/s")
        print("-" * 50)

        for mu in FRICTION_VALUES:
            self.run_single_trial(mu)
            
        self.save_results()

    def run_single_trial(self, mu):
        mujoco.mj_resetData(self.model, self.data)
        
        # 1. SET FRICTION
        # CRITICAL FIX: MuJoCo defaults to max(geom1, geom2).
        # We must update ALL geoms (robot + floor) to ensure the sweep works.
        for i in range(self.model.ngeom):
            self.model.geom_friction[i, 0] = mu

        # 2. SETTLING PHASE (New)
        # Let gravity do its work so the robot is firmly on the ground.
        # We step 500 times (approx 1 second)
        for _ in range(500):
            mujoco.mj_step(self.model, self.data)
            
        # Check if it actually settled (Z velocity should be near 0)
        if abs(self.data.qvel[2]) > 0.01:
            print(f"Warning: Robot still bouncing (vz={self.data.qvel[2]:.3f}) when kicked.")

        # 3. INJECT ENERGY (The "Kick")
        # We add to the current velocity
        self.data.qvel[0] += INITIAL_VELOCITY
        
        # Record where we started the slide
        start_x = self.data.qpos[0]
        
        # 4. RUN SLIDE PHASE
        print(f"Testing mu={mu}...", end="")
        stopping_time = None
        final_dist = 0.0
        
        for step in range(SIM_STEPS):
            mujoco.mj_step(self.model, self.data)
            
            time = self.data.time
            x_pos = self.data.qpos[0]
            x_vel = self.data.qvel[0]
            
            # Log data
            self.results.append([mu, time, x_pos, x_vel])

            # Stop condition: Velocity drops near zero
            if stopping_time is None and abs(x_vel) < 0.05 and step > 10:
                stopping_time = time
                final_dist = x_pos - start_x  # Distance relative to kick

        # Handle case where it never fully stops in time
        if stopping_time is None:
             final_dist = self.data.qpos[0] - start_x
             print(f" (Did not stop) Dist={final_dist:.3f}m")
        else:
             print(f" Slide Dist={final_dist:.3f}m")
        print(f" Stopped at t={stopping_time:.2f}s, Dist={final_dist:.3f}m")

    def save_results(self):
        with open(OUTPUT_PATH, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["friction_mu", "time", "x_pos", "x_vel"])
            writer.writerows(self.results)
        print("-" * 50)
        print(f"Data saved to {OUTPUT_PATH}")

if __name__ == "__main__":
    experiment = FrictionExperiment(MODEL_PATH)
    experiment.run_sweep()