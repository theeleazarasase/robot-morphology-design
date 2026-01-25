import mujoco
import numpy as np
import csv
import os

# Let's define STATELOGGER
class StateLogger:
    def __init__(self,filepath):
        self.filepath = filepath
        self.data_log = []
        # Header for our CSV
        self.header = ["time", "global_x", "global_z", "qpos_len", "qvel_len"]
   
    def log_step(self, data):
        """extracting specific indices from mujoco data structures. qpos[0] = Global X.
        qpos[2]= Global z"""
        #a time stamp
        t= data.time

        #b. Global Position 
        # qpos structure: [x, y, z, qw, qx, qy, qz, joint1, ...]
        g_x = data.qpos[0]  # Metric 1: Forward Displacement
        g_z = data.qpos[2]  # Metric 2: Height (Fall detection)

        #c. debugging sizes toprove the idnex map mismatch
        qp_len = len(data.qpos)
        qv_len = len(data.qvel)
        #d. store as row
        self.data_log.append([t, g_x,g_z, qp_len, qv_len])


    def save(self):
        """Writes the log to disk."""
        f = open(self.filepath, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(self.header)
        writer.writerows(self.data_log)
        f.close()
        print(f"Logs saved to {self.filepath}")

    def validate_drop_test(self):
        """
        Automated Sanity Check logic with Floating Point Tolerance.
        """
        # Define Tolerance Constants
        # 1e-6 is standard for "effectively zero" in double-precision physics 
        DRIFT_TOLERANCE = 1e-6 
        
        # Convert to numpy for easy slicing
        arr = np.array(self.data_log)

        # --- TEST 1: GRAVITY (Z should decrease) ---
        start_z = arr[0, 2]
        end_z = arr[-1, 2]
        
        print("\n--- SANITY CHECK RESULTS ---")
        print(f"Start Z: {start_z:.6f} m")
        print(f"End Z:   {end_z:.6f} m")
        
        if end_z < (start_z - DRIFT_TOLERANCE):
            print("PASS: Gravity is active (Z decreased significantly).")
        else:
            print("FAIL: Robot did not fall (Check gravity vector or frozen joints).")
        
        
        # --- TEST 2: PHANTOM FORCES (X should be zero within tolerance) ---
        # We check the absolute maximum deviation from zero
        max_x_drift = np.max(np.abs(arr[:, 1]))
        print(f"Max X Drift: {max_x_drift:.9f} m") # High precision print
        print(f"Tolerance:   {DRIFT_TOLERANCE:.9f} m")
        
        if max_x_drift < DRIFT_TOLERANCE:
            print(" PASS: Horizontal drift is within floating-point tolerance.")
        else:
            print(" FAIL: Significant drift detected! (Check mesh symmetry or collisions).")


if __name__ == "__main__":
    # 1. Load Model (Using relative path from src/)
    model_path = "../robot_L0.12.xml"

    if not os.path.exists(model_path):
        print(f"Error: Model not found at {model_path}")
        exit()

    print(f"Loading model: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)


    #2. Initialize Logger
    logger = StateLogger("../results/day10_sanity_log.csv")

    # 3. Run Simulation Loop (The "Drop Test")
    # We run for 1000 steps. Default dt is usually 0.002s -> 2 seconds total
    print("Starting Drop Test Simulation")

    for _ in range(1000):
        #Apply Gravity/Physics 
        mujoco.mj_step(model,data)

        #log state
        logger.log_step(data)

    #4. save and validate
    logger.save()
    logger.validate_drop_test