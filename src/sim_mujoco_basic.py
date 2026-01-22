import mujoco
from mujoco import viewer
import numpy as np
import time
from pathlib import Path

XML_PATH = Path(__file__).parent / "two_link_arm.xml"

def reset_data(model, data):
    mujoco.mj_resetData(model, data)
    data.qpos[:] = model.key_qpos[0] if model.nkey > 0 else data.qpos
    data.qvel[:] = 0

def run_episode(duration=5.0):
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)

    sim_steps = int(duration / model.opt.timestep)

    positions = []

    with viewer.launch_passive(model, data) as v:
        for _ in range(sim_steps):
            mujoco.mj_step(model, data)

            # Log base position (or first body)
            start_x = data.qpos[0]
            positions.append(data.qpos.copy())

            v.sync()
            time.sleep(model.opt.timestep)
            end_x = data.qpos[0]
            forward_displacement = end_x - start_x

    return np.array(positions)

if __name__ == "__main__":
    traj = run_episode()
    print("Final qpos:", traj[-1])
