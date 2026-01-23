import mujoco
import numpy as np
from morphology_generator import generate_xml

LEG_LENGTHS = [0.08, 0.12, 0.16, 0.20]

def run_episode(xml_path, duration=5.0):
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    steps = int(duration / model.opt.timestep)
    start_x = data.qpos[0]

    for _ in range(steps):
        mujoco.mj_step(model, data)

    end_x = data.qpos[0]
    return end_x - start_x

results = []

for L in LEG_LENGTHS:
    xml_path = generate_xml(L, out_path=f"robot_L{L:.2f}.xml")
    displacement = run_episode(xml_path)

    results.append({
        "leg_length": L,
        "forward_displacement": displacement
    })

for r in results:
    print(r)
