import mujoco
from mujoco import viewer

# Load XML in the same folder as this script
model = mujoco.MjModel.from_xml_path("./two_link_arm.xml")
data = mujoco.MjData(model)

# Step the simulation 500 steps
for _ in range(500):
    mujoco.mj_step(model, data)

# Launch the built-in viewer
viewer.launch(model, data)
