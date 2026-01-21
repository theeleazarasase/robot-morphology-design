import pybullet as p
import pybullet_data
import time


def main(gui=True):
    # 1. Connect to PyBullet
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)

    # 2. Basic simulation setup
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)

    # 3. Load environment
    plane_id = p.loadURDF("plane.urdf")

    # 4. Load a built-in robot (no custom URDF yet)
    start_pos = [0, 0, 0.3]
    start_ori = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF(
        "r2d2.urdf",
        start_pos,
        start_ori,
        useFixedBase=False
    )

    # 5. Inspect joints (important for later control work)
    num_joints = p.getNumJoints(robot_id)
    print(f"Robot has {num_joints} joints")

    for j in range(num_joints):
        info = p.getJointInfo(robot_id, j)
        print(j, info[1].decode("utf-8"), info[2])

    # 6. Simple simulation loop
    for step in range(2400):  # ~10 seconds
        p.stepSimulation()

        if gui:
            time.sleep(1.0 / 240.0)

    # 7. Clean shutdown
    p.disconnect()


if __name__ == "__main__":
    main(gui=True)
