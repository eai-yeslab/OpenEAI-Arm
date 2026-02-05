import pybullet as p
import pybullet_data
import numpy as np
import pandas as pd
import time

urdf = "assets/openeai_arm_urdf_ros2/urdf/STEP.urdf"
csv = "sim_motor_state_log.csv"
reset_pose = np.array([-1.46, -1.49, -0.18, 0.226, 2.76, 2.03, -2.22])
# 启动可视化
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF(urdf, useFixedBase=True)

# 记录关节顺序（确保和csv一致）
num_joints = p.getNumJoints(robot_id)
joint_idxs = [i for i in range(num_joints)]  # 需据你的URDF调整

df = pd.read_csv(csv)
q_cols = [f'q_{i}' for i in range(num_joints)]
traj = df[q_cols].to_numpy()

for q in traj:
    for i, idx in enumerate(joint_idxs):
        p.resetJointState(robot_id, idx, float(q[i] - reset_pose[i]))
    p.stepSimulation()
    time.sleep(1/200.0)

input("Press Enter to exit and close the window...")
p.disconnect()