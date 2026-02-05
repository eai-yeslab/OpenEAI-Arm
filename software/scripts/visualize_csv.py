import pandas as pd
import matplotlib.pyplot as plt

csv_file = 'motor_test_log.csv'  # 修改为你的文件名
csv_file = 'motor_sin_track_log.csv'  # 修改为你的文件名

# 读取数据
df = pd.read_csv(csv_file)
num_joints = 7   # 如果你的关节数不是7请修改
draw = False

# 绘制所有关节的设定/反馈轨迹
draw = True
fig, axs = plt.subplots(num_joints, 1, figsize=(12, 2.2*num_joints), sharex=True)

for jid in range(num_joints):
    t = df['timestamp_ms'] / 1000.0  # 单位秒
    axs[jid].plot(t, df[f'q_cmd_{jid}'], label=f'Setpoint q_cmd_{jid}')
    axs[jid].plot(t, df[f'q_feedback_{jid}'], label=f'Feedback q_{jid}')
    axs[jid].set_ylabel(f'Joint {jid} (rad)')
    axs[jid].legend(loc='best')
    axs[jid].grid()

axs[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.show()
draw = False

# ---- 绘制误差曲线（反馈-指令） ----
# draw = True
fig2, axs2 = plt.subplots(num_joints, 1, figsize=(12, 2.2*num_joints), sharex=True)
for jid in range(num_joints):
    error = df[f'q_feedback_{jid}'] - df[f'q_cmd_{jid}']
    t = df['timestamp_ms'] / 1000.0
    axs2[jid].plot(t, error, label=f'Error q{jid}')
    axs2[jid].set_ylabel(f'Error Joint {jid} (rad)')
    axs2[jid].legend()
    axs2[jid].grid()
axs2[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.show()
draw = False

# ---- 绘制所有关节的速度和力矩 ----
# draw = True
fig3, axs3 = plt.subplots(num_joints, 2, figsize=(14, 2.4*num_joints), sharex=True)
for jid in range(num_joints):
    t = df['timestamp_ms'] / 1000.0
    axs3[jid,0].plot(t, df[f'dq_{jid}'], label=f'dq_{jid}')
    axs3[jid,0].set_ylabel(f'Velocity J{jid}')
    axs3[jid,0].legend()
    axs3[jid,0].grid()
    axs3[jid,1].plot(t, df[f'tau_{jid}'], label=f'tau_{jid}')
    axs3[jid,1].set_ylabel(f'Torque J{jid}')
    axs3[jid,1].legend()
    axs3[jid,1].grid()
axs3[-1,0].set_xlabel('Time (s)')
axs3[-1,1].set_xlabel('Time (s)')
plt.tight_layout()
plt.show()
draw = False