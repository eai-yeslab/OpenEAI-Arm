import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('motor_state_log.csv')
time = df['timestamp_ms'] / 1000.0

num_joints = 7

fig, axs = plt.subplots(num_joints, 3, figsize=(14, 2.8*num_joints), sharex=True)

for j in range(num_joints):
    # axs[j, 0].plot(time, df[f'q_cmd_{j}'], 'b', label=f'Joint {j} Command Pos')
    # axs[j, 0].plot(time, df[f'q_{j}'], 'r', label=f'Joint {j} Feedback Pos')
    # axs[j, 0].set_ylabel('Pos(rad)')
    # axs[j, 0].legend()
    # axs[j, 0].grid()
    # 左y轴：位置
    ax = axs[j, 0]
    ax.plot(time, df[f'q_cmd_{j}'], 'b', label=f'Joint {j} Command Pos')
    ax.plot(time, df[f'q_{j}'], 'r', label=f'Joint {j} Feedback Pos')
    ax.set_ylabel('Pos(rad)')
    ax.legend(loc='upper left')
    ax.grid()

    # 右y轴：绝对值误差
    ax2 = ax.twinx()
    with np.errstate(divide='ignore', invalid='ignore'):
        percent_err = np.where(df[f'q_cmd_{j}'] != 0,
                               (df[f'q_{j}'] - df[f'q_cmd_{j}']), #/ df[f'q_cmd_{j}'] * 100,
                               0)
    ax2.plot(time, percent_err, 'g', label=f'Joint {j} %Error')
    ax2.set_ylabel('Position Error')
    ax2.legend(loc='upper right')
    q01 = np.percentile(percent_err, 1)
    q99 = np.percentile(percent_err, 99)

    axs[j, 1].plot(time, df[f'dq_cmd_{j}'], 'b', label=f'Joint {j} Command Vel')
    axs[j, 1].plot(time, df[f'dq_{j}'], 'r', label=f'Joint {j} Feedback Vel')
    axs[j, 1].set_ylabel('Vel(rad/s)')
    axs[j, 1].legend()
    axs[j, 1].grid()

    axs[j, 2].plot(time, df[f'tau_cmd_{j}'], 'b', label=f'Joint {j} Command Tau')
    axs[j, 2].plot(time, df[f'tau_{j}'], 'r', label=f'Joint {j} Feedback Tau')
    axs[j, 2].set_ylabel('Tau')
    axs[j, 2].legend()
    axs[j, 2].grid()
    
    print(f"Joint {j} max: {max(df[f'q_{j}'])}, min: {min(df[f'q_{j}'])}, q01={q01:.3f}%  q99={q99:.3f}%")

plt.xlabel('Time(s)')
plt.tight_layout()
plt.show()