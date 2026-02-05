import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time

csv_file = 'motor_state_log.csv'
num_joints = 7
window_sec = 5.0         # 实时窗口长度(秒)
refresh_rate = 5         # 图像更新频率(Hz)
dt = 1/refresh_rate

def plot_realtime():
    plt.ion()
    fig, axs = plt.subplots(num_joints, 3, figsize=(14, 2.8 * num_joints), sharex=True)
    lines = []

    for j in range(num_joints):
        # Position
        ax = axs[j, 0]
        l1, = ax.plot([], [], 'b', label=f'Joint {j} Command Pos')
        l2, = ax.plot([], [], 'r', label=f'Joint {j} Feedback Pos')
        ax.set_ylabel('Pos(rad)')
        ax.legend(loc='upper left')
        ax.grid()
        ax2 = ax.twinx()
        l3, = ax2.plot([], [], 'g', label=f'Joint {j} Error')
        ax2.set_ylabel('Error')
        ax2.legend(loc='upper right')
        lines.append((l1, l2, l3))
        # # Velocity
        # axs[j, 1].plot([], [], 'b', label=f'Joint {j} Command Vel')
        # axs[j, 1].plot([], [], 'r', label=f'Joint {j} Feedback Vel')
        # axs[j, 1].set_ylabel('Vel(rad/s)')
        # axs[j, 1].legend()
        # axs[j, 1].grid()
        # # Torque
        # axs[j, 2].plot([], [], 'b', label=f'Joint {j} Command Tau')
        # axs[j, 2].plot([], [], 'r', label=f'Joint {j} Feedback Tau')
        # axs[j, 2].set_ylabel('Tau')
        # axs[j, 2].legend()
        # axs[j, 2].grid()

    axs[-1, 0].set_xlabel('Time(s)')
    plt.tight_layout()

    # 用于画速度、力矩的line对象
    v_lines, tau_lines = [], []
    for j in range(num_joints):
        v_l1, = axs[j, 1].plot([], [], 'b', label=f'Joint {j} Command Vel')
        v_l2, = axs[j, 1].plot([], [], 'r', label=f'Joint {j} Feedback Vel')
        axs[j, 1].grid()
        v_lines.append((v_l1, v_l2))
        tau_l1, = axs[j, 2].plot([], [], 'b', label=f'Joint {j} Command Tau')
        tau_l2, = axs[j, 2].plot([], [], 'r', label=f'Joint {j} Feedback Tau')
        tau_lines.append((tau_l1, tau_l2))
        axs[j, 2].grid()

    while True:
        try:
            df = pd.read_csv(csv_file)
            time_col = df['timestamp_ms'] / 1000.0
            time_arr = time_col.values

            tnow = time_arr[-1] if len(time_arr) else 0
            tmin = tnow - window_sec
            mask = time_arr >= tmin
            # 截取窗口数据
            time_disp = time_arr[mask]

            for j in range(num_joints):
                q_cmd = df[f'q_cmd_{j}'][mask].values
                q_fb = df[f'q_{j}'][mask].values
                error = q_fb - q_cmd
                # 更新位置
                l1, l2, l3 = lines[j]
                l1.set_data(time_disp, q_cmd)
                l2.set_data(time_disp, q_fb)
                l2.set_label(f'Joint {j} Feedback Pos: {q_fb[-1]:.3f}')
                l3.set_data(time_disp, error)
                axs[j, 0].relim(); axs[j, 0].autoscale_view()
                axs[j, 0].set_xlim(tmin, tnow)
                axs[j, 0].set_title(f"Joint {j}")
                axs[j, 0].legend(loc='upper left')
                # 更新速度
                dq_cmd = df[f'dq_cmd_{j}'][mask].values
                dq_fb = df[f'dq_{j}'][mask].values
                v_l1, v_l2 = v_lines[j]
                v_l1.set_data(time_disp, dq_cmd)
                v_l2.set_data(time_disp, dq_fb)
                v_l2.set_label(f'Joint {j} Feedback Vel: {dq_fb[-1]:.3f}')
                axs[j, 1].relim(); axs[j, 1].autoscale_view()
                axs[j, 1].set_xlim(tmin, tnow)
                axs[j, 1].legend()
                # 更新力矩
                tau_cmd = df[f'tau_cmd_{j}'][mask].values
                tau_fb = df[f'tau_{j}'][mask].values
                tau_l1, tau_l2 = tau_lines[j]
                tau_l1.set_data(time_disp, tau_cmd)
                tau_l2.set_data(time_disp, tau_fb)
                tau_l2.set_label(f'Joint {j} Feedback Tau: {tau_fb[-1]:.3f}')
                axs[j, 2].relim(); axs[j, 2].autoscale_view()
                axs[j, 2].set_xlim(tmin, tnow)
                axs[j, 2].legend()

            plt.pause(dt)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f'Plotting error: {e}')
            time.sleep(0.5)

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    plot_realtime()