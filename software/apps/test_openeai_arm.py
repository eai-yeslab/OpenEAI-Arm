import OpenEAIArm
import time
import numpy as np

arm = OpenEAIArm.OpenEAIArm("configs/default.yml")
# 回零
arm.go_home()
# 单步控制
target = np.array([1.0, 0.5, 1.5, -0.5, 0.67, 0.44, 0.04])
arm.set_joint_targets(target, 1.0, interpolate=True)
time.sleep(5)

test_time = 8.0
freq_hz = 0.25
amplitude = 0.2
send_freq = 50.0
dt = 1.0/send_freq

t0 = time.time()

for k in range(int(test_time * send_freq)):
    tn = time.time()
    t = tn - t0
    cycle = np.zeros(7)
    for j in range(6):
        cycle[j] = target[j] + amplitude * np.sin(2*np.pi*freq_hz*t)
    cycle[-1] = target[-1] + 0.04 * np.sin(2*np.pi*freq_hz*t)
    arm.set_joint_targets(cycle, 0.0)
    time.sleep(dt)

time.sleep(2)
# 补偿力矩
# q = np.zeros(7)
# tau = arm.compute_static_tau(q)
# print("Gravity compensation:", tau)

# 关闭
arm.go_home()
arm.disable_all()