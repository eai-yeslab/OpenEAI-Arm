import OpenEAIArm
import time
import numpy as np

arm = OpenEAIArm.OpenEAIArm("configs/default_right.yml")
try:
    # Move to the reset pose.
    arm.reset()
    # Single-step control.
    target = np.array([0.0, 0.25, 0.75, -0.5, 0.0, 0.0, 0.0])
    arm.set_joint_targets(target, 1.5, interpolate=True)
    time.sleep(2.5)

    N = 100
    radius = 0.02
    index_x = 0
    index_y = 2
    position = np.array(arm.forward_kinetics(arm.get_joint_positions()))
    print(arm.get_joint_positions())
    print(position)
    center = position.copy()
    center[index_x] += radius

    positions = [position]
    real_positions = [position]
    joint_targets = [arm.get_joint_positions()]
    successes = [True]
    ik_options = OpenEAIArm.IKOptions()
    ik_options.angle_jump_threshold = 0.28
    for i in range(1, N+1):
        new_pos = center.copy()
        new_pos[index_y] += radius * np.sin(i / N * np.pi * 2)
        new_pos[index_x] -= radius * np.cos(i / N * np.pi * 2)
        joint_target, success = arm.inverse_kinetics(list(new_pos), ik_options)
        positions.append(new_pos)
        joint_targets.append(joint_target)
        successes.append(success)
        
    if not all(successes):
        print("IK failed for some positions.")
    else:
        print("IK succeeded for all positions.")
        for joint_target in joint_targets:
            arm.set_joint_targets(joint_target, 0.05, interpolate=True)
            time.sleep(0.05)
            real_positions.append(arm.forward_kinetics(arm.get_joint_positions()))

    # Plot the trajectory
    import matplotlib.pyplot as plt
    positions = np.array(positions)
    real_positions = np.array(real_positions)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2])
    ax.plot(real_positions[:, 0], real_positions[:, 1], real_positions[:, 2], 'r--')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('End Effector Trajectory')
    plt.show()

    # test_time = 8.0
    # freq_hz = 0.25
    # amplitude = 0.2
    # send_freq = 50.0
    # dt = 1.0/send_freq

    # t0 = time.time()

    # for k in range(int(test_time * send_freq)):
    #     tn = time.time()
    #     t = tn - t0
    #     cycle = np.zeros(7)
    #     for j in range(6):
    #         cycle[j] = target[j] + amplitude * np.sin(2*np.pi*freq_hz*t)
    #     cycle[-1] = target[-1] + 0.04 * np.sin(2*np.pi*freq_hz*t)
    #     arm.set_joint_targets(cycle, 0.0)
    #     time.sleep(dt)

    # time.sleep(2)
    # Compensating torque.
    # q = np.zeros(7)
    # tau = arm.compute_static_tau(q)
    # print("Gravity compensation:", tau)

    # Disable motors.

except Exception as e:
    print("Error:", e)
finally:
    time.sleep(1)
    arm.go_home(1.0)
    time.sleep(0.5)
    arm.disable_all()
