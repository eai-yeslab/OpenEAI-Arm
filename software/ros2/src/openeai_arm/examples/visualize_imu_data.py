import numpy as np
import h5py
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Line3D

from scipy.spatial.transform import Rotation as R
import csv

# ------------ 0. Camera to EE -----------------
dx, dy, dz = 0, -0.07, 0.04             # Bias on ee coordinates
angle_x_deg = -15.0                      # Rotation around end-effector x-axis
T_cam_to_ee_pos = np.array([dx, dy, dz])
T_cam_to_ee_rot = R.from_euler('x', angle_x_deg, degrees=True).as_matrix() # shape (3,3)

def make_transform(pos, rotmat):
    """Generate homogeneous transformation matrix: 4x4"""
    T = np.eye(4)
    T[:3,:3] = rotmat
    T[:3,3] = pos
    return T

def transform_point(T, p):
    """Apply 4x4 transformation matrix to a 3D point"""
    assert p.shape == (3,)
    p_h = np.ones(4)
    p_h[:3] = p
    return (T @ p_h)[:3]

def transform_dir(T, v):
    """Transform direction vector (rotation only)"""
    return T[:3,:3] @ v

# ------------ 1. Data Reading --------------
with h5py.File('eef_data.h5', 'r') as h5:
    eef_pose = h5['/observations/eef_pose'][:]  # (N,9): x,y,z,rx,ry,rz, eulerxyz (in radians)
    eef_ts = h5['/timestamps'][:]

# Camera CSV
cam_traj = []
with open('cam_traj.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        if row['is_lost'] == 'true':
            continue
        cam_traj.append({
            'timestamp': float(row['timestamp']),
            'pos': np.array([float(row['x']), float(row['y']), float(row['z'])]),      # SLAM trajectory points
            'quat': np.array([float(row['q_x']), float(row['q_y']), float(row['q_z']), float(row['q_w'])])
        })
cam_traj = sorted(cam_traj, key=lambda x: x['timestamp'])
cam_ts = np.array([x['timestamp'] for x in cam_traj])

# Get valid synchronization interval
t0 = max(eef_ts[0], cam_ts[0])
t1 = min(eef_ts[-1], cam_ts[-1])
valid_mask = (eef_ts >= t0) & (eef_ts <= t1)
eef_ts_valid = eef_ts[valid_mask]
eef_pose_valid = eef_pose[valid_mask]

skip = max(1, len(eef_ts_valid)//300)
eef_ts_vis = eef_ts_valid[::skip]
eef_pose_vis = eef_pose_valid[::skip]

# --------- 2/3/4. Trajectory/Theoretical Transformation ----------
def find_nearest_idx(src_ts, q):
    return np.abs(src_ts - q).argmin()

# Initial pose: first ee position and orientation
eef_pos0 = eef_pose_vis[0,:3]
eef_euler0 = eef_pose_vis[0,3:6]
R_eef_to_world0 = R.from_euler('xyz', eef_euler0, degrees=False).as_matrix()
T_eef_to_world0 = make_transform(eef_pos0, R_eef_to_world0)

# Initial theoretical camera pose (world frame)
T_cam_to_world0 = T_eef_to_world0.copy()
T_cam_to_world0[:3,:3] = R_eef_to_world0 @ T_cam_to_ee_rot
T_cam_to_world0[:3,3] = transform_point(T_eef_to_world0, T_cam_to_ee_pos)

# Initial SLAM pose
cam_idx0 = find_nearest_idx(cam_ts, eef_ts_vis[0])
cam_init_local_pos = cam_traj[cam_idx0]['pos']
cam_init_local_quat = cam_traj[cam_idx0]['quat']

# --- Animation cache ---
eef_traj_pts, eef_dirs = [], []
cam_traj_actual_pts, cam_traj_actual_dirs = [], []
cam_traj_theoretical_pts, cam_traj_theoretical_dirs = [], []
reverse_eef_pts, reverse_eef_dirs = [], []

for i, (ts, eef) in enumerate(zip(eef_ts_vis, eef_pose_vis)):
    # -- End-effector world coordinates --
    eef_pos = eef[:3]
    eef_euler = eef[3:6]
    R_eef_to_world = R.from_euler('xyz', eef_euler, degrees=False).as_matrix()
    T_eef_to_world = make_transform(eef_pos, R_eef_to_world)
    # EEF coordinate system z-axis direction (tool forward), in world
    eef_z_dir_world = R_eef_to_world @ np.array([0,0,1])
    eef_traj_pts.append(eef_pos)
    eef_dirs.append(eef_z_dir_world)

    # -- Theoretical camera world coordinates (based on current arm pose + fixed transform) --
    T_cam_to_world = T_eef_to_world.copy()
    T_cam_to_world[:3,:3] = R_eef_to_world @ T_cam_to_ee_rot
    T_cam_to_world[:3,3] = transform_point(T_eef_to_world, T_cam_to_ee_pos)
    # Camera world coordinates
    cam_theory_pos_world = T_cam_to_world[:3,3]
    cam_theory_z_dir_world = transform_dir(T_cam_to_world, np.array([0,0,1]))
    cam_traj_theoretical_pts.append(cam_theory_pos_world)
    cam_traj_theoretical_dirs.append(cam_theory_z_dir_world)

    # -- SLAM camera world coordinates (initial theoretical position + SLAM output increment) --
    cam_idx = find_nearest_idx(cam_ts, ts)
    rel_cam_pos = cam_traj[cam_idx]['pos'] - cam_init_local_pos           # SLAM trajectory increment
    rel_cam_quat = cam_traj[cam_idx]['quat']
    # Use SLAM output quaternion to convert to incremental rotation matrix
    dR_slam = R.from_quat(rel_cam_quat).as_matrix()
    # Current SLAM output camera rotation (in camera's own coordinate system), to be added to theoretical initial position
    cam_actual_pos_world = transform_point(T_cam_to_world0, rel_cam_pos)
    # Current camera Z axis in world coordinate system
    cam_actual_rot_world = T_cam_to_world0[:3,:3] @ dR_slam
    cam_actual_z_dir_world = cam_actual_rot_world @ np.array([0,0,1])
    cam_traj_actual_pts.append(cam_actual_pos_world)
    cam_traj_actual_dirs.append(cam_actual_z_dir_world)
    # -- Estimated end-effector world pose (position + z-axis direction) --
    # Construct current frame camera homogeneous transform (world frame)
    T_cam_actual_to_world = np.eye(4)
    T_cam_actual_to_world[:3,:3] = cam_actual_rot_world
    T_cam_actual_to_world[:3,3]  = cam_actual_pos_world

    # Invert fixed transform (camera to end-effector)
    T_ee_to_cam = np.eye(4)
    T_ee_to_cam[:3,:3] = T_cam_to_ee_rot
    T_ee_to_cam[:3,3]  = T_cam_to_ee_pos
    T_cam_to_ee = np.linalg.inv(T_ee_to_cam)

    # Inverse computation: from camera back to end-effector
    T_eef_hat_to_world = T_cam_actual_to_world @ T_cam_to_ee
    eef_hat_pos = T_eef_hat_to_world[:3,3]
    eef_hat_z_dir = T_eef_hat_to_world[:3,:3] @ np.array([0,0,1])

    # Save for subsequent visualization or error analysis
    reverse_eef_pts.append(eef_hat_pos)
    reverse_eef_dirs.append(eef_hat_z_dir)

eef_traj_pts = np.array(eef_traj_pts)
eef_dirs = np.array(eef_dirs)
cam_traj_actual_pts = np.array(cam_traj_actual_pts)
cam_traj_actual_dirs = np.array(cam_traj_actual_dirs)
cam_traj_theoretical_pts = np.array(cam_traj_theoretical_pts)
cam_traj_theoretical_dirs = np.array(cam_traj_theoretical_dirs)
reverse_eef_pts = np.array(reverse_eef_pts)
reverse_eef_dirs = np.array(reverse_eef_dirs)

# --------- Plotting and Animation ---------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('EEF & Camera Visualization')

# Single points
actual_pt, = ax.plot([],[],[],'o',color='blue', label='Actual Camera')
theory_pt, = ax.plot([],[],[],'o',color='cyan', label='Theory Camera')
eef_pt, = ax.plot([],[],[],'s',color='red', label='EEF')
reverse_eef_pt, = ax.plot([],[],[],'*',color='purple',label='Reverse EEF')

# Lines
line_actual = Line3D([],[],[], color='magenta', linestyle='-', linewidth=2,label='Actual Cam to EEF')
line_theory = Line3D([],[],[], color='cyan', linestyle='--', linewidth=2, label='Theory Cam to EEF')
line_reverse_eef = Line3D([],[],[], color='orange', linestyle=':', linewidth=2, label='Reverse EEF to Actual')
ax.add_line(line_actual)
ax.add_line(line_theory)
ax.add_line(line_reverse_eef)

# Arrows
eef_arrow = None
cam_actual_arrow = None
cam_theory_arrow = None
reverse_eef_arrow = None

ax.legend()

margin = 0.05  # 5% padding
all_pts = np.vstack([cam_traj_actual_pts, eef_traj_pts, cam_traj_theoretical_pts])
data_min = all_pts.min(axis=0)
data_max = all_pts.max(axis=0)
data_range = data_max - data_min
max_range = max(data_range)
mid = (data_max + data_min) / 2

ax.set_xlim(mid[0] - max_range/2 - margin*max_range, mid[0] + max_range/2 + margin*max_range)
ax.set_ylim(mid[1] - max_range/2 - margin*max_range, mid[1] + max_range/2 + margin*max_range)
ax.set_zlim(mid[2] - max_range/2 - margin*max_range, mid[2] + max_range/2 + margin*max_range)

def animate(i):
    # Current points
    actual_pt.set_data([cam_traj_actual_pts[i,0]], [cam_traj_actual_pts[i,1]])
    actual_pt.set_3d_properties([cam_traj_actual_pts[i,2]])
    theory_pt.set_data([cam_traj_theoretical_pts[i,0]], [cam_traj_theoretical_pts[i,1]])
    theory_pt.set_3d_properties([cam_traj_theoretical_pts[i,2]])
    eef_pt.set_data([eef_traj_pts[i,0]], [eef_traj_pts[i,1]])
    eef_pt.set_3d_properties([eef_traj_pts[i,2]])
    # Current reverse-estimated EEF point
    reverse_eef_pt.set_data([reverse_eef_pts[i,0]], [reverse_eef_pts[i,1]])
    reverse_eef_pt.set_3d_properties([reverse_eef_pts[i,2]])

    # Lines: [Cam(i) <-> EEF(i)]
    line_actual.set_data([cam_traj_actual_pts[i,0], reverse_eef_pts[i,0]],
                         [cam_traj_actual_pts[i,1], reverse_eef_pts[i,1]])
    line_actual.set_3d_properties([cam_traj_actual_pts[i,2], reverse_eef_pts[i,2]])

    line_theory.set_data([cam_traj_theoretical_pts[i,0], eef_traj_pts[i,0]],
                         [cam_traj_theoretical_pts[i,1], eef_traj_pts[i,1]])
    line_theory.set_3d_properties([cam_traj_theoretical_pts[i,2], eef_traj_pts[i,2]])

    line_reverse_eef.set_data([reverse_eef_pts[i,0], eef_traj_pts[i,0]],
                            [reverse_eef_pts[i,1], eef_traj_pts[i,1]])
    line_reverse_eef.set_3d_properties([reverse_eef_pts[i,2], eef_traj_pts[i,2]])

    # Remove/redraw arrows
    global eef_arrow, cam_actual_arrow, cam_theory_arrow, reverse_eef_arrow
    if eef_arrow:        eef_arrow.remove()
    if cam_actual_arrow: cam_actual_arrow.remove()
    if cam_theory_arrow: cam_theory_arrow.remove()
    if reverse_eef_arrow: reverse_eef_arrow.remove()
    eef_arrow = ax.quiver(
        eef_traj_pts[i,0], eef_traj_pts[i,1], eef_traj_pts[i,2],
        eef_dirs[i,0], eef_dirs[i,1], eef_dirs[i,2],
        color='red', length=0.04, normalize=True)
    cam_actual_arrow = ax.quiver(
        cam_traj_actual_pts[i,0], cam_traj_actual_pts[i,1], cam_traj_actual_pts[i,2],
        cam_traj_actual_dirs[i,0], cam_traj_actual_dirs[i,1], cam_traj_actual_dirs[i,2],
        color='blue', length=0.04, normalize=True)
    cam_theory_arrow = ax.quiver(
        cam_traj_theoretical_pts[i,0], cam_traj_theoretical_pts[i,1], cam_traj_theoretical_pts[i,2],
        cam_traj_theoretical_dirs[i,0], cam_traj_theoretical_dirs[i,1], cam_traj_theoretical_dirs[i,2],
        color='cyan', length=0.04, normalize=True)
    reverse_eef_arrow = ax.quiver(
        reverse_eef_pts[i,0], reverse_eef_pts[i,1], reverse_eef_pts[i,2],
        reverse_eef_dirs[i,0], reverse_eef_dirs[i,1], reverse_eef_dirs[i,2],
        color='purple', length=0.04, normalize=True)
    return actual_pt, theory_pt, eef_pt, reverse_eef_pt, line_actual, line_theory, line_reverse_eef, eef_arrow, cam_actual_arrow, cam_theory_arrow

def cam_trajectory_statistics(eef_pose_vis, cam_traj_actual_pts, cam_traj_theoretical_pts):
    """
    eef_pose_vis: (N,9) End-effector pose (position xyz + Euler ZYX radians)
    cam_traj_actual_pts: (N,3) SLAM output camera positions (world frame), time-synchronized with eef_pose
    cam_traj_theoretical_pts: (N,3) Theoretical end-effector calculated camera positions (world frame), also time-synchronized with eef_pose
    """
    # 1. Accuracy
    diffs = cam_traj_actual_pts - cam_traj_theoretical_pts
    accuracy_distances = np.linalg.norm(diffs, axis=1)
    accuracy_mean = np.mean(accuracy_distances)
    accuracy_rms = np.sqrt(np.mean(accuracy_distances**2))

    # 2. Compute the inverse of each end-effector transform to get world->eef transform, and apply to actual camera points to get "camera points relative to eef" for each frame
    rel_actual_pts = []
    for i in range(len(eef_pose_vis)):
        pos = eef_pose_vis[i,:3]
        euler = eef_pose_vis[i,3:6]
        R_eef_to_world = R.from_euler('zyx', euler, degrees=False).as_matrix()
        T_eef_to_world = np.eye(4)
        T_eef_to_world[:3,:3] = R_eef_to_world
        T_eef_to_world[:3,3] = pos
        T_world_to_eef = np.linalg.inv(T_eef_to_world)
        cam_actual_world = cam_traj_actual_pts[i]
        cam_actual_rel_eef = T_world_to_eef[:3,:3] @ (cam_actual_world - T_eef_to_world[:3,3])
        rel_actual_pts.append(cam_actual_rel_eef)
    rel_actual_pts = np.vstack(rel_actual_pts)  # shape (N,3)

    # 3. Precision
    deltas = rel_actual_pts - np.mean(rel_actual_pts, axis=0)
    precision_distances = np.linalg.norm(deltas, axis=1)
    precision_mean = np.mean(precision_distances)
    precision_rms = np.sqrt(np.mean(precision_distances**2))

    # Print
    print("======== Camera Trajectory Statistics ========")
    print(f"[Accuracy] SLAM actual - theoretical mean distance: {accuracy_mean:.5f} m, RMS: {accuracy_rms:.5f} m")
    print(f"[Precision] Camera relative to end-effector distribution (jitter) mean: {precision_mean:.5f} m, RMS: {precision_rms:.5f} m")
    return {
        "accuracy_mean": accuracy_mean,
        "accuracy_rms": accuracy_rms,
        "precision_mean": precision_mean,
        "precision_rms": precision_rms,
        "accuracy_distances": accuracy_distances,
        "precision_distances": precision_distances,
        "rel_actual_pts": rel_actual_pts
    }
    
def reverse_eef_statistics(eef_pose_vis, reverse_eef_pts):
    """
    eef_pose_vis: (N,9) Actual end-effector world pose + ZYX Euler
    reverse_eef_pts: (N,3) End-effector world positions reverse-estimated by SLAM
    """
    # 1. Accuracy
    diffs = reverse_eef_pts - eef_pose_vis[:,:3]
    accuracy_distances = np.linalg.norm(diffs, axis=1)
    accuracy_mean = np.mean(accuracy_distances)
    accuracy_rms = np.sqrt(np.mean(accuracy_distances**2))

    # 2. Precision (transform each reverse-estimated point to the actual end-effector coordinate frame for that frame, then compute jitter)
    rel_reverse_pts = []
    for i in range(len(eef_pose_vis)):
        pos = eef_pose_vis[i,:3]
        euler = eef_pose_vis[i,3:6]
        R_eef_to_world = R.from_euler('zyx', euler, degrees=False).as_matrix()
        T_eef_to_world = np.eye(4)
        T_eef_to_world[:3,:3] = R_eef_to_world
        T_eef_to_world[:3,3] = pos
        T_world_to_eef = np.linalg.inv(T_eef_to_world)
        reverse_eef_world = reverse_eef_pts[i]
        reverse_eef_rel_eef = T_world_to_eef[:3,:3] @ (reverse_eef_world - T_eef_to_world[:3,3])
        rel_reverse_pts.append(reverse_eef_rel_eef)
    rel_reverse_pts = np.vstack(rel_reverse_pts)  # shape (N,3)

    deltas = rel_reverse_pts - np.mean(rel_reverse_pts, axis=0)
    precision_distances = np.linalg.norm(deltas, axis=1)
    precision_mean = np.mean(precision_distances)
    precision_rms = np.sqrt(np.mean(precision_distances**2))

    print("======== Reverse EEF Statistics (Reverse-estimated end-effector trajectory) ========")
    print(f"[Accuracy] Reverse-estimated eef - actual eef mean distance: {accuracy_mean:.5f} m, RMS: {accuracy_rms:.5f} m")
    print(f"[Precision] Reverse-estimated eef relative to itself distribution (jitter) mean: {precision_mean:.5f} m, RMS: {precision_rms:.5f} m")
    return {
        "accuracy_mean": accuracy_mean,
        "accuracy_rms": accuracy_rms,
        "precision_mean": precision_mean,
        "precision_rms": precision_rms,
        "accuracy_distances": accuracy_distances,
        "precision_distances": precision_distances,
        "rel_reverse_pts": rel_reverse_pts
    }

# Calculate SLAM accuracy and precision relative to the theoretical position of the robotic arm
stats = cam_trajectory_statistics(
    eef_pose_vis,            # (N,9)
    cam_traj_actual_pts,     # (N,3)
    cam_traj_theoretical_pts # (N,3)
)
reverse_eef_statistics(eef_pose_vis, reverse_eef_pts)

ani = animation.FuncAnimation(fig, animate, frames=len(cam_traj_actual_pts), interval=60, blit=False)
plt.show()