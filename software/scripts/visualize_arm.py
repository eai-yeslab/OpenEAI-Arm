import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ====== DH参数和变换函数 ======
theta=14.927
# theta, d, a, alpha

mdh_params = [
    [0,         71.3+45.96, 0,          0],
    [180,       0,         19,         -90],
    [180+theta, 0,          264,        180],
    [-theta,    0,          233.89,     0],
    [90,        46.5-48.06,      75,         90],
    [0,         35.95,      0,          90],
    [0,         176.04,          0,          0]
]


mdh = [
    [np.radians(mdh_params[0][0]), mdh_params[0][1], mdh_params[0][2], np.radians(mdh_params[0][3])],
    [np.radians(mdh_params[1][0]), mdh_params[1][1], mdh_params[1][2], np.radians(mdh_params[1][3])],
    [np.radians(mdh_params[2][0]), mdh_params[2][1], mdh_params[2][2], np.radians(mdh_params[2][3])],
    [np.radians(mdh_params[3][0]), mdh_params[3][1], mdh_params[3][2], np.radians(mdh_params[3][3])],
    [np.radians(mdh_params[4][0]), mdh_params[4][1], mdh_params[4][2], np.radians(mdh_params[4][3])],
    [np.radians(mdh_params[5][0]), mdh_params[5][1], mdh_params[5][2], np.radians(mdh_params[5][3])],
    [np.radians(mdh_params[6][0]), mdh_params[6][1], mdh_params[6][2], np.radians(mdh_params[6][3])]
]

def mdh_matrix(theta, d, a, alpha):
    """生成MDH变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct,    -st,     0,     a],
        [st*ca,  ct*ca, -sa, -sa*d],
        [st*sa,  ct*sa,  ca,  ca*d],
        [0,      0,     0,     1]
    ])
    

def fk(joints):
    T = np.eye(4)
    points = [T[:3, 3]]
    transforms = [T.copy()]
    for i in range(len(mdh)):
        theta = joints[i] + mdh[i][0]
        d = mdh[i][1]
        a = mdh[i][2]
        alpha = mdh[i][3]
        T = T @ mdh_matrix(theta, d, a, alpha)
        points.append(T[:3, 3])
        transforms.append(T.copy())
    return np.array(points), transforms


# ========== 可视化函数 ==========
    
def plot_arm(joints, ax=None, show=True, length=50):
    points, transforms = fk(joints)
    for i, point in enumerate(points):
        print(f"Joint {i}:", point)
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:,0], points[:,1], points[:,2], '-o', color='k')

    # 绘制所有点的方向坐标系
    colors = ['r','g','b']
    for i, T in enumerate(transforms):
        origin = T[:3,3]
        for k in range(3):  # x,y,z
            dir_vec = T[:3,k]
            ax.quiver(
                origin[0], origin[1], origin[2],
                dir_vec[0], dir_vec[1], dir_vec[2],
                length=length, color=colors[k], arrow_length_ratio=0.2, linewidth=1
            )

    # 设定比例
    xs, ys, zs = points[:,0], points[:,1], points[:,2]
    max_range = max(xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min())
    x_mid, y_mid, z_mid = [np.mean(arr) for arr in [xs, ys, zs]]
    ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
    ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
    ax.set_zlim(z_mid - max_range/2, z_mid + max_range/2)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    try: ax.set_box_aspect([1,1,1])
    except: pass
    if show: plt.show()

# ========== 动画函数 ==========

def animate_arm(joint_traj, interval=50, length=50):
    """
    joint_traj: shape (N, 6)
    dh: DH参数，需要和fk配套
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 各帧的所有关节位置
    all_points = np.array([fk(j)[0] for j in joint_traj])
    xyz_min = np.min(all_points, axis=(0, 1)) 
    xyz_max = np.max(all_points, axis=(0, 1)) 
    d = xyz_max - xyz_min
    xyz_mid = (xyz_max + xyz_min) / 2
    max_range = np.max(d) / 2
    for dim in range(3):
        getattr(ax, f"set_{['x','y','z'][dim]}lim")(xyz_mid[dim]-max_range, xyz_mid[dim]+max_range)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Arm animation')

    # 初始显示
    points, transforms = fk(joint_traj[0])
    line, = ax.plot(points[:,0], points[:,1], points[:,2], '-o', lw=2)

    quivers = []
    colors = ['r','g','b']

    def update(frame):
        nonlocal quivers
        # 清理上一帧箭头
        for q in quivers:
            q.remove()
        quivers = []

        points, transforms = fk(joint_traj[frame])
        line.set_data(points[:, 0], points[:, 1])
        line.set_3d_properties(points[:, 2])

        # 每个关节画坐标轴
        for T in transforms:
            origin = T[:3,3]
            for k in range(3):
                dir_vec = T[:3,k]
                qv = ax.quiver(
                    origin[0], origin[1], origin[2],
                    dir_vec[0], dir_vec[1], dir_vec[2],
                    length=length, color=colors[k], arrow_length_ratio=0.2, linewidth=1
                )
                quivers.append(qv)
        return (line,)+tuple(quivers)
 
    ani = FuncAnimation(fig, update, frames=len(joint_traj), interval=interval, blit=False)
    plt.show()
    return ani

# ========== 示例：单帧调用 ==========
if __name__ == "__main__":
    reset_pose = np.array([-1.46, -1.363, -0.18, 0.226, 0.60, 2.60, -1.167])
#     joints = np.array([-0.00212508,     2.11126  ,   1.77926  ,  0.332004 ,-0.00211878 ,3.56378e-06, 0])
#     joints = np.array([1.0, 0.5, 1.5, -0.5, 0.67, 0.44, -0.5])
#     joints = np.array([0.0167, 1.0943, 1.4265, 0.2726, 0.1494, -0.0096, -0.0263, ])
    joints = np.zeros(7)
    plot_arm(joints)
    # joints = np.array([0.0846, 0.3242, 1.3508, -1.8726, 0.0106, -0.0107, 0.0002])
    # plot_arm(joints)
    # joints = np.array([0.0079, 1.2566, 1.1532, -0.5378, 0.0106, -0.0107, 0.0002])
    # plot_arm(joints)
    joints = np.array([-1.45666,-0.0019536,-0.0205135,-0.00782013,-0.0019536,-4.77949,1.19993,-0.0019536,6.66667,0.564393,-0.00732613,1.57021,0.610551,-0.00732613,-0.00732613,2.61139,-0.00732613,-0.00244236,-1.16331,0.00732803,-0.00244236,])[::3]
    plot_arm(joints - reset_pose)
    joints = np.array([-1.4559,-0.0019536,-0.00683784,-0.902761,-0.0019536,4.34188,1.20108,-0.0019536,3.5624,-0.590333,-0.0219784,1.69719,0.610169,-0.0219784,-0.00732613,2.61177,-0.00732613,-0.00732613,-1.16331,-0.0219784,-0.00244236,])[::3]
    plot_arm(joints - reset_pose)
    joints = np.array([-2.23945,-0.0019536,-0.00683784,0.734531,-0.0019536,-11.2615,1.8389,-0.00586081,6.78975,0.67998,-0.00732613,1.5409,0.80167,-0.00732613,-0.0512819,2.54349,-0.0219784,-0.00244236,-1.16331,-0.00732613,-0.00732613,])[::3]
    plot_arm(joints - reset_pose)

    # ========== 示例：产生动画轨迹 ==========
    # N = 100
    # t = np.linspace(0, 2*np.pi, N)
    # # 前3关节做周期运动，后3保持0
    # joint_traj = np.array([[
    #     0.5*np.sin(tt), 0.4*np.sin(1.5*tt), 0.3*np.cos(2*tt), 0, 0, 0, 0
    # ] for tt in t])
    # animate_arm(joint_traj)