import socket
import struct
import threading
import time
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from std_msgs.msg import Int32
import zmq

    
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

plt.ion()

class VRReceiver:
    def __init__(self, udp_ip, udp_port, timeout=10):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.timeout = timeout
        self.running = False
        self.latest_data = None
        self.key_history_size = 20
        self.key_history = {key: [0]*self.key_history_size for key in ['A', 'B', 'X', 'Y']}
        self.key_ptr = 0 
        self.thread = None
        self._init_sock()
        

    def _init_sock(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)
        self.sock.bind(('', 7777))
        
        message = b'Tick'
        self.sock.sendto(message, (self.udp_ip, self.udp_port))
        print('udp send')
        try:
            data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
            print(f"Received response from {addr}: {data}")
        except socket.timeout:
            print("No response received within timeout period.")
        
    def _parse_data(self, data: bytes):
        # format：'<q16f4i'，little-endian，1 int64, 16 float, 4 int
        try:
            timestamp, *poses, A, B, X, Y = struct.unpack('<q16f4i', data)
            return {
                'timestamp': timestamp,
                'left': poses[:8],
                'right': poses[8:],
                "A": A,
                "B": B,
                "X": X,
                "Y": Y
            }
        except Exception as e:
            return None

    def _receiver_loop(self):
        self.running = True    
        while self.running:
            try:
                # Clear socket buffer, only process the last packet
                latest_pack = None
                while True:
                    try:
                        # Non-blocking mode: immediately raise error when no packet is read
                        self.sock.setblocking(False)
                        udp_pack = self.sock.recv(1024)
                        latest_pack = udp_pack
                    except BlockingIOError:
                        break  # Buffer is empty
                    except socket.error:
                        break
                self.sock.setblocking(True)  # Restore blocking mode
                if latest_pack is not None:
                    res = self._parse_data(latest_pack)
                    if res is not None:
                        for key in ['A', 'B', 'X', 'Y']:
                            self.key_history[key][self.key_ptr] = int(bool(res[key]))
                        self.key_ptr = (self.key_ptr + 1) % self.key_history_size
                        self.latest_data = res
            except socket.timeout:
                print("[VRReceiver] UDP receive timeout, retrying...")
                continue
            except Exception as e:
                print(f"[VRReceiver] Exception: {e}")
                continue
            time.sleep(1/50)
        self.sock.close()

    def start(self):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1)

    def get(self):
        result = self.latest_data.copy() if self.latest_data else None
        
        if result:
            for key in ['A', 'B', 'X', 'Y']:
                if any(self.key_history[key]):
                    result[key] = 1
                else:
                    result[key] = 0
        return result

class VRNode(Node):
    def __init__(self, vr_receiver, base_xyz0, base_q0):
        super().__init__('vr_teleop_node')
        self.receiver = vr_receiver  # VRReceiver instance
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://*:5604")  # Bind to all network interfaces on port 5604
        self.pub_left = self.create_publisher(JointState, '/left/joint_targets', 10)
        self.pub_right = self.create_publisher(JointState, '/right/joint_targets', 10)
        self.reset_mode_publisher = self.create_publisher(Int32, 'robot_reset', 10)
        self.prepare_mode_publisher = self.create_publisher(Int32, 'robot_prepare', 10)
        self.base_xyz0 = np.array(base_xyz0)
        self.base_q0 = R.from_quat(base_q0).as_matrix()
        
        # To be initialized
        self.left_xyz0, self.left_q0 = None, None
        self.right_xyz0, self.right_q0 = None, None
        self.timer = self.create_timer(1/30, self.main_loop)
        
        self.last_left_sent = None
        self.last_right_sent = None
        self.fig_ax = (None, None) 
        self.R_adjust = np.array([
            [0,0,1],
            [0,1,0],
            [-1,0,0]
        ])
        self.align_matrix = np.array([
            [0, -1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ])
        self.align_matrix_left = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, 1, 0]
        ])

        self.indicators = {
            'reset': False,
            'send': False,
            'post_send': True,
            'n': False,
            'r': False,
            's': False
        }
        
    def _to_right_hand(self, quat):
        # Assume original comes from left hand (w, x, y, z), adjust signs as per actual device
        w, x, y, z = quat
        return [w, x, -y, -z]

    def _quaternion_mult(self, q1, q2):
        # scipy Rotation can be directly left-multiplied
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        return (r1 * r2).as_quat()

    def main_loop(self):
        data = self.receiver.get()
        if data is None: return
        
        reset_indicator = data['B'] == 1  # Press B to reset zero point
        if self.indicators['reset'] and (not reset_indicator):
            self.left_xyz0, self.left_q0 = None, None
            self.right_xyz0, self.right_q0 = None, None
            self.indicators['send'] = True
            self.indicators['post_send'] = False
            self.indicators['n'] = False
            self.indicators['r'] = False
            self.indicators['s'] = False
            self.get_logger().info("Reset left/right arm zero points.")
            
        send_indicator = data['A'] == 1  # Press A to send data
        if self.indicators['send'] and send_indicator:
            self.indicators['send'] = False
        elif (not self.indicators['send']):
            if (not self.indicators['post_send']) and (not send_indicator):
                self.indicators['post_send'] = True
            elif self.indicators['post_send']:
                if self.indicators['n'] == False and send_indicator:
                    self.zmq_socket.send_string("press:n")
                    self.get_logger().info("Simulated 'n' key press.")
                elif self.indicators['n'] == True and (not send_indicator):
                    self.zmq_socket.send_string("release:n")
                    self.get_logger().info("Simulated 'n' key release.")
                self.indicators['n'] = send_indicator
                    
        self.indicators['reset'] = reset_indicator
        msg = Int32()
        msg.data = int(self.indicators['reset'])
        self.prepare_mode_publisher.publish(msg)
        
        stop_indicator = data['Y'] == 1  # Press Y to stop sending
        if stop_indicator:
            self.indicators['send'] = False
        msg_done = Int32()
        msg_done.data = int(stop_indicator)  # Press Y to stop sending
        self.reset_mode_publisher.publish(msg_done)
        
        s_indicator = data['X'] == 1
        if self.indicators['s'] and not s_indicator:
            self.zmq_socket.send_string("release:s")
            self.get_logger().info("Simulated 's' key release.")
            self.indicators['s'] = False
        elif self.indicators['r'] and not s_indicator:
            self.zmq_socket.send_string("release:r")
            self.get_logger().info("Simulated 'r' key release.")
            self.indicators['r'] = False
        elif self.indicators['post_send'] and (not self.indicators['r']) and s_indicator:
            self.zmq_socket.send_string("press:r")
            self.get_logger().info("Simulated 'r' key press.")
            self.indicators['r'] = True
        elif (not self.indicators['post_send']) and not self.indicators['s'] and s_indicator:
            self.zmq_socket.send_string("press:s")
            self.get_logger().info("Simulated 's' key press.")
            self.indicators['s'] = True
        
      
        
        if not self.indicators['send']:
            return
        
        now = self.get_clock().now().to_msg()
        try:
            for hand in ['left', 'right']:
                pose = data[hand]   # Length 8
                xyz, quat, width = pose[:3], pose[3:7], pose[7]
                xyz = np.array(xyz)
                quat = np.array(quat)
                # Initialize zero point
                rel_xyz = np.array(xyz) @ self.align_matrix
                if quat[-1] < 0:
                    quat = quat * -1
                    
                raw_rotation_matrix = R.from_quat(quat).as_matrix()
                rel_rotation_matrix = self.align_matrix_left @ raw_rotation_matrix @ np.linalg.inv(self.align_matrix_left) @ self.R_adjust \
                    @ np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
                
                gripper = 0.085 * width - 0.005
                js = JointState()
                js.header.stamp = now
                js.name = [f'joint_{i+1}' for i in range(7)]
                origin_xyz = self.left_xyz0 if hand == 'left' else self.right_xyz0
                if origin_xyz is None:
                    origin_xyz = np.array(rel_xyz)
                    if hand == 'left':
                        self.left_xyz0 = origin_xyz
                    else:
                        self.right_xyz0 = origin_xyz
                    print(f"Initialized {hand} arm origin at {origin_xyz}")
                    
                origin_q0 = self.left_q0 if hand == 'left' else self.right_q0
                if origin_q0 is None:
                    origin_q0 = R.from_quat(quat)
                    if hand == 'left':
                        self.left_q0 = origin_q0
                    else:
                        self.right_q0 = origin_q0
                    print(f"Initialized {hand} arm orientation at {quat}")
                
                r_out = R.from_matrix(rel_rotation_matrix).as_euler('xyz')

                    
                abs_xyz = np.array(rel_xyz) - origin_xyz + self.base_xyz0
                output = list(abs_xyz) + list(r_out) + [gripper]
                js.position = output
                if hand == 'left':
                    self.pub_left.publish(js)
                    self.last_left_sent = output
                else:
                    self.pub_right.publish(js)
                    self.last_right_sent = output
        except Exception as e:
            self.get_logger().error(f"Error in processing VR data: {e}")


def test_vr_receiver():
    import time
    local_ip = '192.168.1.7'
    local_port = 9577

    receiver = VRReceiver(local_ip, local_port)
    receiver.start()
    print('VRReceiver started. Waiting for UDP packets...')
    try:
        while True:
            data = receiver.get()
            if data:
                print(data['left'][-1], flush=True)
                
            time.sleep(0.05)
    except KeyboardInterrupt:
        print('Exiting.')
    receiver.stop()
    
def draw_axes(ax, Rmat, origin, length=0.05, label=None):
    """Draw three basis vectors on ax, columns of Rmat are x/y/z"""
    colors = ['r', 'g', 'b']
    for i in range(3):
        dir_vec = Rmat[:, i]
        ax.quiver(
            origin[0], origin[1], origin[2],
            dir_vec[0], dir_vec[1], dir_vec[2],
            length=length, color=colors[i], linewidth=2, arrow_length_ratio=0.15
        )
    if label:
        ax.text(*origin, label, fontsize=10, color='k')

def visualize_arm_pose(last_left, last_right, fig_ax=None,
                      xyz_range=((-1.0, 1.0), (-1.0, 1.0), (0.0, 1.5))):
    """
    last_left/right: 7维(x, y, z, rx, ry, rz, gripper)
    xyz_range: ((xmin, xmax), (ymin, ymax), (zmin, zmax))，如为None则自动缩放
    """
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from scipy.spatial.transform import Rotation as R

    def draw_axes(ax, Rmat, origin, length=0.05, label=None):
        colors = ['r', 'g', 'b']
        for i in range(3):
            dir_vec = Rmat[:, i]
            ax.quiver(
                origin[0], origin[1], origin[2],
                dir_vec[0], dir_vec[1], dir_vec[2],
                length=length, color=colors[i], linewidth=2, arrow_length_ratio=0.15
            )
        if label:
            ax.text(*origin, label, fontsize=10, color='k')

    if fig_ax is None or fig_ax[0] is None:
        plt.ion()
        fig = plt.figure("Final positions", figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        fig_ax = (fig, ax)
    else:
        fig, ax = fig_ax

    ax.cla()
    for pose, name, col in zip([last_left, last_right], ['Left', 'Right'], ['blue', 'orange']):
        xyz = np.array(pose[:3])
        rxyz = pose[3:6]
        Rmat = R.from_euler('xyz', rxyz).as_matrix()
        ax.scatter(*xyz, color=col, label=f"{name} Pos")
        draw_axes(ax, Rmat, xyz, length=0.08, label=name)
        ax.text(*xyz, f"{name}-{pose[6]:.3f}", color=col)
    if xyz_range is not None:
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = xyz_range
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_zlim(zmin, zmax)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('Final Puppet Positions')
    ax.set_box_aspect([1, 1, 1])
    ax.legend()
    plt.draw()
    plt.pause(0.01)
    return fig_ax
    
if __name__ == "__main__":
    # Initialize base_xyz0, base_q0
    base_xyz0 = [0.0, 0.0, 0.0]     # Actual position of the robot arm base end-effector xyz
    base_q0 = R.from_euler('xyz', [0, 90, 0], degrees=True).as_quat()            # Initial quaternion, wxyz
    # Replace the following port with your actual address
    local_ip = '192.168.1.7'
    local_port = 9577
    vr_receiver = VRReceiver(local_ip, local_port)   # Fill in IP and port as needed
    vr_receiver.start()
    rclpy.init()
    node = VRNode(vr_receiver, base_xyz0, base_q0)
    rclpy.spin(node)
    vr_receiver.stop()