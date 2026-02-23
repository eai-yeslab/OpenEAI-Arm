import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from collections import deque
import cv2
from cv_bridge import CvBridge
import yaml
from pynput import keyboard


class OpenEAIArmROS2Operator:
    def __init__(self, config_path):
        self.config = self._load_config(config_path)
        self.img_deques = {}
        self.arm_deques = {}
        self.arm_keys = []
        self.base_deque = deque()
        self.arm_publishers = {}
        self.base_publisher = None
        self.bridge = CvBridge()
        self.ctrl_state = False
        self.ctrl_state_lock = threading.Lock()
        self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_lock = threading.Lock()
        self._lock = threading.Lock()
        self.executor = None
        self.node = None
        self._init_data_structures()
        self.init()
        self.init_ros()

    def _load_config(self, path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _init_data_structures(self):
        # Cameras
        for cam_name, cam_conf in self.config.get('cameras', {}).items():
            self.img_deques[cam_name] = {}
            if 'rgb_topic' in cam_conf:
                self.img_deques[cam_name]['rgb'] = deque()
            if self.config.get('use_depth_image', False) and 'depth_topic' in cam_conf:
                self.img_deques[cam_name]['depth'] = deque()
        # Arms
        for arm_name, arm_conf in self.config.get('arms', {}).items():
            self.arm_deques[arm_name] = deque()
            self.arm_keys.append(arm_name)
        # Gripper/pose/extra
        self.extra_deques = {
            'gripper_joint': deque(),
            'arm_pose': deque(),
        }

    def init(self):
        self.puppet_arm_publish_lock.acquire()


    def puppet_arm_publish(self, arm_name, target):
        keys = [key for key in self.arm_keys if arm_name in key]
        n_arms = len(keys)
        dof = 7
        n = min(len(target) // dof, n_arms)
        for i in range(n):
            sub_target = target[i*dof:(i+1)*dof]
            sub_arm_name = keys[i]
            if sub_arm_name not in self.arm_publishers:
                continue
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
            joint_state_msg.name = [f'joint_{k+1}' for k in range(dof)]
            joint_state_msg.position = list(sub_target)
            self.arm_publishers[sub_arm_name].publish(joint_state_msg)
            self.node.get_logger().info(
                f"Publish {sub_arm_name} joint targets: {sub_target}"
            )

    def robot_base_publish(self, vel):
        if not self.base_publisher:
            return
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = vel[1]
        self.base_publisher.publish(vel_msg)

    def puppet_arm_publish_continuous(self, arm_name, target):
        rate = self.node.create_rate(self.config['frame_rate'])
        keys = [key for key in self.arm_keys if arm_name in key]
        n_arms = len(keys)
        dof = 7
        n = min(len(target) // dof, n_arms)
        current_arms = []
        for i in range(n):
            sub_arm_name = keys[i]
            while rclpy.ok():
                deque_ref = self.arm_deques[sub_arm_name]
                if len(deque_ref) != 0:
                    arm = list(deque_ref[-1].position)
                    current_arms.append(arm)
                    break
                self.node.get_logger().info(f"Waiting for {sub_arm_name} current arm pose...")
                rate.sleep()
        targets = [target[i*dof:(i+1)*dof] for i in range(n)]

        arm_symbols = []
        for i in range(n):
            arm_symbols.append([1 if targets[i][j] - current_arms[i][j] > 0 else -1 for j in range(dof)])
        flag = True
        steps = 0
        step_size = self.config['arm_steps_length']
        while flag and rclpy.ok():
            if self.puppet_arm_publish_lock.acquire(False):
                return
            flag = False
            for i in range(n):
                arm = current_arms[i]
                tgt = targets[i]
                symbol = arm_symbols[i]
                left_diff = [abs(tgt[j] - arm[j]) for j in range(dof)]
                arm_done = True
                for j in range(dof):
                    if left_diff[j] < step_size[j]:
                        arm[j] = tgt[j]
                    else:
                        arm[j] += symbol[j] * step_size[j]
                        arm_done = False
                sub_arm_name = keys[i]
                joint_state_msg = JointState()
                joint_state_msg.header = Header()
                joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
                joint_state_msg.name = [f'joint_{k+1}' for k in range(dof)]
                joint_state_msg.position = arm
                self.arm_publishers[sub_arm_name].publish(joint_state_msg)
                current_arms[i] = arm
                if not arm_done:
                    flag = True
            steps += 1
            print(f"{arm_name} puppet_arm_publish_continuous: {steps}")
            rate.sleep()

    def run_executor(self):
        self.node.get_logger().info("Running executor...")
        self.executor.spin()

    def _to_sec(self, msg):
        stamp = msg.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def get_frame(self, required = ['imgs', 'arms', 'robot_base']):
        frame_time_candidates = []
        if 'imgs' in required:
            for cam_name, types in self.img_deques.items():
                for typ, dq in types.items():
                    if len(dq) == 0:
                        self.node.get_logger().info(f"{cam_name} {typ} image info is missing...")
                        return False
                    frame_time_candidates.append(self._to_sec(dq[-1]))
                    
        if 'arms' in required:
            for arm_name, dq in self.arm_deques.items():
                if len(dq) == 0:
                    self.node.get_logger().info(f"{arm_name} arm pose info missing...")
                    return False
                frame_time_candidates.append(self._to_sec(dq[-1]))
        if 'robot_base' in required and self.config.get('use_robot_base', False) and len(self.base_deque) == 0:
            return False
        if frame_time_candidates:
            frame_time = min(frame_time_candidates)
        else:
            return False

        def pop_until(dq):
            while len(dq) > 0 and self._to_sec(dq[0]) < frame_time:
                dq.popleft()
            return dq.popleft() if len(dq) > 0 else None

        imgs = {}
        if 'imgs' in required:
            for cam_name, types in self.img_deques.items():
                imgs[cam_name] = {}
                for typ, dq in types.items():
                    msg = pop_until(dq)
                    if msg is None:
                        self.node.get_logger().info(f"Waiting for {cam_name} {typ} image...")
                        return False
                    img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                    if img.shape[-1] == 2:
                        img = cv2.cvtColor(img, cv2.COLOR_YUV2RGB_YUY2)
                    imgs[cam_name][typ] = img

        arms = {}
        if 'arms' in required:
            for arm_name, dq in self.arm_deques.items():
                arms[arm_name] = pop_until(dq)
                if arms[arm_name] is None:
                    self.node.get_logger().info(f"Waiting for {arm_name} arm pose...")
                    return False
        robot_base_msg = None
        if 'robot_base' in required and self.config.get('use_robot_base', False):
            robot_base_msg = pop_until(self.base_deque)

        # return {
        #     'imgs': imgs,         # {'hand': {'rgb':..., 'depth': ...}, ...}
        #     'arm_keys': self.arm_keys,  # ['puppet', ...]
        #     'arms': arms,         # {'puppet': jointstate_msg, ...}
        #     'robot_base': robot_base_msg,
        #     'timestamp': frame_time,
        # }
        result = {}
        if 'imgs' in required:
            result['imgs'] = imgs
        if 'arms' in required:
            result['arm_keys'] = self.arm_keys
            result['arms'] = arms
        if 'robot_base' in required and self.config.get('use_robot_base', False):
            result['robot_base'] = robot_base_msg
        result['timestamp'] = frame_time
        return result

    # ------- ROS2 callback bindings --------
    def _make_image_callback(self, cam_name, typ):
        def cb(msg):
            dq = self.img_deques[cam_name][typ]
            if len(dq) >= 2000:
                dq.popleft()
            dq.append(msg)
        return cb

    def _make_arm_callback(self, arm_name):
        def cb(msg):
            dq = self.arm_deques[arm_name]
            if len(dq) >= 2000:
                dq.popleft()
            dq.append(msg)
        return cb

    def robot_base_callback(self, msg):
        if len(self.base_deque) >= 2000:
            self.base_deque.popleft()
        self.base_deque.append(msg)

    def ctrl_callback(self, msg):
        self.ctrl_state_lock.acquire()
        self.ctrl_state = msg.data
        self.ctrl_state_lock.release()

    def get_ctrl_state(self):
        self.ctrl_state_lock.acquire()
        state = self.ctrl_state
        self.ctrl_state_lock.release()
        return state

    def init_ros(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('OpenEAIArm')
        # Cameras: dynamic subscriptions
        for cam_name, cam_conf in self.config.get('cameras', {}).items():
            if 'rgb_topic' in cam_conf:
                self.node.create_subscription(
                    Image, cam_conf['rgb_topic'],
                    self._make_image_callback(cam_name, 'rgb'), 10
                )
            if self.config.get('use_depth_image', False) and 'depth_topic' in cam_conf:
                self.node.create_subscription(
                    Image, cam_conf['depth_topic'],
                    self._make_image_callback(cam_name, 'depth'), 10
                )
        # Arms: dynamic subscriptions/publications
        for arm_name, arm_conf in self.config.get('arms', {}).items():
            # Subscribe to receive joint states
            if 'recv_joint_state_topic' in arm_conf:
                self.node.create_subscription(
                    JointState, arm_conf['recv_joint_state_topic'],
                    self._make_arm_callback(arm_name), 10
                )
            # Publish joint commands
            if 'send_joint_cmd_topic' in arm_conf:
                self.arm_publishers[arm_name] = self.node.create_publisher(
                    JointState, arm_conf['send_joint_cmd_topic'], 10
                )
            if 'recv_eef_pose_topic' in arm_conf:
                self.arm_deques[arm_name + '_pose'] = deque()
                self.node.create_subscription(
                    JointState, arm_conf['recv_eef_pose_topic'],
                    self._make_arm_callback(arm_name + '_pose'), 10
                )
        # Robot base
        if self.config.get('use_robot_base', False):
            base_conf = self.config.get('robot_base', {})
            if 'recv_topic' in base_conf:
                self.node.create_subscription(
                    Twist, base_conf['recv_topic'],
                    self.robot_base_callback, 10
                )
            if 'send_topic' in base_conf:
                self.base_publisher = self.node.create_publisher(
                    Twist, base_conf['send_topic'], 10
                )
        # Control
        if 'ctrl_topic' in self.config:
            self.node.create_subscription(
                Bool, self.config['ctrl_topic'], self.ctrl_callback, 10
            )
        
        self.prepare_publisher = self.node.create_publisher(
            Int32, self.config.get('prepare_topic', '/robot_prepare'), 10
        )
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.run_executor, daemon=True)
        self.executor_thread.start()
        self.node.get_logger().info(f"RosOperator model_inferencer initialized.")


# Usage example (pass yaml path at startup)
# op = OpenEAIArmROS2Operator('config.yaml')