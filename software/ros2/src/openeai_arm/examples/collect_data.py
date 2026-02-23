import argparse
import yaml
import time
import os
import numpy as np
import collections
import h5py
import rclpy
import dm_env
from pynput import keyboard
import glob
import threading
import zmq

from OpenEAIArmROS2Operator import OpenEAIArmROS2Operator

class OpenEAIArmDataCollector:
    def __init__(self, args):
        with open(args.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.args = args
        if hasattr(args, "camera_names") and args.camera_names is not None:
            self.camera_names = args.camera_names
        elif 'cameras' in self.config:
            self.camera_names = list(self.config['cameras'].keys())
        else:
            self.camera_names = []
        self.args.camera_names = self.camera_names
        self.operator = OpenEAIArmROS2Operator(args.config_path)
        self.node = self.operator.node

        self.running = False
        self.next_episode = None
        self.episode_idx = args.episode_idx if args.episode_idx is not None else len(glob.glob(f"{args.dataset_dir}/{args.task_name}/*.hdf5"))
        if args.episode_idx is None:
            self.node.get_logger().info(f"Starting from episode_{self.episode_idx} as args.episode_idx is None and there are {self.episode_idx} hdf5 files found in {args.dataset_dir}/{args.task_name}")
        else:
            self.node.get_logger().info(f"Starting from episode_{self.episode_idx} as args.episode_idx is set to {self.episode_idx}")
        
        self.timestamps = None
        self.actions = None
        self.missing_info_frame_count = 0

        # Keyboard listener
        self._key_states = {chr(i): False for i in range(ord('a'), ord('z') + 1)}
        self._key_states.update({str(i): False for i in range(10)})
        self._lock = threading.Lock()
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        self.init_zmq()

    def _on_press(self, key):
        try:
            with self._lock:
                key_char = key.char.lower()
                self._key_states[key_char] = True
                self.node.get_logger().info(f"Button {key} pressed")
        except AttributeError:
            pass

    def _on_release(self, key):
        try: 
            with self._lock:
                key_char = key.char.lower()
                self._key_states[key_char] = False
                self.node.get_logger().info(f"Button {key} released")
                self._handle_key(key_char)
        except AttributeError:
            pass
        
    def _handle_key(self, key_char):                
            if key_char == 's':
                self.running = not self.running
                self.node.get_logger().info("\033[34mRecording {}\033[30m".format("\033[32mEnabled" if self.running else "\033[31mDisabled"))
            elif key_char == 'n':
                self.next_episode = True
                self.node.get_logger().info("\033[32mSaving data for episode {}\033[30m".format(self.episode_idx))
                self.running = False
            elif key_char == 'r':
                self.next_episode = False
                self.node.get_logger().info("\033[31mRestart recording data for episode {}\033[30m".format(self.episode_idx))
                self.running = False


    def init_zmq(self):
        self.zmq_port = self.config.get('zmq_port', 5604)
        self.zmq_running = True
        
        # Initialize ZeroMQ
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.connect(f"tcp://localhost:{self.zmq_port}")
        self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.node.get_logger().info(f"ZeroMQ subscriber connected to port {self.zmq_port}")
        
        # Start ZeroMQ receiving thread
        self.zmq_thread = threading.Thread(target=self._receive_zmq_events, daemon=True)
        self.zmq_thread.start()
        
    def _receive_zmq_events(self):
        """Receive ZeroMQ virtual key events"""
        while self.zmq_running:
            try:
                # 100ms timeout to avoid blocking
                if self.zmq_socket.poll(100):
                    message = self.zmq_socket.recv_string()
                    self._handle_zmq_key(message)
            except:
                break

    def _handle_zmq_key(self, message):
        """Handle ZeroMQ key messages"""
        print(f"Received ZMQ message: {message}")
        try:
            # Handle message formats: "press:s", "release:s" or just "s"
            if ':' in message:
                action, key = message.split(':')
                key = key.lower()
                
                with self._lock:
                    if action == 'press':
                        self._key_states[key] = True
                        self.node.get_logger().info(f"[ZMQ] Button {key} pressed")
                    elif action == 'release':
                        self._key_states[key] = False
                        self.node.get_logger().info(f"[ZMQ] Button {key} released")
                        self._handle_key(key)
            else:
                key = message.lower()
                with self._lock:
                    self._key_states[key] = True
                    self.node.get_logger().info(f"[ZMQ] Button {key} pressed")
                    self._key_states[key] = False
                    self.node.get_logger().info(f"[ZMQ] Button {key} released")
                    self._handle_key(key)
                    
        except Exception as e:
            self.node.get_logger().error(f"Failed to handle ZeroMQ message: {e}")

    def get_frame(self):
        """
        Calling operator.get_frame returns an available frame,
        only extracting data for the master arm and puppet arm.
        """
        result = self.operator.get_frame()
        if not result:
            return False

        imgs = result['imgs']
        arms = result['arms']
        robot_base = result.get('robot_base', None)
        # Dynamically unpack master/puppet arms
        try:
            puppet_arm = [arms[key] for key in result['arm_keys'] if 'puppet' in key]
            master_arm = [arms[key] for key in result['arm_keys'] if 'master' in key]
            puppet_arm_pose = [arms[key+'_pose'] for key in result['arm_keys'] if 'puppet' in key and key+'_pose' in arms]
        except Exception as e:
            self.node.get_logger().warning("Frame construct failed: {}".format(e))
            return False
        timestamp = result['timestamp']
        return (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp)

    def process(self):
        if self.config.get('use_depth_image', False):
            self.node.get_logger().info("Depth information enabled.")
        while True:
            timesteps = []
            actions = []
            # Image data
            image = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8)
            image_dict = dict()
            for cam_name in self.args.camera_names:
                image_dict[cam_name] = image
            count = 0

            print_flag = True
            self.missing_info_frame_count = 0
            while rclpy.ok() and self.next_episode is None:
                tick = time.time()
                if not self.running:
                    tock = time.time()
                    time.sleep(max(0, 1 / self.args.frame_rate - (tock - tick)))
                    continue
                # 2 Collect data
                result = self.get_frame()
                if not result:
                    self.missing_info_frame_count += 1
                    if print_flag:
                        self.node.get_logger().warning("syn fail")
                        print_flag = False
                    if self.missing_info_frame_count >= 20:
                        self.next_episode = False
                        self.node.get_logger().info("\033[31mDiscard recording data for episode {} due to long-time missing data\033[30m".format(self.episode_idx))
                        self.running = False
                    tock = time.time()
                    time.sleep(max(0, 1 / self.args.frame_rate - (tock - tick)))
                    continue
                else:
                    self.missing_info_frame_count = 0
                print_flag = True
                count += 1
                (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp) = result
                delay = time.time() - timestamp
                obs = collections.OrderedDict() 
                obs['images'] = imgs
                
                qpos = np.zeros(0)
                qvel = np.zeros(0)
                effort = np.zeros(0)
                eef_pose = np.zeros(0)
                for arm in puppet_arm:
                    qpos_raw = arm.position
                    joint_names = np.array(arm.name)
                    qpos_arm = np.zeros(7)
                    for i in range(1, 8):
                        j_name = f'joint_{i}'
                        idx = np.where(joint_names == j_name)[0][0]
                        qpos_arm[i - 1] = qpos_raw[idx]
                    qpos = np.concatenate((qpos, qpos_arm), axis=0)
                    qvel = np.concatenate((qvel, np.array(arm.velocity)), axis=0)
                    effort =  np.concatenate((effort, np.array(arm.effort)), axis=0)
                for arm_pose in puppet_arm_pose:
                    eef_pose = np.concatenate((eef_pose, np.array(arm_pose.position)), axis=0)
                obs['qpos'] = qpos
                obs['qvel'] = qvel
                obs['effort'] = effort
                obs['eef_pose'] = eef_pose                
                
                obs['timestamp'] = timestamp
                
                if count == 1:
                    ts = dm_env.TimeStep(
                        step_type=dm_env.StepType.FIRST,
                        reward=None,
                        discount=None,
                        observation=obs)
                    timesteps.append(ts)
                    continue

                ts = dm_env.TimeStep(
                    step_type=dm_env.StepType.MID,
                    reward=None,
                    discount=None,
                    observation=obs)

                action = np.zeros(0)
                for arm in master_arm:
                    action = np.concatenate((action, np.array(arm.position)), axis=0)
                actions.append(action)
                timesteps.append(ts)
                self.node.get_logger().info(f"Frame data: {count} / +{delay*1000:.1f} ms")
                if not rclpy.ok():
                    exit(-1)
                tock = time.time()
                time.sleep(max(0, 1 / self.args.frame_rate - (tock - tick)))

            if self.next_episode and len(timesteps) > 0:
                self.node.get_logger().info(f"len(timesteps): {len(timesteps)}")
                self.node.get_logger().info(f"len(actions)  : {len(actions)}")
                self.timestamps = timesteps
                self.actions = actions
                self.save_data()
                self.episode_idx += 1
            self.next_episode = None
    
    def save_data(self):
        if self.actions is None or self.timestamps is None:
            return
        actions = self.actions
        timesteps = self.timestamps
        dataset_dir = os.path.join(self.args.dataset_dir, self.args.task_name)
        if not os.path.exists(dataset_dir):
            os.makedirs(dataset_dir)
        dataset_path = os.path.join(dataset_dir, "episode_" + str(self.episode_idx))
        data_size = len(actions)

        data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/observations/effort': [],
            '/observations/eef_pose': [],
            '/action': [],
            '/timestamps': []
        }
        for cam_name in self.args.camera_names:
            data_dict[f'/observations/images/cam_{cam_name}'] = []
            if self.config['use_depth_image']:
                data_dict[f'/observations/images_depth/cam_{cam_name}'] = []
                
        action_dim = len(actions[0])
        state_dim = len(timesteps[1].observation['qpos'])
        eef_dim = len(timesteps[1].observation['eef_pose'])

        while actions:
            action = actions.pop(0)
            ts = timesteps.pop(0)
            data_dict['/timestamps'].append(ts.observation['timestamp'])
            data_dict['/observations/qpos'].append(ts.observation['qpos'])
            data_dict['/observations/qvel'].append(ts.observation['qvel'])
            data_dict['/observations/effort'].append(ts.observation['effort'])
            data_dict['/observations/eef_pose'].append(ts.observation['eef_pose'])
            
            # Fix data: find frames where qvel or effort length is 0 and replace with the previous frame's value
            if (len(ts.observation['qvel']) == 0 or len(ts.observation['effort']) == 0):
                self.node.get_logger().info("Amending data at frame {}".format(len(data_dict['/observations/qvel'])))
                if len(data_dict['/observations/qvel']) > 1:
                    data_dict['/observations/qpos'][-1] = data_dict['/observations/qpos'][-2]
                    data_dict['/observations/qvel'][-1] = data_dict['/observations/qvel'][-2]
                    data_dict['/observations/effort'][-1] = data_dict['/observations/effort'][-2]
                    data_dict['/observations/eef_pose'][-1] = data_dict['/observations/eef_pose'][-2]
                else:
                    data_dict['/observations/qpos'][-1] = data_dict['/observations/qpos'][0]
                    data_dict['/observations/qvel'][-1] = np.zeros_like(data_dict['/observations/qpos'][0])
                    data_dict['/observations/effort'][-1] = np.zeros_like(data_dict['/observations/qpos'][0])
                    data_dict['/observations/eef_pose'][-1] = np.zeros_like(data_dict['/observations/qpos'][0])
            data_dict['/action'].append(action)

            for cam_name in self.args.camera_names:
                data_dict[f'/observations/images/cam_{cam_name}'].append(ts.observation['images'][cam_name]['rgb'])
                if self.config['use_depth_image']:
                    data_dict[f'/observations/images_depth/cam_{cam_name}'].append(ts.observation['images'][cam_name]['depth'])

        t0 = time.time()
        image_shape = self.args.image_shape.split("x")
        image_width, image_height = int(image_shape[0]), int(image_shape[1])
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            root.attrs['compress'] = False
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for cam_name in self.args.camera_names:
                _ = image.create_dataset(f"cam_{cam_name}", (data_size, image_height, image_width, 3), dtype='uint8',
                                         chunks=(1, image_height, image_width, 3))
            if self.config['use_depth_image']:
                image_depth = obs.create_group('images_depth')
                for cam_name in self.args.camera_names:
                    _ = image_depth.create_dataset(f"cam_{cam_name}", (data_size, image_height, image_width), dtype='uint16',
                                                   chunks=(1, image_height, image_width))
            _ = obs.create_dataset('qpos', (data_size, state_dim))
            _ = obs.create_dataset('qvel', (data_size, state_dim))
            _ = obs.create_dataset('effort', (data_size, state_dim))
            _ = obs.create_dataset('eef_pose', (data_size, eef_dim))
            _ = root.create_dataset('action', (data_size, action_dim))
            _ = root.create_dataset('timestamps', (data_size,), dtype='float64')

            for name, array in data_dict.items():
                root[name][...] = array
            self.node.get_logger().info(f'\033[32m\nSaving: {time.time() - t0:.1f} secs. %s \033[0m\n' % dataset_path)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--dataset_dir', action='store', type=str, help='Dataset_dir.',
                        default="./data", required=False)
    parser.add_argument('-n', '--task_name', action='store', type=str, help='Task name.',
                        default="aloha_mobile_dummy", required=False)
    parser.add_argument('-i', '--episode_idx', action='store', type=int, help='Episode index.',
                        default=None, required=False)
    parser.add_argument('--camera_names', nargs="+", default=None, help="Camera names list, overrides config order")
    parser.add_argument('--frame_rate', type=int, default=20, help="Frame rate")
    parser.add_argument('--image_shape', type=str, default="640x480", help="Image shape WxH")
    parser.add_argument('--use_depth_image', action='store_true')
    parser.add_argument('--config_path', type=str, default="config_single.yml", help="OpenEAIArmROS2Operator config path")
    return parser.parse_args()


# 入口用法

if __name__ == '__main__':
    args = get_arguments()
    collector = OpenEAIArmDataCollector(args)
    collector.process()