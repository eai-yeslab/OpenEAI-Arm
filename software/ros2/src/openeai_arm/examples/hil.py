"""Human-in-the-loop entry point: dual-arm inference with model <-> teaching mode switching.

This is the HIL / teleoperation script (formerly ``rollout.py``). For autonomous
asynchronous rollout collection see ``rollout.py``.
"""
import torch
import numpy as np
import os
import pickle
import argparse
from einops import rearrange
import time
import threading
import math
import cv2
import sys
import rclpy
import select

from OpenEAIArmROS2Operator import OpenEAIArmROS2Operator

import requests
import msgpack
from PIL import Image as PILImage

# ================== CONSTANTS ==================
MASTER_ARM_MODE_PROGRAM = 0      # Model control mode
MASTER_ARM_MODE_TEACHING = 1     # Drag teaching mode

# ================== HELPER FUNCTIONS ==================

def convert_to_uint8(img: np.ndarray) -> np.ndarray:
    """Converts an image to uint8 if it is a float image."""
    if np.issubdtype(img.dtype, np.floating):
        img = (255 * img).astype(np.uint8)
    return img

def resize_with_pad(images: np.ndarray, height: int, width: int, method=PILImage.Resampling.BILINEAR) -> np.ndarray:
    if images.shape[-3:-1] == (height, width):
        return images

    original_shape = images.shape
    images = images.reshape(-1, *original_shape[-3:])
    resized = np.stack([np.array(_resize_with_pad_pil(PILImage.fromarray(im), height, width, method=method)) for im in images])
    return resized.reshape(*original_shape[:-3], *resized.shape[-3:])

def _resize_with_pad_pil(image: PILImage.Image, height: int, width: int, method: int) -> PILImage.Image:
    cur_width, cur_height = image.size
    if cur_width == width and cur_height == height:
        return image
    ratio = max(cur_width / width, cur_height / height)
    resized_height = int(cur_height / ratio)
    resized_width = int(cur_width / ratio)
    resized_image = image.resize((resized_width, resized_height), resample=method)
    zero_image = PILImage.new(resized_image.mode, (width, height), 0)
    pad_height = max(0, int((height - resized_height) / 2))
    pad_width = max(0, int((width - resized_width) / 2))
    zero_image.paste(resized_image, (pad_width, pad_height))
    return zero_image

def pack_array(obj):
    if (isinstance(obj, (np.ndarray, np.generic))) and obj.dtype.kind in ("V", "O", "c"):
        raise ValueError(f"Unsupported dtype: {obj.dtype}")
    if isinstance(obj, np.ndarray):
        return {b"__ndarray__": True, b"data": obj.tobytes(), b"dtype": obj.dtype.str, b"shape": obj.shape}
    if isinstance(obj, np.generic):
        return {b"__npgeneric__": True, b"data": obj.item(), b"dtype": obj.dtype.str}
    return obj

def unpack_array(obj):
    if b"__ndarray__" in obj:
        return np.ndarray(buffer=obj[b"data"], dtype=np.dtype(obj[b"dtype"]), shape=obj[b"shape"])
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj

packb = lambda data: msgpack.packb(data, default=pack_array, use_bin_type=True)
unpackb = lambda data: msgpack.unpackb(data, object_hook=unpack_array, raw=False)

# ================== MSGPACK HTTP CLIENT ==================

class MsgPackHttpClientPolicy:
    def __init__(self, host: str, port: int):
        protocol = "http"
        if host.startswith("http"):
            self.base_url = host
        else:
            self.base_url = f"{protocol}://{host}:{port}"

        self.infer_url = f"{self.base_url.rstrip('/')}/infer"
        self.session = requests.Session()
        self.session.headers.update({"Content-Type": "application/msgpack"})
        print(f"✅ Standalone MsgPack HTTP Client configured for: {self.infer_url}")

    def infer(self, observation: dict) -> dict:
        packed_observation = packb(observation)
        try:
            response = self.session.post(self.infer_url, data=packed_observation, timeout=30)
            response.raise_for_status()
            return unpackb(response.content)
        except requests.exceptions.RequestException as e:
            print(f"❌ Inference request failed: {e}")
            return {"error": str(e)}

# ================== GLOBAL VARIABLES ==================

client = None
inference_thread = None
inference_lock = threading.Lock()
inference_actions = None
inference_timestep = None

# ================== CORE FUNCTIONS ==================

def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)

def initialize_client(args):
    """Initialize a standalone MsgPack HTTP client"""
    global client
    try:
        client = MsgPackHttpClientPolicy(host=args.server_host, port=args.server_port)
    except Exception as e:
        print(f"❌ Failed to initialize HTTP client: {e}")
        raise

def get_frame(ros_operator):
    """Get current frame data from ROS operator"""
    result = ros_operator.get_frame()
    if not result:
        return False

    imgs = result['imgs']
    arms = result['arms']
    robot_base = result.get('robot_base', None)
    try:
        puppet_arm = [arms[key] for key in result['arm_keys'] if key.startswith('puppet')]
        master_arm = [arms[key] for key in result['arm_keys'] if key.startswith('master')]
        puppet_arm_pose = [arms[key+'_pose'] for key in result['arm_keys'] if key.startswith('puppet') and key+'_pose' in arms]
    except Exception as e:
        ros_operator.node.get_logger().warning(f"Frame construct failed: {e}")
        return False
    timestamp = result['timestamp']
    return (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp)

def get_master_arm_state(ros_operator):
    """Get current state of master arm (for teaching mode)"""
    result = ros_operator.get_frame(required=['arms'])
    if not result:
        return None

    arms = result['arms']
    try:
        # Find master arm data
        master_arm = [arms[key] for key in result['arm_keys'] if 'master' in key]
        if master_arm and len(master_arm) > 0:
            return master_arm[0].position  # Return position array
    except Exception as e:
        ros_operator.node.get_logger().warning(f"Failed to get master arm state: {e}")

    return None

def set_master_arm_mode(ros_operator, mode, arm_name="master"):
    """Set master arm control mode (0: program control, 1: drag teaching)"""
    from sensor_msgs.msg import JointState
    import rclpy

    msg = JointState()
    msg.header.stamp = ros_operator.node.get_clock().now().to_msg()
    msg.position = [float(mode), 0.0]  # mode, ee_pose=0

    # Create publisher if not exists
    if not hasattr(ros_operator, 'master_mode_pub'):
        ros_operator.master_mode_pub = ros_operator.node.create_publisher(
            JointState, f'{arm_name}/set_control_mode', 10
        )

    ros_operator.master_mode_pub.publish(msg)
    ros_operator.node.get_logger().info(f"Setting {arm_name} mode to: {mode} (0: program, 1: drag)")

def model_inference_process(args, ros_operator, t, pre_action):
    """Process for model inference (program control mode)"""
    global inference_lock, inference_actions, inference_timestep

    model_type = args.model
    rate = ros_operator.node.create_rate(args.frame_rate)

    while True and rclpy.ok():
        result = get_frame(ros_operator)
        if not result:
            ros_operator.node.get_logger().warning("Failed to get frame")
            rate.sleep()
            continue

        (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp) = result

        # Process images
        images = {}
        img_key_mapping = {
            "cam_left": "cam_left_wrist",
            "cam_right": "cam_right_wrist",
            "cam_agent": "cam_high",
            "cam_hand": "cam_right_wrist",
        }

        for img_key, img in imgs.items():
            img_key = 'cam_' + img_key
            img_key = img_key_mapping.get(img_key, img_key)
            img = img['rgb']

            if model_type == 'pi':
                img = img[:, :, [2, 1, 0]]
                img = resize_with_pad(img, 224, 224)
            elif model_type == 'octo' or model_type == 'openeai':
                img = cv2.resize(img, (224, 224))

            if model_type == 'pi' or model_type == 'act':
                img = rearrange(img, 'h w c -> c h w')

            img = convert_to_uint8(img)
            images[img_key] = img

        # Ensure we have both wrist cameras
        if 'cam_left_wrist' not in images and 'cam_right_wrist' in images:
            images['cam_left_wrist'] = images['cam_right_wrist']
        elif 'cam_right_wrist' not in images and 'cam_left_wrist' in images:
            images['cam_right_wrist'] = images['cam_left_wrist']

        # Get puppet arm state
        qpos = np.concatenate([np.array(arm.position) for arm in puppet_arm])

        if model_type == 'pi' and len(qpos) < 14:
            qpos = np.concatenate([qpos, np.zeros(14 - len(qpos))])

        ros_operator.node.get_logger().info(f"Sending qpos: {qpos}")

        # Prepare observation for model
        observation = {
            "state": qpos,
            "images": images,
            "prompt": args.task_instruction,
        }

        # Call model inference
        start_time = time.time()
        server_result = client.infer(observation)
        end_time = time.time()

        if "error" in server_result:
            ros_operator.node.get_logger().error(f"Server returned an error: {server_result['error']}")
            choice = input("Press 'r' to retry, any other key to exit: ")
            if choice.lower() == 'r':
                continue
            else:
                exit(0)

        ros_operator.node.get_logger().info(f"Model inference time: {end_time - start_time:.3f}s")

        # Process actions
        inference_lock.acquire()
        inference_actions = server_result["actions"]

        if len(inference_actions.shape) > 2:
            inference_actions = inference_actions[0]

        inference_actions = inference_actions.astype(np.float64)

        if pre_action is None:
            pre_action = qpos

        inference_timestep = t
        inference_lock.release()

        ros_operator.node.get_logger().info(f"Updated new actions: {inference_actions[0]}")

        # Check for joint jumps
        jump_flag = False
        JUMP_THRESHOLD = args.jump_threshold

        for i in range(qpos.shape[0]):
            if (i + 1) % 7 == 0:
                continue
            if abs(inference_actions[0, i] - qpos[i]) > JUMP_THRESHOLD:
                jump_flag = True
                ros_operator.node.get_logger().warning(f"Joint {i} jump detected: from {qpos[i]:.3f} to {inference_actions[0, i]:.3f}")
                break

        # Handle smooth transition if needed
        need_smooth = False
        if args.smooth:
            need_smooth = True
        elif jump_flag:
            ros_operator.node.get_logger().warning(f"Joint jump detected: from {qpos} to {inference_actions[0]}")
            choice = input("Press 'y' to execute despite jump, any other key to move slowly: ")
            if choice.lower() != 'y':
                need_smooth = True

        if need_smooth:
            # For dual-arm, we need to send 14-dim action
            dual_action = np.concatenate([inference_actions[0, :7], inference_actions[0, :7]])
            ros_operator.puppet_arm_publish_continuous('puppet', dual_action)

        break

def teaching_mode_process(args, ros_operator, t):
    """Process for teaching mode (using master arm as input)"""
    global inference_lock, inference_actions, inference_timestep

    rate = ros_operator.node.create_rate(args.frame_rate)

    while True and rclpy.ok():
        # Get current master arm state
        master_state = get_master_arm_state(ros_operator)

        if master_state is None:
            ros_operator.node.get_logger().warning("Failed to get master arm state")
            rate.sleep()
            continue

        # Convert to numpy array and ensure it's 7-dimensional
        master_state = np.array(master_state)
        if len(master_state) > 7:
            master_state = master_state[:7]
        elif len(master_state) < 7:
            master_state = np.concatenate([master_state, np.zeros(7 - len(master_state))])

        ros_operator.node.get_logger().info(f"Master arm state: {master_state}")

        # Create 14-dim action for dual arms (both arms follow master)
        dual_action = np.concatenate([master_state, master_state])

        inference_lock.acquire()
        inference_actions = dual_action.reshape(1, -1)
        inference_timestep = t
        inference_lock.release()

        ros_operator.node.get_logger().info(f"Teaching mode action: {dual_action}")
        break

def dual_arm_inference(args, ros_operator):
    """Main dual-arm inference loop with mode switching capability"""
    global inference_lock, inference_actions, inference_timestep, inference_thread

    set_seed(args.seed)

    # Initialize master arm in program control mode
    ros_operator.node.get_logger().info("Initializing master arm in program control mode")
    set_master_arm_mode(ros_operator, MASTER_ARM_MODE_PROGRAM)

    # Define preset poses
    RESET_POSE = np.array([0.0, 0.0, 0.033, -0.07, 0.025, 0.075, 0.0] * 2)
    PREPARE_POSE = np.array([0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0] * 2)

    # Move to initial position
    if args.resume:
        ros_operator.puppet_arm_publish_continuous('puppet', RESET_POSE)
        return

    if args.stand:
        ros_operator.node.get_logger().info("Moving to prepare pose")
        ros_operator.puppet_arm_publish_continuous('puppet', PREPARE_POSE)
    else:
        ros_operator.node.get_logger().info("Moving to reset pose")
        ros_operator.puppet_arm_publish_continuous('puppet', RESET_POSE)

    input("Press Enter to continue...")

    # Main control loop
    with torch.inference_mode():
        current_master_mode = MASTER_ARM_MODE_PROGRAM
        t = 0
        max_t = 0
        inference_t = 0
        all_actions = None

        rate = ros_operator.node.create_rate(args.frame_rate)

        if args.pos_lookahead_step <= 0:
            args.pos_lookahead_step = args.chunk_size

        while t < args.max_publish_step and rclpy.ok():
            # Check for mode switching input
            if args.enable_mode_switch and sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 't' and current_master_mode != MASTER_ARM_MODE_TEACHING:
                    # Switch to teaching mode
                    ros_operator.node.get_logger().info("Switching to TEACHING mode")
                    set_master_arm_mode(ros_operator, MASTER_ARM_MODE_TEACHING)
                    current_master_mode = MASTER_ARM_MODE_TEACHING
                    max_t = t  # Reset lookahead for immediate mode switch
                elif cmd == 'm' and current_master_mode != MASTER_ARM_MODE_PROGRAM:
                    # Switch to model control mode
                    ros_operator.node.get_logger().info("Switching to MODEL control mode")
                    set_master_arm_mode(ros_operator, MASTER_ARM_MODE_PROGRAM)
                    current_master_mode = MASTER_ARM_MODE_PROGRAM
                    max_t = t  # Reset lookahead for immediate mode switch
                elif cmd == 'q':
                    ros_operator.node.get_logger().info("Exiting...")
                    break

            # Get new actions if needed
            if t >= max_t:
                pre_action = None if all_actions is None else all_actions[t - inference_t - 1, :]

                if current_master_mode == MASTER_ARM_MODE_PROGRAM:
                    # Model inference mode
                    inference_thread = threading.Thread(
                        target=model_inference_process,
                        args=(args, ros_operator, t, pre_action)
                    )
                else:
                    # Teaching mode
                    inference_thread = threading.Thread(
                        target=teaching_mode_process,
                        args=(args, ros_operator, t)
                    )

                inference_thread.start()
                inference_thread.join()

                inference_lock.acquire()
                if inference_actions is not None:
                    inference_thread = None

                    if current_master_mode == MASTER_ARM_MODE_PROGRAM:
                        # Model returns sequence of actions
                        all_actions = inference_actions
                        max_t = t + args.pos_lookahead_step
                    else:
                        # Teaching mode returns single action
                        all_actions = inference_actions
                        max_t = t + 1  # Only look ahead 1 step in teaching mode

                    inference_actions = None
                    inference_t = t
                inference_lock.release()

            # Execute current action
            if all_actions is not None:
                if current_master_mode == MASTER_ARM_MODE_PROGRAM:
                    # Model control: get action from sequence
                    raw_action = all_actions[t - inference_t, :]

                    # Take first 7 dimensions for dual-arm execution
                    if len(raw_action) == 7:
                        action = np.concatenate([raw_action, raw_action])
                    elif len(raw_action) == 14:
                        action = raw_action
                    else:
                        ros_operator.node.get_logger().error(f"Unexpected action dimension: {len(raw_action)}")
                        action = np.zeros(14)
                else:
                    # Teaching mode: use current master state
                    action = all_actions[0, :]  # Already 14-dim from teaching process

                ros_operator.node.get_logger().info(f"Step {t}: Executing action {action[:7]}...")
                ros_operator.puppet_arm_publish('puppet', action)
                t += 1

            rate.sleep()

# ================== ARGUMENT PARSING ==================

def get_arguments():
    parser = argparse.ArgumentParser(description='Dual-arm inference with mode switching')

    # Execution parameters
    parser.add_argument('--max_publish_step', type=int, default=10000, help='Maximum steps to publish')
    parser.add_argument('--seed', type=int, default=0, help='Random seed')
    parser.add_argument('--frame_rate', type=int, default=20, help='Control frame rate')
    parser.add_argument('--image_shape', type=str, default='640x480x30', help='Image shape')
    parser.add_argument('--config_path', type=str, default="config_single.yml", help='ROS operator config path')

    # Model server parameters
    parser.add_argument('--task_instruction', type=str, default='pick up the object', help='Task instruction for model')
    parser.add_argument('--pos_lookahead_step', type=int, default=25, help='Position lookahead steps')
    parser.add_argument('--chunk_size', type=int, default=50, help='Chunk size for model inference')
    parser.add_argument('--server_host', type=str, required=True, help='Model server host')
    parser.add_argument('--server_port', type=int, default=8000, help='Model server port')
    parser.add_argument('--contiguous', action='store_true', default=False, help='Run without confirmation')
    parser.add_argument('--model', type=str, default='openeai', choices=['openeai', 'pi', 'act', 'octo'], help='Model type')

    # Mode and safety parameters
    parser.add_argument('-r', '--resume', action='store_true', default=False, help='Resume from reset pose')
    parser.add_argument('--smooth', action='store_true', default=False, help='Always use smooth transitions')
    parser.add_argument('--stand', action='store_true', default=False, help='Start in prepare pose')
    parser.add_argument('--enable_mode_switch', action='store_true', default=True, help='Enable real-time mode switching')
    parser.add_argument('--jump_threshold', type=float, default=0.2, help='Threshold for joint jump detection')
    parser.add_argument('--initial_mode', type=int, default=0, choices=[0, 1], help='Initial mode (0: model, 1: teaching)')

    return parser.parse_args()

# ================== MAIN FUNCTION ==================

def main():
    args = get_arguments()

    # Initialize ROS operator
    ros_operator = OpenEAIArmROS2Operator(args.config_path)

    # Initialize model client
    initialize_client(args)

    # Print control instructions
    print("\n" + "="*60)
    print("DUAL-ARM INFERENCE WITH MODE SWITCHING")
    print("="*60)
    print("Controls:")
    print("  Press 't' to switch to TEACHING mode (drag master arm)")
    print("  Press 'm' to switch to MODEL control mode")
    print("  Press 'q' to quit")
    print("="*60 + "\n")

    # Set initial mode
    if args.initial_mode == 1:
        set_master_arm_mode(ros_operator, MASTER_ARM_MODE_TEACHING)
        print("Starting in TEACHING mode")
    else:
        print("Starting in MODEL control mode")

    # Start main inference loop
    dual_arm_inference(args, ros_operator)

if __name__ == '__main__':
    # Note: For real-time keyboard input, run with:
    # python dual_arm_inference.py --server_host 127.0.0.1 --contiguous --model openeai

    main()
