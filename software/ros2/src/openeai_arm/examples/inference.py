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

from OpenEAIArmROS2Operator import OpenEAIArmROS2Operator

import requests
import msgpack
from PIL import Image as PILImage


def convert_to_uint8(img: np.ndarray) -> np.ndarray:
    """Converts an image to uint8 if it is a float image.

    This is important for reducing the size of the image when sending it over the network.
    """
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
        


client = None
inference_thread = None
inference_lock = threading.Lock()
inference_actions = None
inference_timestep = None


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
    result = ros_operator.get_frame()
    if not result:
        return False

    imgs = result['imgs']
    arms = result['arms']
    robot_base = result.get('robot_base', None)
    try:
        puppet_arm = [arms[key] for key in result['arm_keys'] if 'puppet' in key]
        master_arm = [arms[key] for key in result['arm_keys'] if 'master' in key]
        puppet_arm_pose = [arms[key+'_pose'] for key in result['arm_keys'] if 'puppet' in key and key+'_pose' in arms]
    except Exception as e:
        ros_operator.node.get_logger().warning("Frame construct failed: {}".format(e))
        return False
    timestamp = result['timestamp']
    return (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp)

def inference_process(args, ros_operator, t, pre_action):
    global inference_lock
    global inference_actions
    global inference_timestep
    print_flag = True
    model_type = args.model

    rate = ros_operator.node.create_rate(args.frame_rate)
    while True and rclpy.ok():
        if model_type == 'octo':
            time.sleep(0.25)
        result = get_frame(ros_operator)
        if not result:
            if print_flag:
                print("syn fail")
                print_flag = False
            rate.sleep()
            continue
        print_flag = True
        (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp) = result

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
            # resize only
            elif model_type == 'octo' or model_type == 'openeai':
                img = cv2.resize(img, (224, 224))
            if model_type == 'pi' or model_type == 'act':
                img = rearrange(img, 'h w c -> c h w')
            img = convert_to_uint8(img)
            images[img_key] = img
        if 'cam_left_wrist' not in images and 'cam_right_wrist' in images:
            images['cam_left_wrist'] = images['cam_right_wrist']
        elif 'cam_right_wrist' not in images and 'cam_left_wrist' in images:
            images['cam_right_wrist'] = images['cam_left_wrist']
        
        qpos = np.concatenate([np.array(arm.position) for arm in puppet_arm])
        if model_type == 'pi':
            # padding to 14 dims
            if len(qpos) < 14:
                qpos = np.concatenate([qpos, np.zeros(14 - len(qpos))])
        
        ros_operator.node.get_logger().info("Sending qpos: {}".format(qpos))
        
        observation = {
            "state": qpos,
            "images": images,
            "prompt": args.task_instruction,
        }

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
        print("model cost time: ", end_time -start_time)
        inference_lock.acquire()
        inference_actions = server_result["actions"]
        if len(inference_actions.shape) > 2:
            inference_actions = inference_actions[0]
        inference_actions = inference_actions.astype(np.float64)

        if pre_action is None:
            pre_action = qpos
        
        inference_timestep = t
        inference_lock.release()
        ros_operator.node.get_logger().info("Updated new actions: {}".format(inference_actions))
        if not args.contiguous:
            choice = input("Press Enter to continue...")
            if choice.lower() == 'n':
                exit(0)
                
        jump_flag = False
        jump_threshold = 0.2
        for i in range(qpos.shape[0]):
            if (i + 1) % 7 == 0:
                continue
            if abs(inference_actions[0, i] - qpos[i]) > jump_threshold: 
                jump_flag = True
                print(f"Joint {i} jump detected: from {qpos[i]} to {inference_actions[0, i]}")
                break
        need_smooth = False
        if args.smooth:
            need_smooth = True
        elif jump_flag:
            ros_operator.node.get_logger().warning("Joint jump detected: from {} to {}".format(qpos, inference_actions[0]))
            choice = input("Press 'y' to execute the action despite the jump, any other key to move slowly to the first predicted action: ")
            if choice.lower() != 'y':
                need_smooth = True
        if need_smooth:
            ros_operator.puppet_arm_publish_continuous('puppet', inference_actions[0])
                
        break


def model_inference(args, ros_operator):
    global inference_lock
    global inference_actions
    global inference_timestep
    global inference_thread
    set_seed(1000)

    max_publish_step = args.max_publish_step
    chunk_size = args.chunk_size

    reset_pose = [0.0, 0.0, 0.033, -0.07, 0.025, 0.075, 0.0, 0.0, 0.0, 0.033, -0.07, 0.025, 0.075, 0.0]
    prepare_pose = [0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0]

    
    if args.resume:
        ros_operator.puppet_arm_publish_continuous('puppet', reset_pose)
        return
    
    if args.stand:
        ros_operator.puppet_arm_publish_continuous('puppet', prepare_pose)
    else:
        ros_operator.puppet_arm_publish_continuous('puppet', reset_pose)
    input("Enter any key to continue :")
    action = None
    with torch.inference_mode():
        while True and rclpy.ok():
            t = 0
            max_t = 0
            rate = ros_operator.node.create_rate(args.frame_rate)

            if args.pos_lookahead_step <= 0:
                args.pos_lookahead_step = chunk_size

            while t < max_publish_step and rclpy.ok():
                if t >= max_t:
                    pre_action = action
                    inference_thread = threading.Thread(target=inference_process,
                                                        args=(args, ros_operator,
                                                                t, pre_action))
                    inference_thread.start()
                    inference_thread.join()
                    inference_lock.acquire()
                    if inference_actions is not None:
                        inference_thread = None
                        all_actions = inference_actions
                        inference_actions = None
                        max_t = t + args.pos_lookahead_step
                    inference_lock.release()
                

                raw_action = all_actions[t % args.pos_lookahead_step, :]
                action = raw_action
                print(action)
                ros_operator.puppet_arm_publish('puppet', action)
                t += 1

                rate.sleep()


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--max_publish_step', action='store', type=int, help='max_publish_step', default=10000, required=False)
    parser.add_argument('--seed', action='store', type=int, help='seed', default=0, required=False)
    
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=20, required=False)
    
    parser.add_argument('--image_shape', action='store', type=str, help='frame_rate',
                        default='640x480x30', required=False)
    parser.add_argument('--config_path', type=str, default="config_single.yml", help="OpenEAIArmROS2Operator config path")

    
    # for server
    parser.add_argument('--task_instruction', action='store', type=str, 
                    help='task_instruction', default='pick up the object', required=False)
    parser.add_argument('--pos_lookahead_step', action='store', type=int, help='pos_lookahead_step',
                        default=25, required=False)
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size',
                        default=50, required=False)
    parser.add_argument('--server_host', action='store', type=str, 
                       help='Model server host', required=True)
    parser.add_argument('--server_port', action='store', type=int, 
                       help='Model server port', default=8000, required=False)
    parser.add_argument('--contiguous', action='store_true',
                       help='Run the policy without confirmation. Make sure your policy works before enabling this option.', default=False, required=False)
    parser.add_argument('--model', type=str, default='openeai', help='Model type on the server. Available options: [openeai, pi, act, octo]', required=False, choices=['openeai', 'pi', 'act', 'octo'])
    parser.add_argument('-r', '--resume', default=False, required=False, action='store_true')
    parser.add_argument('--smooth', default=False, required=False, action='store_true')
    parser.add_argument('--stand', default=False, required=False, action='store_true')


    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    ros_operator = OpenEAIArmROS2Operator(args.config_path)
    initialize_client(args)
    model_inference(args, ros_operator)


if __name__ == '__main__':
    main()

# python ./inference.py --task_instruction "finish the task" --server_host 127.0.0.1 --contiguous --model openeai

