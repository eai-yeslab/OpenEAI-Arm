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
import h5py
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.collections import LineCollection
from pathlib import Path
from sensor_msgs.msg import JointState
matplotlib.use("TkAgg")

from OpenEAIArmROS2Operator import OpenEAIArmROS2Operator
from replay_latency_metrics import (
    LatencyTrace,
    SynchronizedFeedbackRecorder,
    analyze_latency_trace,
    print_latency_report,
    save_latency_report,
)

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

def load_replay_actions(h5_path: str, field: str) -> np.ndarray:
    """Load one or more complete 7-DoF arm actions from an HDF5 dataset."""
    with h5py.File(h5_path, "r") as f:
        if field not in f:
            raise KeyError(f"Replay field '{field}' not found in hdf5. Available keys: {list(f.keys())}")
        actions = np.asarray(f[field])
    if actions.ndim != 2 or actions.shape[1] < 7 or actions.shape[1] % 7 != 0:
        raise ValueError(
            "Replay actions must contain one or more complete 7-DoF arms, "
            f"got shape {actions.shape}"
        )
    return actions.astype(np.float64)


def smooth_replay_actions(
    actions: np.ndarray,
    window_sizes: list[int],
    strengths: list[float],
) -> np.ndarray:
    """Smooth replay joint positions while preserving the first and last frames."""
    joint_count = actions.shape[1]
    if len(window_sizes) != joint_count or len(strengths) != joint_count:
        raise ValueError(
            "Replay smoothing parameters must contain one value per joint: "
            f"expected {joint_count}, got {len(window_sizes)} windows and "
            f"{len(strengths)} strengths"
        )

    smoothed = actions.copy()
    progress = np.linspace(0.0, 1.0, actions.shape[0])

    for joint_idx, (window_size, strength) in enumerate(zip(window_sizes, strengths)):
        if window_size <= 1 or strength <= 0.0:
            continue
        if window_size % 2 == 0:
            raise ValueError(f"Replay smoothing window must be odd, got {window_size}")
        if not 0.0 <= strength <= 1.0:
            raise ValueError(f"Replay smoothing strength must be in [0, 1], got {strength}")

        window_size = min(window_size, actions.shape[0])
        if window_size % 2 == 0:
            window_size -= 1
        if window_size <= 1:
            continue

        kernel = np.hanning(window_size)
        kernel /= np.sum(kernel)
        padding = window_size // 2
        padded = np.pad(actions[:, joint_idx], (padding, padding), mode="edge")
        filtered = np.convolve(padded, kernel, mode="valid")

        # Preserve both endpoints without introducing a step at either boundary.
        filtered += (
            (actions[0, joint_idx] - filtered[0]) * (1.0 - progress)
            + (actions[-1, joint_idx] - filtered[-1]) * progress
        )
        smoothed[:, joint_idx] = (
            (1.0 - strength) * actions[:, joint_idx] + strength * filtered
        )

    smoothed[0] = actions[0]
    smoothed[-1] = actions[-1]
    return smoothed


def smooth_inference_chunk(
    actions: np.ndarray,
    window_size: int,
    strength: float,
) -> np.ndarray:
    """Smooth all arm joints in a model action chunk without changing grippers."""
    actions = np.asarray(actions, dtype=np.float64)
    if actions.ndim != 2:
        raise ValueError(f"Inference actions must be 2D, got shape {actions.shape}")
    if len(actions) <= 1 or window_size <= 1 or strength <= 0.0:
        return actions.copy()
    if window_size % 2 == 0:
        raise ValueError(f"Inference smoothing window must be odd, got {window_size}")
    if not 0.0 <= strength <= 1.0:
        raise ValueError(f"Inference smoothing strength must be in [0, 1], got {strength}")

    effective_window = min(window_size, len(actions))
    if effective_window % 2 == 0:
        effective_window -= 1
    if effective_window <= 1:
        return actions.copy()

    kernel = np.hanning(effective_window)
    kernel /= np.sum(kernel)
    padding = effective_window // 2
    progress = np.linspace(0.0, 1.0, len(actions))
    smoothed = actions.copy()
    gripper_dims = set(range(6, actions.shape[1], 7))

    for dimension in range(actions.shape[1]):
        if dimension in gripper_dims:
            continue

        padded = np.pad(actions[:, dimension], (padding, padding), mode="edge")
        filtered = np.convolve(padded, kernel, mode="valid")

        # Preserve the model's chunk endpoints while distributing the small
        # correction continuously across the full chunk.
        filtered += (
            (actions[0, dimension] - filtered[0]) * (1.0 - progress)
            + (actions[-1, dimension] - filtered[-1]) * progress
        )
        smoothed[:, dimension] = (
            (1.0 - strength) * actions[:, dimension] + strength * filtered
        )

    smoothed[0] = actions[0]
    smoothed[-1] = actions[-1]
    return smoothed


client = None
inference_thread = None
inference_lock = threading.Lock()
inference_actions = None
inference_timestep = -1
inference_meta = None
execute_history = []
state_history = []
inference_count = 0


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
    extra = result.get('extra', None)
    current_time = time.time()
    # print("Get frame latency: {:.3f}s".format(current_time - timestamp))
    return (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp, extra)

def _build_rtc_bridge_actions(
    prev_action: np.ndarray,
    next_action: np.ndarray,
    future_actions: np.ndarray | None = None,
    max_step: float = 0.01,
    speed_mode: str = "mean",   # "mean" or "max"
):
    """
    在 prev_action 和 next_action 之间插入若干补偿帧。
    速度估计按 chunk 内平均速度或最快速度处理，并忽略夹爪维度（每第7维，即索引 6, 13, 20, ...）。
    """
    prev_action = np.asarray(prev_action, dtype=np.float64).copy()
    next_action = np.asarray(next_action, dtype=np.float64).copy()
    D = prev_action.shape[0]

    step_size = np.full(D, max_step, dtype=np.float64)

    # 忽略夹爪维度：每第7维（0-based 下为 6, 13, 20, ...）
    gripper_dims = set(range(6, D, 7))
    arm_dims = [j for j in range(D) if j not in gripper_dims]

    # 根据 future_actions 在 chunk 内估计每个关节速度
    if future_actions is not None:
        future_actions = np.asarray(future_actions, dtype=np.float64)
        if future_actions.ndim == 2 and future_actions.shape[0] >= 2:
            diffs = np.abs(np.diff(future_actions, axis=0))  # [T-1, D]

            if speed_mode == "max":
                est = diffs.max(axis=0)
            elif speed_mode == "mean":
                est = diffs.mean(axis=0)
            else:
                try:
                    speed_mode = float(speed_mode)
                    est = np.percentile(diffs, speed_mode, axis=0)
                except ValueError:
                    raise ValueError(f"Invalid speed_mode: {speed_mode}")

            if len(arm_dims) > 0:
                step_size[arm_dims] = np.clip(est[arm_dims], 0.01, 0.1) * 1.5

    current = prev_action.copy()
    bridge = []

    while True:
        done = True
        new_current = current.copy()

        for j in range(D):
            diff = next_action[j] - current[j]
            abs_diff = abs(diff)
            step = step_size[j]

            if abs_diff <= step:
                new_current[j] = next_action[j]
            else:
                new_current[j] += np.sign(diff) * step
                done = False

        bridge.append(new_current.copy())
        current = new_current

        if done:
            break

    return np.asarray(bridge, dtype=np.float64)[:-1]  # exclude the last frame which is exactly next_action


def _merge_rtc_overlap(
    current_chunk: np.ndarray,
    current_idx: int,
    next_chunk: np.ndarray,
    next_idx: int,
    max_overlap: int | None = None,
):
    """Cross-fade overlapping chunk frames without adding execution steps."""
    merged_chunk = np.asarray(next_chunk, dtype=np.float64).copy()
    current_chunk = np.asarray(current_chunk, dtype=np.float64)
    if current_chunk.ndim != 2 or merged_chunk.ndim != 2:
        raise ValueError("RTC chunks must be two-dimensional")
    if current_chunk.shape[1] != merged_chunk.shape[1]:
        raise ValueError(
            "RTC chunks must have matching action dimensions: "
            f"{current_chunk.shape[1]} != {merged_chunk.shape[1]}"
        )

    current_idx = max(0, int(current_idx))
    next_idx = max(0, int(next_idx))
    overlap = min(
        max(0, len(current_chunk) - current_idx),
        max(0, len(merged_chunk) - next_idx),
    )
    if max_overlap is not None:
        overlap = min(overlap, max(0, int(max_overlap)))
    # print(f"Merging {overlap} frames")
    if overlap <= 0:
        return merged_chunk, 0

    # A raised-cosine cross-fade starts close to the remaining old trajectory
    # and reaches the time-aligned new trajectory at the end of the overlap.
    # Do not use either cross-fade endpoint inside the overlap. This keeps a
    # one-frame overlap as an equal blend and avoids treating any overlap frame
    # as entirely old or entirely new.
    phase = (
        np.arange(1, overlap + 1, dtype=np.float64)
        / (overlap + 1)
    )
    new_weight = 0.5 - 0.5 * np.cos(np.pi * phase)
    gripper_dims = [] #set(range(6, merged_chunk.shape[1], 7))
    arm_dims = [
        dimension
        for dimension in range(merged_chunk.shape[1])
        if dimension not in gripper_dims
    ]
    merged_chunk[next_idx:next_idx + overlap, arm_dims] = (
        (1.0 - new_weight[:, None])
        * current_chunk[current_idx:current_idx + overlap, arm_dims]
        + new_weight[:, None]
        * merged_chunk[next_idx:next_idx + overlap, arm_dims]
    )
    return merged_chunk, overlap


def _warn_model_wait(node, start_time: float, frame_rate: float, context: str):
    """Report wall-clock time for which action progression waited on inference."""
    wait_seconds = max(0.0, time.perf_counter() - start_time)
    if wait_seconds >= 0.001:
        wait_frames = wait_seconds * frame_rate
        # node.get_logger().warning(
        #     f"Model wait ({context}): {wait_seconds:.3f}s "
        #     f"(~{wait_frames:.1f} frames at {frame_rate:g} Hz)"
        # )
    return wait_seconds

def inference_process(
    args,
    ros_operator,
    global_t,
    current_chunk,
    trigger_chunk_idx,
    recorder=None,
):
    """
    异步推理线程：
    1) 获取当前观测
    2) 构造 RTC previous_action
    3) 调用服务器推理
    4) 将结果写入共享全局变量

    参数
    ----
    global_t : int
        发起本次推理时的全局时间步
    current_chunk : np.ndarray or None, shape [L, D]
        当前正在执行的 chunk；初次推理时可为 None
    trigger_chunk_idx : int
        发起推理时，在当前 chunk 内已经执行到的位置
        即：当前 chunk 的“时间对齐点”
    """
    global inference_lock
    global inference_actions
    global inference_timestep
    global inference_meta
    global inference_count

    print_flag = True
    model_type = args.model
    inference_count += 1

    while rclpy.ok():
        if model_type == 'octo':
            time.sleep(0.25)

        result = get_frame(ros_operator)
        if not result:
            if print_flag:
                print("syn fail")
                print_flag = False
            time.sleep(1.0 / args.frame_rate)
            continue

        print_flag = True
        (imgs, master_arm, puppet_arm, puppet_arm_pose, timestamp, extra) = result

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
            elif model_type in ('octo', 'openeai'):
                img = cv2.resize(img, (224, 224))

            if model_type in ('act', 'pi'):
                img = rearrange(img, 'h w c -> c h w')

            img = convert_to_uint8(img)
            images[img_key] = img

        if 'cam_left_wrist' not in images and 'cam_right_wrist' in images:
            images['cam_left_wrist'] = images['cam_right_wrist']
        elif 'cam_right_wrist' not in images and 'cam_left_wrist' in images:
            images['cam_right_wrist'] = images['cam_left_wrist']

        if args.use_tactile and 'cam_agent' in images:
            images['cam_left_wrist'] = images['cam_agent']
            images['cam_right_wrist'] = images['cam_agent']

        qpos = np.concatenate([np.array(arm.position) for arm in puppet_arm])

        tactile_idx = []
        if args.use_tactile:
            tactile_data = extra.get('tactile', None)
            if tactile_data is not None:
                tactile_data = tactile_data['msg'].data
                tactile_data = np.array(tactile_data).astype(np.float32)
                if len(tactile_data.shape) == 0:
                    tactile_data = tactile_data.reshape(1)
                tactile_idx = list(range(len(qpos), len(qpos) + len(tactile_data)))
                qpos = np.concatenate([qpos, tactile_data])

        if model_type == 'pi' and len(qpos) < 14:
            qpos = np.concatenate([qpos, np.zeros(14 - len(qpos))])

        # ros_operator.node.get_logger().info(f"Sending qpos: {qpos}")
        # print(f"Sending qpos at time {global_t}: {qpos}")

        observation = {
            "state": qpos,
            "images": images,
            "prompt": args.task_instruction,
            "embodiment_id": "mobile-aloha",
            "recap_condition": "positive"
            # "data_key": "siiarm_clean_table_ordered_fast"
        }
        state_history.append((global_t, observation["state"].tolist()))
        if recorder is not None:
            recorder.on_observation(global_t, timestamp, observation)

        previous_action = None

        # RTC payload:
        # 这里 remaining_actions 对齐到“推理启动时刻”
        # execution_horizon 传固定的 lookahead step，更符合 chunk 调度语义
        if args.pos_inference_step > 0:
            if current_chunk is not None:
                remaining_actions = current_chunk[trigger_chunk_idx:trigger_chunk_idx+args.pos_inference_step]  # [L_remain, D]
            else:
                remaining_actions = qpos[None, :].repeat(args.pos_inference_step, axis=0)  # [L_remain, D]

            previous_action = {
                "remaining_actions": remaining_actions.tolist(),
                "remaining_actions_size": [len(remaining_actions)],
                "max_delay_steps": int(max(0, args.pos_inference_step)),
                "execution_horizon": int(max(0, args.pos_lookahead_step)),
                "guidance_beta": float(getattr(args, "rtc_guidance_beta", 10)),
                "alpha_dq": 0.5 if model_type == 'openeai' else 0.0,
                "alpha_tau": 0.0,
                "enable_tau": False,
                "smooth_weight": 0.5 if model_type == 'openeai' else 0.0,
                "base_weight": 0.0 if model_type == 'openeai' else 0.0,
                "torque_eps": 1e-5,
                "gamma": 0.1,
                "use_previous_action_in_ae": False,
                # "prev_velocity": np.zeros_like(qpos).tolist(),
                # "prev_torque": np.zeros_like(qpos).tolist(),
            }
            observation["previous_action"] = previous_action
            # print("======== remaining actions ========")
            # for i in range(len(remaining_actions)):
            #     print(f"Step {i}: "
            #         f"{list(remaining_actions[i])}")

        start_time = time.time()
        server_result = client.infer(observation)
        # time.sleep(0.2)
        end_time = time.time()
        model_latency = end_time - start_time

        if "error" in server_result:
            ros_operator.node.get_logger().error(
                f"Server returned an error: {server_result['error']}"
            )
            choice = input("Press 'r' to retry, any other key to exit: ")
            if choice.lower() == 'r':
                continue
            else:
                exit(0)

        # print(f"model cost time: {model_latency:.3f}s")

        new_actions = np.asarray(server_result["actions"])
        if len(new_actions.shape) > 2:
            new_actions = new_actions[0]
        new_actions = new_actions.astype(np.float64)
        raw_new_actions = new_actions.copy()

        new_actions[..., 1] += 0.03
        # new_actions[..., -1] += 0.01
        if new_actions.shape[-1] > 8:
            new_actions[..., 8] += 0.07

        sync_actions = np.asarray(server_result.get("sync_actions", new_actions)).astype(np.float64)
        if len(sync_actions.shape) > 2:
            sync_actions = sync_actions[0]
        sync_actions = sync_actions.astype(np.float64)

        # Smooth the complete model chunk before selecting its executable
        # suffix. The chunk remains local and is still published one frame at
        # a time by model_inference(). Gripper dimensions remain unchanged.
        chunk_smoothing_window = 25
        chunk_smoothing_strength = 1.0
        if args.smooth:
            new_actions = smooth_inference_chunk(
                new_actions,
                window_size=chunk_smoothing_window,
                strength=chunk_smoothing_strength,
            )
            # print(
            #     "Inference chunk smoothing enabled: "
            #     f"window={chunk_smoothing_window}, "
            #     f"strength={chunk_smoothing_strength}"
            # )

        # 估计 delay steps:
        # 优先保守取 args.pos_inference_step，同时也结合真实 latency
        measured_delay_steps = int(np.ceil(model_latency * args.frame_rate)) * 0
        delay_steps = max(int(max(0, args.pos_inference_step)), measured_delay_steps)

        # RTC 返回的新 chunk 前 delay_steps 个动作为“历史/冻结前缀”，
        # 真正切换执行时应从该 offset 开始消费
        first_exec_idx = min(delay_steps, len(new_actions) - 1)
        for i in range(min(sync_actions.shape[0], first_exec_idx)):
            execute_history.append((global_t + i, f"sync/{inference_count}", sync_actions[i].tolist()))

        # print("======== Discarded actions ========")
        for i in range(first_exec_idx):
            # print(f"Step {i}: "
            #       f"{list(raw_new_actions[i])}")
            execute_history.append((global_t + i, f"discarded/{inference_count}", raw_new_actions[i].tolist()))

        if recorder is not None:
            recorder.on_prediction(
                global_t,
                raw_new_actions,
                {
                    "latency_sec": model_latency,
                    "delay_steps": delay_steps,
                    "first_exec_idx": first_exec_idx,
                    "trigger_chunk_idx": int(trigger_chunk_idx),
                    "timestamp": timestamp,
                },
            )

        jump_flag = False
        jump_threshold = 0.2

        # for i in range(qpos.shape[0]):
        #     if (i + 1) % 7 == 0:
        #         continue
        #     if i not in tactile_idx and abs(new_actions[first_exec_idx, i] - qpos[i]) > jump_threshold:
        #         jump_flag = True
        #         print(
        #             f"Joint {i} jump detected: "
        #             f"from {qpos[i]} to {new_actions[first_exec_idx, i]}"
        #         )
        #         break

        # 改为生成 bridge actions，而不是直接 publish_continuous
        bridge_actions = None

        if current_chunk is not None and len(current_chunk) > 0:
            current_idx = min(trigger_chunk_idx + args.pos_inference_step, len(current_chunk) - 1)
            prev_last_action = np.asarray(current_chunk[current_idx], dtype=np.float64)
            next_first_action = np.asarray(new_actions[first_exec_idx], dtype=np.float64)

            # if current_idx < len(current_chunk) - 1:
            #     print("======== Remaining current chunk actions not executed ========")
            #     for i in range(current_idx, len(current_chunk)):
            #         print(f"Step {i}: {list(current_chunk[i])}")

            future_for_speed = None
            if first_exec_idx + 2 <= len(new_actions):
                future_for_speed = new_actions[first_exec_idx:first_exec_idx + 2]

            need_bridge = False
            if args.merge:
                need_bridge = False
            elif args.smooth:
                need_bridge = True
            elif jump_flag:
                ros_operator.node.get_logger().warning(
                    f"Joint jump detected near RTC switch: "
                    f"from {prev_last_action} to {next_first_action}"
                )
                choice = input(
                    "Press 'y' to execute the action despite the jump, "
                    "any other key to insert compensation frames: "
                )
                if choice.lower() != 'y':
                    need_bridge = True

            if need_bridge:
                bridge_actions = _build_rtc_bridge_actions(
                    prev_action=prev_last_action,
                    next_action=next_first_action,
                    future_actions=future_for_speed,
                    max_step=0.005,
                    speed_mode="mean"
                )
                # bridge_actions = []
                # input("Press Enter to continue after reviewing the jump issue...")

        with inference_lock:
            inference_actions = new_actions
            inference_timestep = global_t
            inference_meta = {
                "latency_sec": model_latency,
                "delay_steps": delay_steps,
                "trigger_chunk_idx": int(trigger_chunk_idx),
                "previous_action": previous_action,
                "timestamp": timestamp,
                "bridge_actions": bridge_actions,
                "chunk_smoothing_window": chunk_smoothing_window if args.smooth else 1,
            }

        if not args.contiguous:
            choice = input("Press Enter to continue...")
            if choice.lower() == 'n':
                exit(0)

        break

def model_inference(args, ros_operator, recorder=None):
    global inference_lock
    global inference_actions
    global inference_timestep
    global inference_thread
    global inference_meta
    global execute_history

    set_seed(1000)

    max_publish_step = args.max_publish_step
    chunk_size = args.chunk_size
    trigger_step = max(0, args.pos_lookahead_step)

    reset_pose = [0.0, 0.2, 0.3, -0.2, 0.0, 0.0, 0.0,
                  0.0, 0.2, 0.3, -0.2, 0.0, 0.0, 0.0]
    prepare_pose = [0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0]

    if args.resume:
        ros_operator.puppet_arm_publish_continuous('puppet', reset_pose)
        return

    if args.stand:
        ros_operator.puppet_arm_publish_continuous('puppet', prepare_pose)
    else:
        ros_operator.puppet_arm_publish_continuous('puppet', reset_pose)

    input("Enter any key to continue :")
    if recorder is not None:
        recorder.on_ready()

    # 当前正在执行的 chunk 及其执行偏移
    current_chunk = None              # np.ndarray [L, D]
    current_chunk_base_idx = 0        # 当前 chunk 从哪个 index 开始被消费（RTC 切换后通常是 d）
    chunk_start_t = 0                 # 当前 chunk 生效时的全局 t

    # bridge chunk：旧 chunk 和新 chunk 之间的补偿帧
    bridge_chunk = None               # np.ndarray [M, D] or None
    bridge_start_t = 0                # bridge 生效时的全局 t

    # 后台推理状态
    pending_thread = None
    pending_trigger_t = None          # 发起该次推理时的全局 t
    pending_trigger_chunk_idx = None  # 发起该次推理时，当前 chunk 内的 index

    last_moving_time = time.time()
    rate = ros_operator.node.create_rate(args.frame_rate)

    t = 0
    start_time = time.time()
    try:
        while t < max_publish_step and rclpy.ok():
            # Episode boundary requested by the recorder: discard any in-flight
            # inference result and the current chunk so the next chunk is taken
            # from the new episode's observations (reject late results).
            if recorder is not None and recorder.consume_reset_request():
                if pending_thread is not None:
                    pending_thread.join()
                    pending_thread = None
                with inference_lock:
                    inference_actions = None
                    inference_meta = None
                current_chunk = None
                bridge_chunk = None
                pending_trigger_t = None
                pending_trigger_chunk_idx = None
                continue

            # Episode finished via 'n'/'r': leave the loop so the entry point can
            # re-run the start-up sequence (reset pose + confirm + new episode).
            if recorder is not None and recorder.consume_stop_request():
                break

            # =========================================================
            # 1) 如果还没有当前 chunk，先同步拿一个初始 chunk
            # =========================================================
            if current_chunk is None:
                with inference_lock:
                    inference_actions = None
                    inference_timestep = -1
                    inference_meta = None

                pending_thread = threading.Thread(
                    target=inference_process,
                    args=(args, ros_operator, t, None, 0, recorder),
                    daemon=True,
                )
                pending_thread.start()
                initial_wait_start = time.perf_counter()
                pending_thread.join()
                _warn_model_wait(
                    ros_operator.node,
                    initial_wait_start,
                    args.frame_rate,
                    "initial chunk",
                )
                pending_thread = None

                with inference_lock:
                    if inference_actions is None:
                        ros_operator.node.get_logger().error("Initial inference failed.")
                        break

                    current_chunk = inference_actions.copy()
                    inference_actions = None

                    # 初始 chunk 没有 previous_action，因此从 0 开始消费
                    current_chunk_base_idx = 0
                    chunk_start_t = t

                    pending_trigger_t = None
                    pending_trigger_chunk_idx = None

            # 当前 chunk 内“已经过去了多少个执行步”
            executed_since_switch = t - chunk_start_t
            current_idx = current_chunk_base_idx + executed_since_switch
            current_len = len(current_chunk)

            # =========================================================
            # 1.5) 如果当前存在 bridge chunk，优先执行 bridge
            # =========================================================
            if bridge_chunk is not None:
                bridge_idx = t - bridge_start_t
                if bridge_idx < len(bridge_chunk):
                    action = bridge_chunk[bridge_idx]
                    current_time = time.time()
                    # print(f"[+{(current_time - last_moving_time)*1000:.1f}ms] [BRIDGE] {list(action)}")
                    last_moving_time = current_time

                    ros_operator.puppet_arm_publish('puppet', action)
                    execute_history.append((t, "bridge", action.tolist()))

                    t += 1
                    rate.sleep()
                    continue
                else:
                    bridge_chunk = None

            # =========================================================
            # 2) 如果当前 chunk 已耗尽，则必须先等后台结果
            # =========================================================
            if current_idx >= current_len:
                ros_operator.node.get_logger().warning(
                    "Current chunk exhausted. Waiting for next chunk..."
                )
                exhausted_wait_start = time.perf_counter()

                while rclpy.ok():
                    with inference_lock:
                        has_new_chunk = inference_actions is not None

                    if has_new_chunk:
                        break

                    if pending_thread is not None and not pending_thread.is_alive():
                        pending_thread.join()
                        pending_thread = None

                        with inference_lock:
                            has_new_chunk = inference_actions is not None

                        if has_new_chunk:
                            break

                    # ---- 新增：兜底发起推理 ----
                    if pending_thread is None:
                        ros_operator.node.get_logger().warning(
                            "No pending inference thread. Launching fallback inference."
                        )
                        pending_thread = threading.Thread(
                            target=inference_process,
                            args=(args, ros_operator, t, current_chunk.copy(), min(current_idx, len(current_chunk) - 1), recorder),
                            daemon=True,
                        )
                        pending_thread.start()
                    # ---- 新增结束 ----

                    wait_action = current_chunk[-1]
                    ros_operator.puppet_arm_publish('puppet', wait_action)
                    execute_history.append((t, "wait", wait_action.tolist()))
                    rate.sleep()

                _warn_model_wait(
                    ros_operator.node,
                    exhausted_wait_start,
                    args.frame_rate,
                    "chunk exhausted",
                )

                # ... 后续不变

                with inference_lock:
                    if inference_actions is None:
                        ros_operator.node.get_logger().error(
                            "ROS stopped before next chunk became available."
                        )
                        break

                    next_chunk = inference_actions.copy()
                    meta = inference_meta if inference_meta is not None else {}
                    inference_actions = None
                    inference_meta = None

                next_chunk_base_idx = int(meta.get("delay_steps", 0))
                next_chunk_base_idx = min(next_chunk_base_idx, len(next_chunk) - 1)

                if args.merge:
                    next_chunk, merged_frames = _merge_rtc_overlap(
                        current_chunk,
                        current_idx,
                        next_chunk,
                        next_chunk_base_idx,
                    )
                    if merged_frames > 0:
                        # ros_operator.node.get_logger().info(
                        #     f"Merged {merged_frames} overlapping RTC frames "
                        #     f"(old {current_idx}/{len(current_chunk)}, "
                        #     f"new {next_chunk_base_idx}/{len(next_chunk)})."
                        # )
                        pass
                    else:
                        ros_operator.node.get_logger().warning(
                            "Merge requested, but no overlapping RTC frames remain; "
                            "switching directly."
                        )

                # 若有 bridge，先执行 bridge，再开始新 chunk
                bridge_actions = (
                    None if args.merge else meta.get("bridge_actions", None)
                )
                if bridge_actions is not None and len(bridge_actions) > 0:
                    bridge_chunk = np.asarray(bridge_actions, dtype=np.float64)
                    bridge_start_t = t

                current_chunk = next_chunk
                current_chunk_base_idx = next_chunk_base_idx

                # 注意：新 chunk 的时间起点应是“bridge 执行完之后”
                if bridge_chunk is not None:
                    chunk_start_t = t + len(bridge_chunk)
                else:
                    chunk_start_t = t

                pending_trigger_t = None
                pending_trigger_chunk_idx = None

                executed_since_switch = t - chunk_start_t
                current_idx = current_chunk_base_idx + executed_since_switch
                current_len = len(current_chunk)



            # =========================================================
            # 3) 当执行到 trigger_step 时，异步触发下一次推理
            #    注意这里的 trigger 是相对于“当前 chunk 的消费起点”而言的
            # =========================================================
            if pending_thread is None:
                # 正常触发：到达 trigger_step
                # 兜底触发：chunk 剩余帧数不足以等到 trigger_step
                # remaining_frames = current_len - current_idx
                # effective_trigger = min(trigger_step, remaining_frames - 1)

                # if executed_since_switch >= effective_trigger:
                if executed_since_switch >= trigger_step or current_idx >= current_len - 1 - args.pos_inference_step:
                    pending_trigger_t = t
                    pending_trigger_chunk_idx = current_idx

                    pending_thread = threading.Thread(
                        target=inference_process,
                        args=(args, ros_operator, t, current_chunk.copy(), current_idx, recorder),
                        daemon=True,
                    )
                    pending_thread.start()

            # =========================================================
            # 4) 执行当前动作（后台可同时在推理）
            # =========================================================
            action = current_chunk[current_idx]
            current_time = time.time()
            # print(f"[+{(current_time - last_moving_time)*1000:.1f}ms] ({current_idx}/{current_len}) {list(action)}")
            last_moving_time = current_time

            ros_operator.puppet_arm_publish('puppet', action)
            execute_history.append((t, "action", action.tolist()))

            t += 1
            rate.sleep()

            # =========================================================
            # 5) 判断是否需要等待后台结果
            #
            # 统一逻辑：
            # - 只要后台线程存在
            # - 且当前已经执行到“触发后 delay_steps”位置
            #   或者当前 chunk 快耗尽
            # 就等待
            #
            # 当 pos_inference_step == 0 时，
            # 该条件自然退化成：触发后立即等待
            # =========================================================
            if pending_thread is not None:
                next_executed_since_switch = t - chunk_start_t
                next_idx = current_chunk_base_idx + next_executed_since_switch
                current_len = len(current_chunk)

                required_delay = max(0, args.pos_inference_step)
                trigger_executed_since_switch = None
                if pending_trigger_t is not None:
                    trigger_executed_since_switch = pending_trigger_t - chunk_start_t

                should_wait_for_result = False
                if trigger_executed_since_switch is not None:
                    with inference_lock:
                        inference_result_ready = inference_actions is not None

                    if args.merge and inference_result_ready:
                        # Merge needs the old chunk frames that are still
                        # available when inference actually finishes. Waiting
                        # for the full delay budget would consume that overlap.
                        should_wait_for_result = True
                    else:
                        should_wait_for_result = (
                            next_executed_since_switch
                            >= trigger_executed_since_switch + required_delay
                            or next_idx >= current_len
                        )

                if should_wait_for_result:
                    model_wait_start = time.perf_counter()
                    pending_thread.join()
                    _warn_model_wait(
                        ros_operator.node,
                        model_wait_start,
                        args.frame_rate,
                        "chunk switch",
                    )
                    pending_thread = None

                    with inference_lock:
                        if inference_actions is not None:
                            next_chunk = inference_actions.copy()
                            meta = inference_meta if inference_meta is not None else {}
                            inference_actions = None
                            inference_meta = None

                            next_chunk_base_idx = int(meta.get("delay_steps", required_delay))
                            next_chunk_base_idx = min(next_chunk_base_idx, len(next_chunk) - 1)

                            if args.merge:
                                elapsed_since_trigger = max(
                                    0,
                                    next_executed_since_switch
                                    - trigger_executed_since_switch,
                                )
                                merge_start_idx = min(
                                    next_chunk_base_idx,
                                    int(elapsed_since_trigger),
                                )
                                merge_budget = (
                                    next_chunk_base_idx - merge_start_idx
                                )
                                next_chunk, merged_frames = _merge_rtc_overlap(
                                    current_chunk,
                                    next_idx,
                                    next_chunk,
                                    merge_start_idx,
                                    max_overlap=merge_budget,
                                )
                                next_chunk_base_idx = merge_start_idx
                                if merged_frames > 0:
                                    # ros_operator.node.get_logger().info(
                                    #     f"Merged {merged_frames} overlapping RTC frames "
                                    #     f"(old {next_idx}/{len(current_chunk)}, "
                                    #     f"new {next_chunk_base_idx}/{len(next_chunk)})."
                                    # )
                                    pass
                                else:
                                    ros_operator.node.get_logger().warning(
                                        "Merge requested, but no overlapping RTC frames remain; "
                                        "switching directly."
                                    )

                            bridge_actions = (
                                None if args.merge else meta.get("bridge_actions", None)
                            )
                            if bridge_actions is not None and len(bridge_actions) > 0:
                                bridge_chunk = np.asarray(bridge_actions, dtype=np.float64)
                                bridge_start_t = t
                            else:
                                bridge_chunk = None

                            current_chunk = next_chunk
                            current_chunk_base_idx = next_chunk_base_idx

                            # 若先执行 bridge，则新 chunk 的“逻辑起点”在 bridge 结束之后
                            if bridge_chunk is not None:
                                chunk_start_t = t + len(bridge_chunk)
                            else:
                                chunk_start_t = t

                            pending_trigger_t = None
                            pending_trigger_chunk_idx = None
                        else:
                            ros_operator.node.get_logger().warning(
                                "Inference thread finished but produced no actions; "
                                "continue current chunk if possible."
                            )
    except KeyboardInterrupt:
        ros_operator.node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        end_time = time.time()
        ros_operator.node.get_logger().info(f"Total execution time: \033[1;32m{end_time - start_time:.2f}s\033[0m")

# --- add new replay loop (place near model_inference) ---
def model_replay(args, ros_operator):
    global execute_history
    set_seed(1000)

    replay_actions = load_replay_actions(args.replay, args.replay_field)
    puppet_keys = [key for key in ros_operator.arm_keys if 'puppet' in key]
    if not puppet_keys:
        raise RuntimeError("Replay latency measurement requires at least one puppet arm")

    target_arm_count = len(puppet_keys)
    target_dimension = 7 * target_arm_count
    source_arm_count = replay_actions.shape[1] // 7
    controlled_arm_count = min(source_arm_count, target_arm_count)
    controlled_dimension = 7 * controlled_arm_count

    if source_arm_count > target_arm_count:
        print(
            "Replay contains more arms than the configured target; "
            f"using the first {target_arm_count} arm(s)."
        )
        replay_actions = replay_actions[:, :target_dimension]

    per_arm_window_sizes = [5, 7, 7, 9, 5, 5, 1]
    per_arm_strengths = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
    smooth_window_sizes = per_arm_window_sizes * controlled_arm_count
    smooth_strengths = per_arm_strengths * controlled_arm_count
    if args.smooth:
        replay_actions = smooth_replay_actions(
            replay_actions,
            smooth_window_sizes,
            smooth_strengths,
        )
        print(
            "Replay smoothing enabled: "
            f"windows={smooth_window_sizes}, strengths={smooth_strengths}"
        )
    traj_len = replay_actions.shape[0]

    latency_trace = LatencyTrace()
    expected_dimension = target_dimension
    feedback_recorder = SynchronizedFeedbackRecorder(puppet_keys, latency_trace)
    latency_subscriptions = []
    for arm_key in puppet_keys:
        arm_config = ros_operator.config['arms'][arm_key]
        feedback_topic = arm_config.get('recv_joint_state_topic')
        if feedback_topic is None:
            raise ValueError(f"Arm '{arm_key}' has no recv_joint_state_topic")

        def feedback_callback(message, key=arm_key):
            feedback_recorder.update(key, message.position, time.perf_counter())

        latency_subscriptions.append(
            ros_operator.node.create_subscription(
                JointState,
                feedback_topic,
                feedback_callback,
                100,
            )
        )

    reset_pose = [0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
    prepare_pose = [0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 1.5, -1.0, 0.0, 0.0, 0.0]
    controlled_reset_pose = reset_pose[:controlled_dimension]
    controlled_prepare_pose = prepare_pose[:controlled_dimension]

    if args.resume:
        ros_operator.puppet_arm_publish_continuous(
            'puppet', controlled_reset_pose
        )
        return

    if args.stand:
        ros_operator.puppet_arm_publish_continuous(
            'puppet', controlled_prepare_pose
        )
    else:
        ros_operator.puppet_arm_publish_continuous(
            'puppet', controlled_reset_pose
        )

    if controlled_arm_count < target_arm_count:
        wait_rate = ros_operator.node.create_rate(args.frame_rate)
        stationary_poses = []
        for arm_key in puppet_keys[controlled_arm_count:]:
            while rclpy.ok() and not ros_operator.arm_deques[arm_key]:
                ros_operator.node.get_logger().info(
                    f"Waiting for {arm_key} current pose before replay..."
                )
                wait_rate.sleep()
            if not rclpy.ok():
                return
            stationary_poses.extend(
                list(ros_operator.arm_deques[arm_key][-1].position[:7])
            )

        stationary_actions = np.tile(
            np.asarray(stationary_poses, dtype=np.float64),
            (traj_len, 1),
        )
        replay_actions = np.concatenate(
            [replay_actions, stationary_actions], axis=1
        )
        print(
            "Replay contains fewer arms than the configured target; "
            "uncommanded arms will hold their current feedback poses."
        )

    print("Moving puppet arms slowly to the first replay frame.")
    ros_operator.puppet_arm_publish_continuous(
        'puppet', replay_actions[0].tolist()
    )

    input("Enter any key to continue :")

    rate = ros_operator.node.create_rate(args.frame_rate)
    last_moving_time = time.time()
    start_time = time.time()

    try:
        # publish sequentially, one frame per tick
        max_steps = min(args.max_publish_step, traj_len)
        for t in range(max_steps):
            if not rclpy.ok():
                break
            action7 = replay_actions[t]  # (7,)
            current_time = time.time()
            print(f"[+{(current_time - last_moving_time)*1000:.1f}ms] [REPLAY] ({t+1}/{max_steps}) {list(action7)}")
            last_moving_time = current_time

            publish_time = time.perf_counter()
            ros_operator.puppet_arm_publish('puppet', action7)
            latency_trace.record_command(action7, publish_time)
            execute_history.append((t, "replay", action7.tolist()))
            rate.sleep()

        if max_steps > 0 and rclpy.ok():
            final_action = replay_actions[max_steps - 1]
            settle_deadline = time.perf_counter() + args.replay_settle_seconds
            while rclpy.ok() and time.perf_counter() < settle_deadline:
                ros_operator.puppet_arm_publish('puppet', final_action)
                rate.sleep()
    except KeyboardInterrupt:
        ros_operator.node.get_logger().info("Interrupted by user, shutting down.")
    finally:
        end_time = time.time()
        ros_operator.node.get_logger().info(
            f"Total execution time: \033[1;32m{end_time - start_time:.2f}s\033[0m"
        )

    if latency_trace.snapshot()[0].size:
        report = analyze_latency_trace(
            latency_trace,
            gripper_indices=list(range(6, expected_dimension, 7)),
            analysis_rate_hz=args.replay_analysis_rate,
            max_lag_ms=args.replay_max_lag_ms,
            warmup_seconds=args.replay_latency_warmup,
            settle_tolerance=args.replay_settle_tolerance,
            settle_hold_seconds=args.replay_settle_hold_seconds,
        )
        report["replay"] = {
            "platform": "openeai_arm_ros2",
            "dataset_path": str(Path(args.replay).resolve()),
            "dataset_field": args.replay_field,
            "publish_rate_hz": float(args.frame_rate),
            "camera_subscriptions_enabled": bool(ros_operator.config.get('cameras', {})),
        }
        print_latency_report(report)
        report_path = args.replay_latency_report
        if report_path is None:
            report_path = f"{Path(args.replay).stem}_openeai_latency.json"
        save_latency_report(report, report_path)
        print(f"Saved latency report: {report_path}")

    if rclpy.ok():
        print("Moving puppet arms slowly back to the all-zero pose.")
        ros_operator.puppet_arm_publish_continuous(
            'puppet', np.zeros(expected_dimension, dtype=np.float64).tolist()
        )

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--max_publish_step', action='store', type=int, help='max_publish_step', default=100000, required=False)
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
    parser.add_argument('--pos_inference_step', action='store', type=int, help='pos_inference_step',
                        default=0, required=False)
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size',
                        default=50, required=False)
    parser.add_argument('--server_host', action='store', type=str,
                       help='Model server host. Not required in replay mode.', default=None, required=False)
    parser.add_argument('--server_port', action='store', type=int,
                       help='Model server port', default=8000, required=False)
    parser.add_argument('--contiguous', action='store_true',
                       help='Run the policy without confirmation. Make sure your policy works before enabling this option.', default=False, required=False)
    parser.add_argument('--model', type=str, default='openeai', help='Model type on the server. Available options: [openeai, pi, act, octo]', required=False, choices=['openeai', 'pi', 'act', 'octo'])
    parser.add_argument('-r', '--resume', default=False, required=False, action='store_true')
    parser.add_argument('--smooth', default=False, required=False, action='store_true')
    parser.add_argument(
        '--merge',
        default=False,
        required=False,
        action='store_true',
        help='Cross-fade overlapping RTC chunk frames without adding bridge frames.',
    )
    parser.add_argument('--stand', default=False, required=False, action='store_true')
    parser.add_argument('--use_tactile', default=False, required=False, action='store_true')
    parser.add_argument('--plot', default=False, required=False, action='store_true')

    parser.add_argument('--replay', type=str, default=None,
                        help='Replay from hdf5 file. If set, bypass model inference and publish actions sequentially.')
    parser.add_argument('--replay_field', type=str, default='action',
                        help='Dataset field containing one or more complete 7-DoF arm actions.')
    parser.add_argument('--replay_settle_seconds', type=float, default=2.0,
                        help='Continue holding the final replay target while collecting feedback.')
    parser.add_argument('--replay_settle_tolerance', type=float, default=0.02,
                        help='Per-joint terminal position tolerance in radians.')
    parser.add_argument('--replay_settle_hold_seconds', type=float, default=0.2,
                        help='Required continuous in-tolerance time for terminal settling.')
    parser.add_argument('--replay_analysis_rate', type=float, default=200.0,
                        help='Uniform resampling rate used by latency analysis.')
    parser.add_argument('--replay_max_lag_ms', type=float, default=1000.0,
                        help='Maximum trajectory lag considered by latency analysis.')
    parser.add_argument('--replay_latency_warmup', type=float, default=0.5,
                        help='Replay prefix excluded from trajectory lag analysis.')
    parser.add_argument('--replay_latency_report', type=str, default=None,
                        help='Output JSON path for replay latency statistics.')



    args = parser.parse_args()
    if args.merge and args.smooth:
        parser.error("--merge and --smooth cannot be enabled together")
    return args


# def plot_history():
#     """
#     需要全局变量：
#       execute_history: list of (t:int, typ:str, action:list[float])
#         typ in {"replay","action","wait","bridge","discarded", ...}
#         允许同一 t 出现多条（尤其 discarded 与其它类型重合）
#       state_history: list of (t:int, state:list[float])

#     画法（前6维）：
#       - 每个维度一张子图，共 6 张（或不足 6 则取最小）
#       - state：只画散点，不连线
#       - action：画散点；并对“非 discarded 的动作序列”按相邻点连线
#         * 线段颜色取前一个点的类型
#         * discarded 不参与连线（避免把真实执行轨迹串乱），但会以不同 marker/透明度显示
#       - 对齐时间戳：散点的 x 就是各自的 t，因此同一时间会在同一 x 上出现多个 action 点
#     """
#     global execute_history, state_history

#     # --------- 颜色与样式配置 ----------
#     type_colors = {
#         "replay": "C0",
#         "action": "C1",
#         "wait": "C2",
#         "bridge": "C3",
#         "discarded": "C7",   # 计算出但未执行
#     }
#     default_color = "C6"

#     action_marker_executed = "x"
#     action_marker_discarded = "o"
#     action_alpha_executed = 0.95
#     action_alpha_discarded = 0.35

#     state_color = "k"
#     state_alpha = 0.35
#     state_marker = "."

#     # --------- 解析 action 历史 ----------
#     a_t, a_type, a_val = [], [], []
#     for item in execute_history:
#         if not (isinstance(item, (list, tuple)) and len(item) == 3):
#             continue
#         t, typ, vec = item
#         if vec is None:
#             continue
#         vec = np.asarray(vec, dtype=float).reshape(-1)
#         if vec.size == 0:
#             continue
#         a_t.append(int(t))
#         a_type.append(str(typ))
#         a_val.append(vec)

#     if len(a_t) == 0:
#         raise ValueError("execute_history is empty or invalid.")

#     a_t = np.asarray(a_t, dtype=int)
#     a_val = np.asarray(a_val, dtype=float)  # (Na, dim_a)

#     # --------- 解析 state 历史 ----------
#     s_t, s_val = [], []
#     for item in state_history:
#         if not (isinstance(item, (list, tuple)) and len(item) == 2):
#             continue
#         t, vec = item
#         if vec is None:
#             continue
#         vec = np.asarray(vec, dtype=float).reshape(-1)
#         if vec.size == 0:
#             continue
#         s_t.append(int(t))
#         s_val.append(vec)

#     if len(s_t) == 0:
#         raise ValueError("state_history is empty or invalid.")

#     s_t = np.asarray(s_t, dtype=int)
#     s_val = np.asarray(s_val, dtype=float)  # (Ns, dim_s)

#     # --------- 维度（各取前6维，state/action dim 不同则取交集最小） ----------
#     d = min(6, a_val.shape[1], s_val.shape[1])
#     if d <= 0:
#         raise ValueError(f"Not enough dims. action_dim={a_val.shape[1]}, state_dim={s_val.shape[1]}")

#     # --------- 排序（按时间；同时间保持原相对顺序用 stable sort） ----------
#     a_idx = np.argsort(a_t, kind="stable")
#     a_t = a_t[a_idx]
#     a_val = a_val[a_idx]
#     a_type = [a_type[i] for i in a_idx]

#     s_idx = np.argsort(s_t, kind="stable")
#     s_t = s_t[s_idx]
#     s_val = s_val[s_idx]

#     # --------- 为 action 点准备 mask ----------
#     a_type_arr = np.asarray(a_type, dtype=object)
#     mask_discarded = (a_type_arr == "discarded")
#     mask_executed = ~mask_discarded

#     # 执行的 action 序列（用于连线）
#     exec_t = a_t[mask_executed]
#     exec_val = a_val[mask_executed]
#     exec_type = a_type_arr[mask_executed].tolist()

#     # --------- 开始画图 ----------
#     fig, axes = plt.subplots(d, 1, figsize=(14, 2.6 * d), sharex=True)
#     if d == 1:
#         axes = [axes]

#     for k in range(d):
#         ax = axes[k]

#         # 1) state：散点（不连线）
#         ax.scatter(
#             s_t, s_val[:, k],
#             s=18, marker=state_marker,
#             c=state_color, alpha=state_alpha,
#             label=f"state[{k}]"
#         )

#         # 2) action：散点（executed / discarded 分开画）
#         # executed
#         ax.scatter(
#             a_t[mask_executed], a_val[mask_executed, k],
#             s=30, marker=action_marker_executed,
#             c=[type_colors.get(tp, default_color) for tp in a_type_arr[mask_executed].tolist()],
#             alpha=action_alpha_executed,
#             label=f"action[{k}] executed"
#         )
#         # discarded（同时间戳会叠在同一 x 上）
#         if np.any(mask_discarded):
#             ax.scatter(
#                 a_t[mask_discarded], a_val[mask_discarded, k],
#                 s=22, marker=action_marker_discarded,
#                 c=type_colors.get("discarded", default_color),
#                 edgecolors="none",
#                 alpha=action_alpha_discarded,
#                 label=f"action[{k}] discarded"
#             )

#         # 3) action：连线（仅 executed；每一段颜色用前一个点的 type）
#         if exec_t.size >= 2:
#             segments = []
#             seg_colors = []
#             for i in range(exec_t.size - 1):
#                 segments.append([(exec_t[i], exec_val[i, k]), (exec_t[i + 1], exec_val[i + 1, k])])
#                 seg_colors.append(type_colors.get(exec_type[i], default_color))
#             lc = LineCollection(segments, colors=seg_colors, linewidths=1.8, alpha=0.9)
#             ax.add_collection(lc)

#         ax.set_ylabel(f"dim {k}")
#         ax.grid(True, alpha=0.25)

#         # 简洁 legend：类型颜色 + discarded 特殊说明
#         handles = []
#         labels = []

#         # state / executed / discarded 点样式
#         handles.append(plt.Line2D([], [], linestyle="None", marker=state_marker, color="k", alpha=state_alpha, markersize=7))
#         labels.append("state pts")

#         handles.append(plt.Line2D([], [], linestyle="None", marker=action_marker_executed, color="k", alpha=0.9, markersize=7))
#         labels.append("action pts (executed)")

#         if np.any(mask_discarded):
#             handles.append(plt.Line2D([], [], linestyle="None", marker=action_marker_discarded,
#                                       color=type_colors.get("discarded", default_color), alpha=action_alpha_discarded, markersize=7))
#             labels.append("action pts (discarded)")

#         # 线段颜色对应的 type（不把 discarded 放进线段 legend）
#         types_for_legend = [tp for tp in type_colors.keys() if tp != "discarded"]
#         for tp in types_for_legend:
#             handles.append(plt.Line2D([], [], color=type_colors[tp], linewidth=2.0))
#             labels.append(f"exec line: {tp}")

#         ax.legend(handles, labels, loc="best", ncol=3, fontsize=9, framealpha=0.9)

#     axes[-1].set_xlabel("global timestep")
#     fig.suptitle(f"State vs Action history (first {d} dims)", y=0.995)
#     fig.tight_layout()
#     plt.savefig("execution_history.png", dpi=300)
#     plt.show()

#     # =======================
#     # 再画一遍：剪掉所有 bridge 段（时间轴同步前移）
#     # =======================

#     # 统计每个整型时间戳是否属于 bridge
#     bridge_ts = set(a_t[a_type_arr == "bridge"].tolist())

#     if len(bridge_ts) == 0:
#         # 没有 bridge：直接复用原始时间轴数据再画一次（等价于原图）
#         a_t2, a_val2, a_type2 = a_t.copy(), a_val.copy(), a_type_arr.copy()
#         s_t2, s_val2 = s_t.copy(), s_val.copy()
#         removed_cnt = 0
#     else:
#         # 构造“剪掉 bridge”后的时间映射：t' = t - (#bridge timestamps <= t)
#         bridge_sorted = np.array(sorted(bridge_ts), dtype=int)

#         def shift_times(ts: np.ndarray) -> np.ndarray:
#             # 对每个 t，计算有多少 bridge 时间戳 <= t
#             cnt = np.searchsorted(bridge_sorted, ts, side="right")
#             return ts - cnt

#         # 去掉 bridge action 点，并对剩余 action/state 统一平移
#         keep_action = (a_type_arr != "bridge")
#         a_t2 = shift_times(a_t[keep_action])
#         a_val2 = a_val[keep_action]
#         a_type2 = a_type_arr[keep_action]

#         s_t2 = shift_times(s_t)
#         s_val2 = s_val

#         removed_cnt = len(bridge_ts)

#     # 重新做 discarded / executed mask（bridge 已经移除）
#     mask_discarded2 = (a_type2 == "discarded")
#     mask_executed2 = ~mask_discarded2

#     exec_t2 = a_t2[mask_executed2]
#     exec_val2 = a_val2[mask_executed2]
#     exec_type2 = a_type2[mask_executed2].tolist()

#     fig2, axes2 = plt.subplots(d, 1, figsize=(14, 2.6 * d), sharex=True)
#     if d == 1:
#         axes2 = [axes2]

#     for k in range(d):
#         ax = axes2[k]

#         # state：散点
#         ax.scatter(
#             s_t2, s_val2[:, k],
#             s=18, marker=state_marker,
#             c=state_color, alpha=state_alpha,
#             label=f"state[{k}]"
#         )

#         # action：散点（executed / discarded）
#         ax.scatter(
#             a_t2[mask_executed2], a_val2[mask_executed2, k],
#             s=30, marker=action_marker_executed,
#             c=[type_colors.get(tp, default_color) for tp in a_type2[mask_executed2].tolist()],
#             alpha=action_alpha_executed,
#             label=f"action[{k}] executed"
#         )
#         if np.any(mask_discarded2):
#             ax.scatter(
#                 a_t2[mask_discarded2], a_val2[mask_discarded2, k],
#                 s=22, marker=action_marker_discarded,
#                 c=type_colors.get("discarded", default_color),
#                 edgecolors="none",
#                 alpha=action_alpha_discarded,
#                 label=f"action[{k}] discarded"
#             )

#         # executed 连线（颜色取前一个点类型）
#         if exec_t2.size >= 2:
#             segments, seg_colors = [], []
#             for i in range(exec_t2.size - 1):
#                 segments.append([(exec_t2[i], exec_val2[i, k]), (exec_t2[i + 1], exec_val2[i + 1, k])])
#                 seg_colors.append(type_colors.get(exec_type2[i], default_color))
#             lc = LineCollection(segments, colors=seg_colors, linewidths=1.8, alpha=0.9)
#             ax.add_collection(lc)

#         ax.set_ylabel(f"dim {k}")
#         ax.grid(True, alpha=0.25)

#         # legend（bridge 不再出现）
#         handles, labels = [], []
#         handles.append(plt.Line2D([], [], linestyle="None", marker=state_marker, color="k", alpha=state_alpha, markersize=7))
#         labels.append("state pts")
#         handles.append(plt.Line2D([], [], linestyle="None", marker=action_marker_executed, color="k", alpha=0.9, markersize=7))
#         labels.append("action pts (executed)")
#         if np.any(mask_discarded2):
#             handles.append(plt.Line2D([], [], linestyle="None", marker=action_marker_discarded,
#                                       color=type_colors.get("discarded", default_color), alpha=action_alpha_discarded, markersize=7))
#             labels.append("action pts (discarded)")

#         types_for_legend = [tp for tp in type_colors.keys() if tp not in ("discarded", "bridge")]
#         for tp in types_for_legend:
#             handles.append(plt.Line2D([], [], color=type_colors[tp], linewidth=2.0))
#             labels.append(f"exec line: {tp}")

#         ax.legend(handles, labels, loc="best", ncol=3, fontsize=9, framealpha=0.9)

#     axes2[-1].set_xlabel("global timestep (bridge removed & time compressed)")
#     fig2.suptitle(f"History (bridge removed; compressed by {removed_cnt} timesteps) | first {d} dims", y=0.995)
#     fig2.tight_layout()
#     fig2.savefig("execution_history_no_bridge.png", dpi=300)
#     plt.show()

def plot_history():
    """
    需要全局变量：
      execute_history: list of (t:int, typ:str, action:list[float])
      state_history:   list of (t:int, state:list[float])

    布局：6行 × 2列
      左列 = 原始时间轴；右列 = 去掉 bridge 后压缩时间轴
    动作类型：
      PRIMARY_EXEC_TYPES 中的类型视为"主执行动作"——画 x-marker 并互相连线
      其余（含 discarded 以及所有 <type>/<index> 格式类型）视为"旁路动作"
        - 画 o-marker、不连线
        - 右列时间轴：每组旁路动作（相同 type/index）整体平移量与该组第一帧相同
        - 颜色：先查 type_colors，若不存在则从 extra_palette 自动分配
        - 同一 <type> 的不同 <index> 共享同一颜色
    """
    global execute_history, state_history

    # ──────────────────────────────────────────────
    # 0. 配置区
    # ──────────────────────────────────────────────

    PRIMARY_EXEC_TYPES: set = {"replay", "action", "wait", "bridge"}

    type_colors: dict = {
        "replay":  "#4C8BE2",
        "action":  "#E88C1A",
        "wait":    "#3BB87F",
        "bridge":  "#D95F5F",
    }

    extra_palette: list = [
        "#9B59B6", "#1ABC9C", "#F39C12", "#2ECC71",
        "#E74C3C", "#3498DB", "#E91E63", "#FF5722",
        "#607D8B", "#795548",
    ]
    _auto_color_map: dict = {}

    def _base_type(tp: str) -> str:
        """'discarded/3' → 'discarded'；无斜杠则原样返回。"""
        return tp.split("/")[0] if "/" in tp else tp

    def _get_color(tp: str) -> str:
        """
        颜色查找顺序：
          1. 精确匹配 type_colors（如 'replay'）
          2. 以 base_type 匹配 type_colors（如 'discarded/3' → 'discarded'）
          3. 以 base_type 在 _auto_color_map 中自动分配
        """
        if tp in type_colors:
            return type_colors[tp]
        base = _base_type(tp)
        if base in type_colors:
            return type_colors[base]
        if base not in _auto_color_map:
            idx = len(_auto_color_map) % len(extra_palette)
            _auto_color_map[base] = extra_palette[idx]
        return _auto_color_map[base]

    marker_primary   = "x"
    marker_secondary = "o"
    alpha_primary    = 0.95
    alpha_secondary  = 0.75      # ← 旁路透明度

    state_color  = "#E53935"
    state_alpha  = 0.75
    state_marker = "."
    state_size   = 22

    # ──────────────────────────────────────────────
    # 1. 解析 execute_history
    # ──────────────────────────────────────────────
    a_t_list, a_type_list, a_val_list = [], [], []
    for item in execute_history:
        if not (isinstance(item, (list, tuple)) and len(item) == 3):
            continue
        t, typ, vec = item
        if vec is None:
            continue
        vec = np.asarray(vec, dtype=float).reshape(-1)
        if vec.size == 0:
            continue
        a_t_list.append(int(t))
        a_type_list.append(str(typ))
        a_val_list.append(vec)

    if len(a_t_list) == 0:
        raise ValueError("execute_history is empty or invalid.")

    a_t   = np.asarray(a_t_list,  dtype=int)
    a_val = np.asarray(a_val_list, dtype=float)

    # ──────────────────────────────────────────────
    # 2. 解析 state_history
    # ──────────────────────────────────────────────
    s_t_list, s_val_list = [], []
    for item in state_history:
        if not (isinstance(item, (list, tuple)) and len(item) == 2):
            continue
        t, vec = item
        if vec is None:
            continue
        vec = np.asarray(vec, dtype=float).reshape(-1)
        if vec.size == 0:
            continue
        s_t_list.append(int(t))
        s_val_list.append(vec)

    if len(s_t_list) == 0:
        raise ValueError("state_history is empty or invalid.")

    s_t   = np.asarray(s_t_list,  dtype=int)
    s_val = np.asarray(s_val_list, dtype=float)

    # ──────────────────────────────────────────────
    # 3. 维度 & 排序
    # ──────────────────────────────────────────────
    d = min(6, a_val.shape[1], s_val.shape[1])
    if d <= 0:
        raise ValueError(
            f"Not enough dims. action_dim={a_val.shape[1]}, state_dim={s_val.shape[1]}"
        )

    a_idx  = np.argsort(a_t, kind="stable")
    a_t    = a_t[a_idx]
    a_val  = a_val[a_idx]
    a_type = np.asarray(a_type_list, dtype=object)[a_idx]

    s_idx  = np.argsort(s_t, kind="stable")
    s_t    = s_t[s_idx]
    s_val  = s_val[s_idx]

    # ──────────────────────────────────────────────
    # 4. 预先触发所有类型颜色分配（保证两列一致）
    # ──────────────────────────────────────────────
    for tp in np.unique(a_type):
        _get_color(str(tp))

    # ──────────────────────────────────────────────
    # 5. bridge 压缩：构造右列数据
    # ──────────────────────────────────────────────
    is_primary_arr = np.array(
        [tp in PRIMARY_EXEC_TYPES for tp in a_type], dtype=bool
    )
    is_secondary_arr = ~is_primary_arr

    bridge_ts_set = set(a_t[a_type == "bridge"].tolist())

    if len(bridge_ts_set) == 0:
        a_t2    = a_t.copy()
        a_val2  = a_val.copy()
        a_type2 = a_type.copy()
        s_t2    = s_t.copy()
        s_val2  = s_val.copy()
        removed_cnt = 0
    else:
        bridge_sorted = np.array(sorted(bridge_ts_set), dtype=int)

        def _shift(ts: np.ndarray) -> np.ndarray:
            """对每个时间戳，减去"不超过它的 bridge 时间戳数量"。"""
            return ts - np.searchsorted(bridge_sorted, ts, side="right")

        # ── 主执行（非 bridge 的 primary）：正常压缩时间 ──
        keep_primary = is_primary_arr & (a_type != "bridge")
        at_primary   = _shift(a_t[keep_primary])
        av_primary   = a_val[keep_primary]
        atype_primary = a_type[keep_primary]

        # ── 旁路（secondary）：每组整体平移量锚定到该组第一帧 ──
        #    "组"由完整 type 字符串区分（如 'discarded/1' 是一组）
        keep_secondary = is_secondary_arr
        at_sec_orig    = a_t[keep_secondary]
        av_sec         = a_val[keep_secondary]
        atype_sec      = a_type[keep_secondary]

        at_sec_shifted = np.empty_like(at_sec_orig)

        # 按组（完整 type 字符串）计算平移量
        unique_sec_types = np.unique(atype_sec)
        for grp in unique_sec_types:
            grp_mask = (atype_sec == grp)
            grp_ts   = at_sec_orig[grp_mask]
            # 该组第一帧的时间戳（排序后最小值）
            first_t  = grp_ts.min()
            # 锚定平移量 = 第一帧对应的 bridge 压缩偏移量
            anchor_shift = int(np.searchsorted(bridge_sorted, first_t, side="right"))
            at_sec_shifted[grp_mask] = grp_ts - anchor_shift

        # ── 合并主执行 + 旁路，按时间重排 ──
        a_t2    = np.concatenate([at_primary,    at_sec_shifted])
        a_val2  = np.concatenate([av_primary,    av_sec])
        a_type2 = np.concatenate([atype_primary, atype_sec])

        reorder = np.argsort(a_t2, kind="stable")
        a_t2    = a_t2[reorder]
        a_val2  = a_val2[reorder]
        a_type2 = a_type2[reorder]

        s_t2    = _shift(s_t)
        s_val2  = s_val
        removed_cnt = len(bridge_ts_set)

    # ──────────────────────────────────────────────
    # 6. 通用绘图函数
    # ──────────────────────────────────────────────
    def _draw_column(axes_col, at, av, atype, st, sv):
        """在 axes_col（长度 d）中绘制一列。"""
        mask_primary   = np.array([tp in PRIMARY_EXEC_TYPES for tp in atype], dtype=bool)
        mask_secondary = ~mask_primary

        pt    = at[mask_primary]
        pv    = av[mask_primary]
        ptype = atype[mask_primary].tolist()

        secondary_types_present = sorted(set(atype[mask_secondary].tolist()))

        for k in range(d):
            ax = axes_col[k]

            # ① state
            ax.scatter(
                st, sv[:, k],
                s=state_size, marker=state_marker,
                c=state_color, alpha=state_alpha,
                zorder=2
            )

            # ② 主执行散点
            if mask_primary.any():
                ax.scatter(
                    at[mask_primary], av[mask_primary, k],
                    s=40, marker=marker_primary,
                    c=[_get_color(tp) for tp in atype[mask_primary].tolist()],
                    alpha=alpha_primary,
                    zorder=3
                )

            # ③ 旁路散点（所有非主执行类型，含 discarded 及 <type>/<index>）
            if mask_secondary.any():
                ax.scatter(
                    at[mask_secondary], av[mask_secondary, k],
                    s=25, marker=marker_secondary,
                    c=[_get_color(tp) for tp in atype[mask_secondary].tolist()],
                    edgecolors="none",
                    alpha=alpha_secondary,
                    zorder=2
                )

            # ④ 主执行连线
            if pt.size >= 2:
                segments, seg_colors = [], []
                for i in range(pt.size - 1):
                    segments.append(
                        [(pt[i],   pv[i,   k]),
                         (pt[i+1], pv[i+1, k])]
                    )
                    seg_colors.append(_get_color(ptype[i]))
                lc = LineCollection(
                    segments, colors=seg_colors,
                    linewidths=1.8, alpha=0.88, zorder=1
                )
                ax.add_collection(lc)

            ax.set_ylabel(f"dim {k}", fontsize=9)
            ax.grid(True, alpha=0.22)

            # ── legend ──
            h, lb = [], []

            h.append(plt.Line2D([], [], ls="None", marker=state_marker,
                                color=state_color, alpha=state_alpha, ms=8))
            lb.append("state")

            # 主执行：出现过的类型
            primary_present = sorted(set(atype[mask_primary].tolist()))
            for tp in primary_present:
                h.append(plt.Line2D([], [], color=_get_color(tp),
                                    linewidth=2.0, marker=marker_primary,
                                    ms=7, alpha=alpha_primary))
                lb.append(f"exec: {tp}")

            # 旁路：按 base_type 去重显示（同 base_type 颜色相同，合并展示）
            seen_base = {}
            for tp in secondary_types_present:
                base = _base_type(tp)
                if base not in seen_base:
                    seen_base[base] = _get_color(tp)
            for base, color in seen_base.items():
                h.append(plt.Line2D([], [], ls="None", marker=marker_secondary,
                                    color=color, alpha=alpha_secondary, ms=7))
                lb.append(f"side: {base}")

            ax.legend(h, lb, loc="best", ncol=2,
                      fontsize=8, framealpha=0.88)

        axes_col[-1].set_xlabel("timestep", fontsize=9)

    # ──────────────────────────────────────────────
    # 7. 构建 d×2 画布
    # ──────────────────────────────────────────────
    fig, axes_all = plt.subplots(
        d, 2,
        figsize=(18, 2.8 * d),
        sharex="col"
    )

    if d == 1:
        axes_all = np.array([axes_all])

    axes_left  = [axes_all[k, 0] for k in range(d)]
    axes_right = [axes_all[k, 1] for k in range(d)]

    _draw_column(axes_left,  a_t,  a_val,  a_type,  s_t,  s_val)
    _draw_column(axes_right, a_t2, a_val2, a_type2, s_t2, s_val2)

    axes_left[0].set_title("Original timeline",
                            fontsize=10, fontweight="bold")
    axes_right[0].set_title(
        f"Bridge removed (compressed by {removed_cnt} steps)",
        fontsize=10, fontweight="bold"
    )

    fig.suptitle(
        f"State vs Action history  —  first {d} dims",
        fontsize=12, fontweight="bold", y=1.002
    )
    fig.tight_layout()

    out_path = "execution_history_combined.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.show()
    print(f"[plot_history] saved → {out_path}")

def main():
    global execute_history, state_history
    args = get_arguments()
    ros_operator = OpenEAIArmROS2Operator(args.config_path)
    execute_history = []
    state_history = []
    if args.replay is None:
        if args.server_host is None:
            raise ValueError("--server_host is required for model inference")
        initialize_client(args)
        model_inference(args, ros_operator)
    else:
        # replay mode: no server needed
        model_replay(args, ros_operator)
    if args.plot:
        plot_history()

if __name__ == '__main__':
    main()

# python ./inference.py --task_instruction "finish the task" --server_host 127.0.0.1 --contiguous --model openeai
