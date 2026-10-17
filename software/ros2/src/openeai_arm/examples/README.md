# Inference and rollout examples

Run examples from `software/ros2/src/openeai_arm/examples` after building the C++ library and ROS 2 packages and sourcing `software/ros2/install/setup.bash`.

## Python dependencies

Use Python 3.10 and the ROS 2 Humble Python environment. Install compatible PyTorch and:

```bash
pip install numpy scipy h5py matplotlib opencv-python Pillow requests msgpack einops PyYAML pyzmq pynput
sudo apt install python3-tk ros-humble-cv-bridge
```

ROS 2 modules (`rclpy`, `sensor_msgs`, and `cv_bridge`) must be importable from the selected environment. Keyboard capture uses a desktop session. Start the arm nodes in joint-control mode (`ee_pose:=0`) and match configured arm and camera topics to the publishers. VR uses `ee_pose:=2` and must not publish to the same arm concurrently.

## Entry points

| File | Purpose |
| --- | --- |
| `inference.py` | Asynchronous inference, RTC overlap merge, optional smoothing, and replay |
| `rollout.py` | Autonomous inference with asynchronous HDF5 recording |
| `hil.py` | Previous human-in-the-loop program, moved from dev's former `rollout.py` |
| `collect_data.py` | Manual demonstration recording |

The optional recorder hooks leave ordinary `inference.py` execution unchanged when no recorder is supplied. Per-request ROS rate timers were removed to prevent timers accumulating during long inference runs.

## Autonomous collection

```bash
python rollout.py \
    --config_path config_multi_inference.yml \
    --server_host 127.0.0.1 \
    --task_name pick_place \
    --dataset_dir ./data \
    --task_instruction "Pick up the object and place it in the tray" \
    --frame_rate 50 \
    --pos_lookahead_step 2 \
    --pos_inference_step 24 \
    --merge --stand --contiguous
```

`bash rollout.sh --task_name pick_place` supplies example task and server settings; trailing arguments override its defaults. The initial pose move and confirmation still run. Recording starts after confirmation unless `--no_auto_start` is supplied. `--contiguous` skips repeated policy confirmations, not the initial confirmation.

Keyboard controls:

- `s`: start or end recording.
- `n`: save the episode, leave inference, and repeat preparation for the next episode.
- `r`: discard the episode and repeat preparation.
- `t`: annotate a completed delivery at the most recent recording step.
- `Ctrl+C`: stop and finalize the current recording.

The ZMQ channel (port 5604 by default) accepts `episode_start`, `episode_end:<reason>`, and `outcome:<reason>`. The collector also accepts key events from that channel. Run one collector per output stream.

`--dry_run` suppresses this collector's arm-command publishing while exercising inference and recording. It still requires the configured model service and sensor feedback.

## Recording format and timing

Episodes are written as `<dataset_dir>/<task_name>/episode_<index>.hdf5`, using a temporary file until finalization. Each file contains:

- `observations/qpos`, `qvel`, `effort`, and `eef_pose`.
- `action`, the command in effect at each feedback sample.
- `observations/images`, optional depth, and elapsed `timestamps`.
- `key_frame`, per-request `inference` traces, and sensor-time `alignment` metadata.

Feedback sampling follows incoming arm feedback rather than forcing 50 Hz. The `--frame_rate` command clock controls publishing independently. Raising it changes the physical duration of model chunks. Fixed-rate recording is available with `--record_rate fixed --record_hz 50`.

Recorded `action` is sampled on the recording timebase; it is not an archive of every command published at a higher frequency. The separate local experiment scripts and their `--save` extension are not included in this release.

## Replay and smoothing

```bash
python inference.py --config_path config_right_inference.yml \
    --replay ./data/pick_place/episode_0.hdf5 --frame_rate 50
```

Replay slowly moves to the first action, publishes the trajectory, holds the final target for feedback analysis, and returns to the all-zero control pose. It writes a latency report. Actions contain complete 7-dimensional arm vectors. Single-arm playback uses the first arm from wider data; a missing second arm holds its initial state. Set `cameras: {}` in a dedicated operator configuration to avoid camera subscriptions during latency measurements.

`--smooth` applies chunk smoothing and bridge transitions; `--merge` blends overlapping RTC frames. They are mutually exclusive. Use `hil.py` for model/teaching-mode switching.

## Changed inference defaults

- `max_publish_step` is now 100000.
- The inference reset pose is `[0, 0.2, 0.3, -0.2, 0, 0, 0]` for each arm.
- OpenEAI requests include `recap_condition: positive`; the service must accept this field.
- Several per-cycle latency/merge prints and the model-wait warning are disabled.
- Edit example ports, topics, tasks, and arm calibration for the deployment.
