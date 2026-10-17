"""Autonomous asynchronous rollout collection.

Runs the existing asynchronous policy loop from :mod:`inference` (chunk
scheduling, RTC merge, bridge generation, smoothing) and records the result.

Two clocks are kept deliberately separate:

* the **sensor/feedback clock** (~50-60 Hz, the SFT timebase) drives the
  recorded trajectory: one sample is taken per new arm-feedback message;
* the **command clock** (``--frame_rate``, may be 100/125 Hz) and the
  **request clock** (RTC scheduling) run freely -- they only update the command
  that is "in effect" for the next recorded sample (zero-order hold).

The recorder never publishes robot commands and never blocks the control loop:
samples are queued and a dedicated writer thread does image encoding and HDF5
writing into ``<...>.hdf5.tmp``, renamed to the final name on finalize.

One HDF5 per episode (single file, different fields):

* SFT-compatible: ``observations/qpos|qvel|effort``, ``observations/eef_pose``,
  ``action``, ``observations/images/cam_<name>``, ``timestamps`` (elapsed
  monotonic seconds since episode start);
* ``/key_frame`` [K] int64 -- delivery annotations (the ``recording_step`` of
  the last completed interval at the time of each ``t`` event);
* ``/inference/`` -- one row per policy request: the observation actually used,
  the *complete* predicted chunk (executed or not), and scheduling metadata;
* ``/alignment/`` -- per-sample original sensor timestamps for staleness checks.

Episode lifecycle comes from the online task script over the existing ZeroMQ
key channel (``episode_start`` / ``episode_end:<reason>`` / ``outcome:<reason>``)
with the local keyboard (``s`` start/toggle, ``n`` end+save, ``r`` discard,
``t`` annotate one new delivery) as the manual fallback. Task success/failure
and timeouts are owned by the online script; this collector only records.
"""
import argparse
import base64
import bisect
import collections
import glob
import os
import signal
import sys
import threading
import time

import cv2
import h5py
import numpy as np
import yaml
import zmq
from pynput import keyboard
from sensor_msgs.msg import Image, JointState

import inference as inf


# ---------------------------------------------------------------- utilities


def _to_sec(msg):
    stamp = msg.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def _latest_before(msgs, target_ts):
    """Newest message whose header stamp is <= target_ts (causal alignment).

    Picks the last frame captured at or before the sample instant, so a recorded
    camera image can never be newer than the arm state it is paired with. Falls
    back to the oldest available only if none qualifies (e.g. clock skew).
    """
    best = None
    best_ts = None
    for m in msgs:
        s = _to_sec(m)
        if s <= target_ts and (best_ts is None or s >= best_ts):
            best, best_ts = m, s
    return best if best is not None else msgs[0]


def _encode_image(img, ext=".jpg"):
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        raise ValueError("cv2.imencode failed")
    return base64.b64encode(buf)


def _puppet_keys(operator):
    return [k for k in operator.arm_keys if "puppet" in k]


def _arm_state(msgs, keys):
    """Measured state, same joint ordering / units as collect_data.py.

    qpos is reordered by joint name (joint_1..joint_7); qvel/effort are taken in
    the raw message order, exactly as collect_data.py does.
    """
    qpos = np.zeros(0)
    qvel = np.zeros(0)
    effort = np.zeros(0)
    eef = np.zeros(0)
    for key in keys:
        msg = msgs.get(key)
        if msg is None:
            return None
        names = np.array(msg.name)
        pos = np.asarray(msg.position, dtype=np.float64)
        v_raw = np.asarray(msg.velocity, dtype=np.float64)
        e_raw = np.asarray(msg.effort, dtype=np.float64)
        q_arm = np.zeros(7)
        for i in range(1, 8):
            idx = np.where(names == f"joint_{i}")[0]
            if len(idx):
                q_arm[i - 1] = pos[idx[0]]
        qpos = np.concatenate((qpos, q_arm))
        qvel = np.concatenate((qvel, v_raw))
        effort = np.concatenate((effort, e_raw))
        pose = msgs.get(key + "_pose")
        if pose is not None:
            eef = np.concatenate((eef, np.asarray(pose.position, dtype=np.float64)))
    return qpos, qvel, effort, eef


# ------------------------------------------------------------- the recorder


class RolloutRecorder:
    """Observes the policy loop and records it. Never publishes commands."""

    MAX_QUEUE = 4000

    def __init__(self, args, operator, config, settings, next_episode_idx):
        self.args = args
        self.operator = operator
        self.config = config
        self.settings = settings
        self.camera_names = list(config.get("cameras", {}).keys())
        # only record depth when the operator actually subscribed to a depth topic
        self.use_depth = any("depth" in types for types in operator.img_deques.values())

        self.puppet_keys = _puppet_keys(operator)
        self.primary_key = self.puppet_keys[0] if self.puppet_keys else None
        n_arm = len(self.puppet_keys)
        self.state_dim = 7 * n_arm
        self.eef_dim = 7 * n_arm

        self._fb = {}                     # latest JointState per record name
        self._fb_cam = {}                 # latest Image per (camera, type)
        self._stop_requested = False
        self._cycle_requested = False
        self._auto_start = not getattr(args, "no_auto_start", False)
        self.annot_timeout = float(getattr(args, "annot_timeout", 30.0))
        self._last_annot_wall = None     # when the last delivery was annotated
        self._annot_warned = False       # one reminder per timeout window
        self._install_subscriptions()

        self.next_episode_idx = next_episode_idx
        self._lock = threading.Lock()
        self._jobs = collections.deque()
        self._jobs_cv = threading.Condition()
        self._stop = False

        # episode state
        self._recording = False
        self._episode_idx = None
        self._step = 0
        self._t0 = None
        self._sample_wall = []
        self._key_frames = []
        self._dropped = 0

        # command zero-order hold
        self._last_cmd = None
        # last well-formed state components (joint_states sometimes omit
        # velocity/effort or a pose topic is momentarily empty)
        self._prev = {}

        # image de-dup caches
        self._cam_msg = {}
        self._cam_encoded = {}
        self._cam_stamp = {}

        self._reset_requested = False
        self._epoch = 0                  # bumped per episode; stale trace jobs dropped
        self._obs_epoch = 0              # epoch of the in-flight request's observation
        self._sample_on_feedback = (args.record_rate == "feedback")
        self._finalize_done = threading.Event()
        self.episodes_written = 0

        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()
        self._status = threading.Thread(target=self._status_loop, daemon=True)
        self._status.start()
        if self.args.record_rate == "fixed":
            self._timer = threading.Thread(target=self._timer_loop, daemon=True)
            self._timer.start()

    # ---------------------------------------------------- hooks (policy loop)

    def note_command(self, target):
        """Record the command currently in effect (called on every publish)."""
        with self._lock:
            self._last_cmd = np.asarray(target, dtype=np.float64)

    def consume_reset_request(self):
        with self._lock:
            if self._reset_requested:
                self._reset_requested = False
                return True
        return False

    def on_ready(self):
        """Called by the policy loop right after the start-up pose + confirmation."""
        if self._auto_start and not self._recording:
            self.start_episode()

    def request_stop(self):
        """Ask the policy loop to leave so the entry point can re-run the
        start-up sequence (reset pose + confirm) for the next episode."""
        with self._lock:
            self._stop_requested = True
            self._cycle_requested = True

    def consume_stop_request(self):
        with self._lock:
            if self._stop_requested:
                self._stop_requested = False
                return True
        return False

    def consume_cycle_request(self):
        with self._lock:
            if self._cycle_requested:
                self._cycle_requested = False
                return True
        return False

    def on_observation(self, global_t, timestamp, observation):
        state = np.asarray(observation["state"], dtype=np.float64)
        with self._lock:
            self._obs_epoch = self._epoch
            self._push(("trace_obs", {
                "epoch": self._obs_epoch,
                "wall": time.perf_counter(),
                "global_t": int(global_t),
                "timestamp": float(timestamp),
                "state": state,
                "images": self._clone_images(observation.get("images")),
            }))

    def on_prediction(self, global_t, chunk, meta):
        with self._lock:
            # the request may have been issued before an episode boundary; keep its
            # own epoch so a discarded in-flight result cannot leak into the new one
            self._push(("trace_pred", {
                "epoch": self._obs_epoch,
                "global_t": int(global_t),
                "chunk": np.asarray(chunk, dtype=np.float32),
                "meta": dict(meta),
            }))

    # ------------------------------------------------------ episode control

    def start_episode(self):
        with self._lock:
            if self._recording:
                return False
            self._episode_idx = self.next_episode_idx
            self._step = 0
            self._t0 = time.perf_counter()
            self._sample_wall = []
            self._key_frames = []
            self._dropped = 0
            self._prev = {}
            self._cam_msg = {}
            self._cam_encoded = {}
            self._cam_stamp = {}
            self._recording = True
            self._reset_requested = True     # invalidate in-flight chunk/results
            self._last_annot_wall = time.perf_counter()
            self._annot_warned = False
            self._epoch += 1
            self._push(("start", {"episode_idx": self._episode_idx,
                                  "epoch": self._epoch,
                                  "settings": dict(self.settings)}))
            idx = self._episode_idx
        self.operator.node.get_logger().info(
            f"[rollout] \u25b6 RECORDING episode_{idx} | "
            f"'t' annotate delivery, 'n' save+end, 's' end, 'r' discard")
        return True

    def end_episode(self, reason=None, outcome=None):
        with self._lock:
            if not self._recording:
                return False
            self._recording = False
            end_time = time.perf_counter() - self._t0
            self._key_frames.sort()
            self._finalize_done.clear()
            self._push(("finalize", {
                "episode_idx": self._episode_idx,
                "end_time": end_time,
                "key_frames": list(self._key_frames),
                "walls": list(self._sample_wall),
                "reason": reason,
                "outcome": outcome,
            }))
            idx = self._episode_idx
            self.next_episode_idx += 1
            self.episodes_written += 1
        self.operator.node.get_logger().info(
            f"[rollout] \u25a0 episode_{idx} stopped (reason={reason or 'operator'}), writing...")
        return True

    def discard_episode(self):
        with self._lock:
            if not self._recording:
                return False
            self._recording = False
            self._reset_requested = True
            idx = self._episode_idx
            self._push(("abort", {"episode_idx": idx}))
        self.operator.node.get_logger().warning(
            f"[rollout] episode_{idx} DISCARDED (not saved)")
        return True

    def annotate(self, event_wall=None):
        """One new confirmed delivery -> recording_step of the last completed sample."""
        if event_wall is None:
            event_wall = time.perf_counter()
        with self._lock:
            if not self._recording:
                return False, "no active recording"
            if not self._sample_wall:
                return False, "no completed sample yet"
            i = bisect.bisect_right(self._sample_wall, event_wall) - 1
            if i < 0:
                return False, "event precedes first sample"
            self._key_frames.append(i)      # sample index == recording_step (1:1)
            self._last_annot_wall = time.perf_counter()
            self._annot_warned = False
            return True, i

    @property
    def recording(self):
        return self._recording

    # ------------------------------------------------------------- sampler

    def _install_subscriptions(self):
        """Own subscriptions so every feedback message / camera frame is captured.

        Reading the operator's deques instead would miss messages (and leave empty
        images) whenever the inference thread drains them.
        """
        arms = self.config.get("arms", {})
        for key in self.puppet_keys:
            cfg = arms.get(key, {})
            topic = cfg.get("recv_joint_state_topic")
            if topic:
                self.operator.node.create_subscription(
                    JointState, topic, self._make_fb_cb(key, key == self.primary_key), 50)
            ptopic = cfg.get("recv_eef_pose_topic")
            if ptopic:
                self.operator.node.create_subscription(
                    JointState, ptopic, self._make_fb_cb(key + "_pose", False), 50)
        for cam in self.camera_names:
            cfg = self.config.get("cameras", {}).get(cam, {})
            types = {"rgb": cfg.get("rgb_topic")}
            if self.use_depth:
                types["depth"] = cfg.get("depth_topic")
            for typ, topic in types.items():
                if not topic:
                    continue
                self._fb_cam[(cam, typ)] = collections.deque(maxlen=8)

                def cam_cb(msg, cam=cam, typ=typ):
                    self._fb_cam[(cam, typ)].append(msg)

                self.operator.node.create_subscription(Image, topic, cam_cb, 10)

    def _make_fb_cb(self, name, primary):
        def cb(msg):
            self._fb[name] = msg
            if primary and self._sample_on_feedback and self._recording:
                self._sample_now()
        return cb

    def _sample_now(self):
        frames = self._snapshot()
        if frames is None:
            return
        with self._lock:
            if not self._recording:
                return
            step = self._step
            self._step += 1
            now = time.perf_counter()
            self._sample_wall.append(now)
            action = self._fmt("action",
                               self._last_cmd if self._last_cmd is not None else frames["qpos"],
                               self.state_dim)
            self._push(("sample", {
                "step": step,
                "t_elapsed": now - self._t0,
                "arm_ts": frames["arm_ts"],
                "qpos": frames["qpos"], "qvel": frames["qvel"],
                "effort": frames["effort"], "eef": frames["eef"],
                "action": action, "frames": frames["cam"],
            }))

    def _timer_loop(self):
        """fixed-rate sampling (zero-order hold); feedback mode samples in the callback."""
        period = 1.0 / float(self.args.record_hz)
        next_tick = time.perf_counter()
        while not self._stop:
            next_tick += period
            slack = next_tick - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            if self._recording:
                self._sample_now()

    def _status_loop(self):
        """Periodic recording progress so the operator can see it is working."""
        while not self._stop:
            time.sleep(2.0)
            stale = None
            with self._lock:
                if not self._recording:
                    continue
                ep, step = self._episode_idx, self._step
                elapsed = time.perf_counter() - self._t0
                deliveries = len(self._key_frames)
                if (self.annot_timeout > 0 and self._last_annot_wall is not None
                        and not self._annot_warned):
                    idle = time.perf_counter() - self._last_annot_wall
                    if idle >= self.annot_timeout:
                        self._annot_warned = True     # once per timeout window
                        stale = idle
            self.operator.node.get_logger().info(
                f"[rollout] \033[33mREC\033[0m episode_{ep} | "
                f"step={step} elapsed={elapsed:6.1f}s rate={step / max(elapsed, 1e-9):.1f}Hz "
                f"deliveries={deliveries}")
            if stale is not None:
                self.operator.node.get_logger().warning(
                    f"\033[1;33m[rollout] no delivery annotated for {stale:.0f}s "
                    f"\u2014 press 't' when you deliver one\033[0m")

    def _fmt(self, name, arr, dim):
        """Coerce to exactly ``dim``; backfill from the last good value if short."""
        arr = np.asarray(arr, dtype=np.float64).reshape(-1)
        if arr.size == dim:
            self._prev[name] = arr
            return arr
        out = np.zeros(dim)
        prev = self._prev.get(name)
        if prev is not None and prev.size == dim:
            out[:] = prev
        k = min(arr.size, dim)
        if k:
            out[:k] = arr[:k]
        self._prev[name] = out
        return out

    def _snapshot(self):
        """Latest feedback (own subscriptions) + camera messages (deque[-1])."""
        state = _arm_state(self._fb, self.puppet_keys)
        if state is None:
            return None
        qpos, qvel, effort, eef = state
        qpos = self._fmt("qpos", qpos, self.state_dim)
        qvel = self._fmt("qvel", qvel, self.state_dim)
        effort = self._fmt("effort", effort, self.state_dim)
        eef = self._fmt("eef", eef, self.eef_dim)
        arm = self._fb.get(self.primary_key)
        target = _to_sec(arm) if arm is not None else -1.0
        cam = {}
        for name in self.camera_names:
            entry = {}
            for (c, typ), queue in self._fb_cam.items():
                if c != name or not queue:
                    continue
                entry[typ] = _latest_before(queue, target)
            if entry:
                cam[name] = entry
        return {"qpos": qpos, "qvel": qvel, "effort": effort, "eef": eef,
                "arm_ts": target, "cam": cam}

    def _clone_images(self, images):
        if not self.args.trace_images or not images:
            return None
        return {k: np.array(v["rgb"]) for k, v in images.items() if "rgb" in v}

    # -------------------------------------------------------------- queue

    def _push(self, job):
        with self._jobs_cv:
            self._jobs.append(job)
            while len(self._jobs) > self.MAX_QUEUE:
                self._jobs.popleft()
                self._dropped += 1
            self._jobs_cv.notify()

    def _pop(self, timeout=0.5):
        with self._jobs_cv:
            if not self._jobs:
                self._jobs_cv.wait(timeout)
            if self._jobs:
                return self._jobs.popleft()
        return None

    # ------------------------------------------------------------- writer

    def _writer_loop(self):
        f = None
        tmp_path = None
        final_path = None
        pending_obs = []
        rows = []              # paired (obs, pred) request rows
        n = 0
        cur_epoch = None
        while not self._stop:
            job = self._pop()
            if job is None:
                continue
            kind, payload = job
            try:
                if kind == "start":
                    f, tmp_path, final_path = self._open_file(payload)
                    pending_obs, rows, n = [], [], 0
                    cur_epoch = payload["epoch"]
                elif f is None:
                    continue
                elif kind == "sample":
                    n = self._write_sample(f, payload, n)
                elif kind == "trace_obs":
                    if payload.get("epoch") == cur_epoch:
                        pending_obs.append(payload)
                elif kind == "trace_pred":
                    if payload.get("epoch") != cur_epoch:
                        continue        # late result from an earlier episode
                    obs = pending_obs.pop(0) if pending_obs else None
                    rows.append((obs, payload))
                elif kind == "finalize":
                    self._finalize(f, payload, rows, n)
                    f, tmp_path, final_path = None, None, None
                elif kind == "abort":
                    f.close()
                    if os.path.exists(tmp_path):
                        os.remove(tmp_path)
                    f, tmp_path, final_path = None, None, None
            except Exception as e:      # never die silently
                print(f"[rollout] writer error on {kind}: {e}")

    def _out_dir(self):
        return os.path.join(self.args.dataset_dir, self.args.task_name)

    def _stem(self, idx):
        return os.path.join(self._out_dir(), f"episode_{idx}")

    def _write_instruction_txt(self):
        path = os.path.join(self._out_dir(), "instruction.txt")
        try:
            if not os.path.exists(path):
                with open(path, "w") as fh:
                    fh.write(str(self.settings.get("task_instruction", "")))
        except OSError as e:
            print(f"[rollout] could not write instruction.txt: {e}")

    def _open_file(self, payload):
        os.makedirs(self._out_dir(), exist_ok=True)
        tmp_path = self._stem(payload["episode_idx"]) + ".hdf5.tmp"
        final_path = self._stem(payload["episode_idx"]) + ".hdf5"
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        f = h5py.File(tmp_path, "w", rdcc_nbytes=1024 ** 2 * 2)
        f.attrs["sim"] = False
        f.attrs["compress"] = True
        f.attrs["recording_complete"] = False
        for k, v in payload["settings"].items():
            f.attrs[k] = v
        obs = f.create_group("observations")
        image = obs.create_group("images")
        dt = h5py.string_dtype(encoding="utf-8")
        for cam in self.camera_names:
            image.create_dataset(f"cam_{cam}", (0,), maxshape=(None,), dtype=dt, chunks=True)
        if self.use_depth:
            depth = obs.create_group("images_depth")
            for cam in self.camera_names:
                depth.create_dataset(f"cam_{cam}", (0,), maxshape=(None,), dtype=dt, chunks=True)
        # well-formed empty main arrays even if no sample is recorded
        for name, dim in (("qpos", self.state_dim), ("qvel", self.state_dim),
                          ("effort", self.state_dim)):
            obs.create_dataset(name, (0, dim), maxshape=(None, dim), dtype="float32", chunks=True)
        obs.create_dataset("eef_pose", (0, self.eef_dim), maxshape=(None, self.eef_dim),
                           dtype="float32", chunks=True)
        f.create_dataset("action", (0, self.state_dim), maxshape=(None, self.state_dim),
                         dtype="float32", chunks=True)
        f.create_dataset("timestamps", (0,), maxshape=(None,), dtype="float64", chunks=True)
        f.create_group("alignment")
        f.create_group("inference")
        print(f"[rollout] writing episode {payload['episode_idx']} -> {final_path}")
        return f, tmp_path, final_path

    def _append_row(self, f, name, dim, arr, n, dtype="float32"):
        ds = f[name]
        ds.resize((n + 1, dim))
        ds[n] = np.asarray(arr, dtype=dtype)

    def _write_sample(self, f, s, n):
        # never start writing a sample unless every array is well-formed, so a
        # failure cannot leave ragged datasets at different lengths
        for arr, dim in ((s["qpos"], self.state_dim), (s["qvel"], self.state_dim),
                         (s["effort"], self.state_dim), (s["eef"], self.eef_dim),
                         (s["action"], self.state_dim)):
            if np.asarray(arr).reshape(-1).size != dim:
                print(f"[rollout] skip malformed sample {s['step']}: "
                      f"len={np.asarray(arr).size} != {dim}")
                return n
        self._append_row(f, "observations/qpos", self.state_dim, s["qpos"], n)
        self._append_row(f, "observations/qvel", self.state_dim, s["qvel"], n)
        self._append_row(f, "observations/effort", self.state_dim, s["effort"], n)
        self._append_row(f, "observations/eef_pose", self.eef_dim, s["eef"], n)
        self._append_row(f, "action", self.state_dim, s["action"], n)
        f["timestamps"].resize((n + 1,))
        f["timestamps"][n] = s["t_elapsed"]
        ds = f["alignment/arm_ts"] if "alignment/arm_ts" in f else \
            f["alignment"].create_dataset("arm_ts", (0,), maxshape=(None,), dtype="float64", chunks=True)
        ds.resize((n + 1,))
        ds[n] = s["arm_ts"]
        for cam, entry in s["frames"].items():
            msg = entry.get("rgb")
            if msg is not None:
                self._write_img(f, n, f"observations/images/cam_{cam}",
                                f"alignment/cam_{cam}_ts", cam, msg, ".jpg")
            if self.use_depth and "depth" in entry:
                self._write_img(f, n, f"observations/images_depth/cam_{cam}",
                                f"alignment/cam_{cam}_depth_ts", cam + ":depth",
                                entry["depth"], ".png")
        return n + 1

    def _write_img(self, f, n, dsname, tsname, key, msg, ext):
        if self._cam_msg.get(key) is not id(msg):
            img = self.operator.bridge.imgmsg_to_cv2(msg, "passthrough")
            if img.ndim == 3 and img.shape[-1] == 2:
                img = cv2.cvtColor(img, cv2.COLOR_YUV2RGB_YUY2)
            self._cam_encoded[key] = _encode_image(img, ext=ext)
            self._cam_msg[key] = id(msg)
            self._cam_stamp[key] = _to_sec(msg)
        ds = f[dsname] if dsname in f else \
            f[dsname.rsplit("/", 1)[0]].create_dataset(dsname.rsplit("/", 1)[1],
                                                       (0,), maxshape=(None,), chunks=True)
        ds.resize((n + 1,))
        ds[n] = self._cam_encoded[key]
        ts = f[tsname] if tsname in f else \
            f["alignment"].create_dataset(tsname.rsplit("/", 1)[1],
                                          (0,), maxshape=(None,), dtype="float64", chunks=True)
        ts.resize((n + 1,))
        ts[n] = self._cam_stamp.get(key, -1.0)

    def _finalize(self, f, payload, rows, n):
        kf = np.asarray(payload["key_frames"], dtype=np.int64)
        f.create_dataset("key_frame", data=kf)
        f.attrs["episode_end_time_seconds"] = float(payload["end_time"])
        f.attrs["num_samples"] = int(n)
        f.attrs["num_key_frames"] = int(kf.size)
        f.attrs["num_inference_requests"] = int(len(rows))
        if payload.get("reason") is not None:
            f.attrs["end_reason"] = str(payload["reason"])
        if payload.get("outcome") is not None:
            f.attrs["task_outcome"] = str(payload["outcome"])
        if n > 0 and payload["end_time"] > 0:
            f.attrs["recording_rate_hz"] = float(n / payload["end_time"])
        self._write_trace(f, rows, payload.get("walls", []))
        f.attrs["recording_complete"] = True
        f.flush()
        f.close()
        os.replace(self._stem(payload["episode_idx"]) + ".hdf5.tmp",
                   self._stem(payload["episode_idx"]) + ".hdf5")
        # repository convention: the fixed task instruction lives in instruction.txt
        # next to the episode files (collect_data.py does not store it in the HDF5,
        # and the OpenEAI_VLA converter reads it from there)
        self._write_instruction_txt()
        print(f"\033[1;32m[rollout] saved {self._stem(payload['episode_idx'])}.hdf5 "
              f"(T={n}, key_frames={kf.size}, requests={len(rows)})\033[0m")
        if self._dropped:
            print(f"[rollout] WARNING: {self._dropped} queued samples dropped (writer backlog)")
        self._finalize_done.set()

    def wait_finalize(self, timeout=180.0):
        """Block until the pending episode write has completed (used on shutdown)."""
        return self._finalize_done.wait(timeout)

    def _write_trace(self, f, rows, walls):
        if not rows:
            return
        g = f["inference"]
        preds = [p for _, p in rows]
        lmax = max(p["chunk"].shape[0] for p in preds)
        dim = preds[0]["chunk"].shape[1]
        chunks = np.zeros((len(preds), lmax, dim), dtype=np.float32)
        lengths = np.zeros(len(preds), dtype=np.int64)
        for i, p in enumerate(preds):
            L = p["chunk"].shape[0]
            chunks[i, :L] = p["chunk"]
            lengths[i] = L
        g.create_dataset("chunk_actions", data=chunks)
        g.create_dataset("chunk_lengths", data=lengths)
        g.create_dataset("request_global_t",
                         data=np.asarray([p["global_t"] for p in preds], dtype=np.int64))
        for key in ("latency_sec", "delay_steps", "first_exec_idx", "trigger_chunk_idx"):
            g.create_dataset(key, data=np.asarray([p["meta"].get(key, -1) for p in preds],
                                                  dtype="float64"))
        obs = [o for o, _ in rows]
        if any(o is not None for o in obs):
            g.create_dataset("obs_timestamp",
                             data=np.asarray([o["timestamp"] if o else np.nan for o in obs],
                                             dtype="float64"))
            # resolve the trajectory index at write time: the last sample taken at or
            # before the observation (0 if it predates the first sample, -1 if no samples)
            steps = []
            for o in obs:
                if not o:
                    steps.append(-1)
                    continue
                i = bisect.bisect_right(walls, o["wall"]) - 1
                steps.append(i if i >= 0 else (0 if walls else -1))
            g.create_dataset("obs_recording_step", data=np.asarray(steps, dtype=np.int64))
            g.create_dataset("obs_state",
                             data=np.asarray([o["state"] if o else np.zeros(self.state_dim)
                                              for o in obs], dtype="float32"))
            if self.args.trace_images and any(o and o.get("images") for o in obs):
                ig = g.create_group("obs_images")
                cams = sorted({c for o in obs if o and o.get("images") for c in o["images"]})
                dt = h5py.string_dtype(encoding="utf-8")
                for cam in cams:
                    arr = np.empty(len(obs), dtype=object)
                    for i, o in enumerate(obs):
                        if o and o.get("images") and cam in o["images"]:
                            arr[i] = base64.b64encode(cv2.imencode(".jpg", o["images"][cam])[1])
                        else:
                            arr[i] = b""
                    ig.create_dataset(cam, data=arr.astype(dt))


# --------------------------------------------------------- episode control


class EpisodeController:
    """Merges local keyboard + ZeroMQ (online task script) into one event stream."""

    def __init__(self, recorder, zmq_port):
        self.recorder = recorder
        self.node = recorder.operator.node
        self._latch = set()
        self.quit = False

        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()

        self._zmq_running = True
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.connect(f"tcp://localhost:{zmq_port}")
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._zthread = threading.Thread(target=self._zmq_loop, daemon=True)
        self._zthread.start()
        self.node.get_logger().info(
            "Controls: s=start/end recording, n=end+save, r=discard, t=annotate delivery"
        )

    def _on_press(self, key):
        try:
            self._latch.add(key.char.lower())
        except AttributeError:
            pass

    def _on_release(self, key):
        try:
            c = key.char.lower()
        except AttributeError:
            return
        was = c in self._latch
        self._latch.discard(c)
        if was:                    # exactly one event per physical press
            self.handle(c)

    def _zmq_loop(self):
        while self._zmq_running:
            try:
                if self._sock.poll(100):
                    self._zmq_message(self._sock.recv_string())
            except Exception:
                break

    def _zmq_message(self, message):
        if ":" in message:
            action, val = message.split(":", 1)
            action = action.lower()
            if action == "episode_end":
                self.recorder.end_episode(reason=val)
            elif action == "outcome":
                self.node.get_logger().info(f"[rollout] task outcome: {val}")
            elif action == "episode_start":
                self.handle("s")
            elif action in ("release", "press"):
                self.handle(val.lower())
            return
        self.handle(message.lower())

    def handle(self, key):
        r = self.recorder
        if key == "s":
            if r.recording:
                r.end_episode(reason="operator_toggle")
            else:
                r.start_episode()
        elif key == "n":
            if r.recording:
                r.end_episode(reason="operator_save")
            r.request_stop()          # reset + confirm + start the next episode
        elif key == "r":
            if r.recording:
                r.discard_episode()
            r.request_stop()
        elif key == "t":
            ok, info = r.annotate()
            if ok:
                self.node.get_logger().info(
                    f"\033[1;32m[rollout] delivery annotated at recording_step {info}\033[0m")
            else:
                self.node.get_logger().warning(f"[rollout] t ignored: {info}")
        elif key == "q":
            self.quit = True


# ------------------------------------------------------------------ main


def get_arguments():
    p = argparse.ArgumentParser(description="Autonomous asynchronous rollout collection")
    p.add_argument("--max_publish_step", type=int, default=100000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--frame_rate", type=int, default=50,
                   help="requested command-publish rate (NOT the recording rate)")
    p.add_argument("--image_shape", type=str, default="640x480x30")
    p.add_argument("--config_path", type=str, default="config_multi_inference.yml")
    p.add_argument("--task_instruction", type=str,
                   default="move the package to the conveyor belt on the right")
    p.add_argument("--pos_lookahead_step", type=int, default=2)
    p.add_argument("--pos_inference_step", type=int, default=24)
    p.add_argument("--chunk_size", type=int, default=50)
    p.add_argument("--server_host", type=str, default=None)
    p.add_argument("--server_port", type=int, default=8000)
    p.add_argument("--contiguous", action="store_true")
    p.add_argument("--model", type=str, default="openeai",
                   choices=["openeai", "pi", "act", "octo"])
    p.add_argument("-r", "--resume", action="store_true")
    p.add_argument("--smooth", action="store_true")
    p.add_argument("--merge", action="store_true")
    p.add_argument("--stand", action="store_true")
    p.add_argument("--use_tactile", action="store_true")
    # collection
    p.add_argument("-d", "--dataset_dir", type=str, default="./data")
    p.add_argument("-n", "--task_name", type=str, default="aloha_mobile_dummy")
    p.add_argument("-i", "--episode_idx", type=int, default=None)
    p.add_argument("--zmq_port", type=int, default=5604)
    p.add_argument("--record_rate", choices=["feedback", "fixed"], default="feedback",
                   help="feedback = one sample per new arm feedback (SFT 50Hz timebase)")
    p.add_argument("--record_hz", type=float, default=None, help="only for --record_rate fixed")
    p.add_argument("--trace_images", action="store_true",
                   help="also store the 224x224 observation images in /inference")
    p.add_argument("--no_auto_start", action="store_true",
                   help="do not start recording automatically after the start-up confirm")
    p.add_argument("--annot_timeout", type=float, default=30.0,
                   help="warn once if no delivery is annotated for this many seconds (0 disables)")
    p.add_argument("--dry_run", action="store_true",
                   help="run the scheduling+recording path without moving the robot")
    return p.parse_args()


def _restore_signal_handlers():
    """Undo rclpy's signal takeover for this process.

    ``rclpy.init()`` installs its own SIGINT *and* SIGTERM handlers. Consequences:
    Ctrl+C only shuts the rclpy context down instead of raising KeyboardInterrupt,
    so the process cannot be interrupted while blocked in ``input()``/``join()``;
    and ``kill``/SIGTERM is swallowed, so only SIGKILL stops it. Restore the usual
    behaviour so the operator keeps normal control over the process.
    """
    try:
        signal.signal(signal.SIGINT, signal.default_int_handler)   # -> KeyboardInterrupt
        signal.signal(signal.SIGTERM, signal.SIG_DFL)              # -> terminate
    except (ValueError, OSError):
        pass


def main():
    args = get_arguments()
    if args.record_rate == "fixed" and not args.record_hz:
        args.record_hz = float(args.frame_rate)
    if args.server_host is None:
        raise ValueError("--server_host is required")

    operator = inf.OpenEAIArmROS2Operator(args.config_path)
    _restore_signal_handlers()
    with open(args.config_path) as fh:
        config = yaml.safe_load(fh)

    settings = {
        "task_instruction": args.task_instruction,
        "frame_rate": int(args.frame_rate),
        "pos_lookahead_step": int(args.pos_lookahead_step),
        "pos_inference_step": int(args.pos_inference_step),
        "chunk_size": int(args.chunk_size),
        "merge": bool(args.merge),
        "contiguous": bool(args.contiguous),
        "smooth": bool(args.smooth),
        "model": str(args.model),
        "timebase": "feedback",
        "record_rate": args.record_rate,
        "zmq_port": int(args.zmq_port),
    }

    out_dir = os.path.join(args.dataset_dir, args.task_name)
    if args.episode_idx is None:
        args.episode_idx = len(glob.glob(os.path.join(out_dir, "*.hdf5")))
    recorder = RolloutRecorder(args, operator, config, settings, args.episode_idx)

    if args.dry_run:
        def _noop_publish(arm_name, target):
            recorder.note_command(target)
        def _noop_continuous(arm_name, target):
            pass
        operator.puppet_arm_publish = _noop_publish
        operator.puppet_arm_publish_continuous = _noop_continuous
        operator.node.get_logger().warning("[rollout] DRY RUN: no commands will be published")
    else:
        _orig_publish = operator.puppet_arm_publish
        def _recording_publish(arm_name, target):
            recorder.note_command(target)
            return _orig_publish(arm_name, target)
        operator.puppet_arm_publish = _recording_publish

    controller = EpisodeController(recorder, args.zmq_port)
    inf.initialize_client(args)
    print(f"[rollout] task={args.task_name} starting at episode_{args.episode_idx}; "
          f"press 's' to start recording")

    try:
        while inf.rclpy.ok():
            inf.model_inference(args, operator, recorder=recorder)
            # 'n'/'r' asked for a full cycle: model_inference re-runs the start-up
            # pose + confirmation, then the next episode is recorded.
            if not recorder.consume_cycle_request():
                break
    except KeyboardInterrupt:
        operator.node.get_logger().info("[rollout] interrupted")
    finally:
        pending = False
        if recorder.recording:
            recorder.end_episode(reason="interrupt")
            pending = True
        controller._zmq_running = False
        try:
            controller.listener.stop()
        except Exception:
            pass
        if pending:
            operator.node.get_logger().info(
                "[rollout] writing final episode before exit (Ctrl+C again / SIGKILL to skip)...")
            recorder.wait_finalize(180.0)
        recorder._stop = True
        time.sleep(0.1)
        # rclpy's executor thread can abort the C++ runtime during interpreter
        # finalization ("terminate called without an active exception", SIGABRT).
        # Everything is flushed and the episode is written, so exit directly.
        try:
            sys.stdout.flush()
            sys.stderr.flush()
        except Exception:
            pass
        os._exit(0)


if __name__ == "__main__":
    main()
