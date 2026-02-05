import h5py
import cv2
import numpy as np
import sys
import tqdm

# ---- Modify this part ----
HDF5_PATH = 'ros2/src/openeai_arm/examples/data/fold_shirt/episode_5.hdf5'      # your HDF5 file path
DATASET_KEY = '/observations/images/cam_agent'         # dataset name inside HDF5 file
MP4_OUTPUT = 'output.mp4'
FPS = 30
# --------------------------

# 1. Open HDF5 and load dataset shape
with h5py.File(HDF5_PATH, 'r') as f:
    data = f[DATASET_KEY]
    frames, h, w = data.shape[:3]
    if data.ndim == 4:
        channels = data.shape[3]
    elif data.ndim == 3:
        channels = 1
    else:
        print("Unsupported dataset shape:", data.shape)
        sys.exit(1)

    # 2. Prepare video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # mp4v is widely supported
    out = cv2.VideoWriter(MP4_OUTPUT, fourcc, FPS, (w, h))

    # 3. Iterate and write frame by frame
    for i in tqdm.tqdm(range(frames), desc="Written Frame"):
        frame = data[i]
        # Convert to uint8 if needed
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255)
            frame = frame.astype(np.uint8)

        # Handle grayscale to 3-channel BGR for mp4
        if channels == 1 or frame.ndim == 2:
            frame = np.squeeze(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif channels == 3:
            # OpenCV expects BGR, most datasets store as RGB. Convert if necessary
            if (frame.shape[2] == 3):
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            print(f"Cannot handle frame with {channels} channels.")
            sys.exit(1)

        out.write(frame)
        # if i % 50 == 0:
        #     print(f"Written frame {i+1}/{frames}")

    out.release()
    print(f"Saved video to {MP4_OUTPUT}")