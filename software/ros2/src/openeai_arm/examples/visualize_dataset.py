import argparse
import base64
from typing import Union

import cv2
import h5py
import numpy as np


def decode_image(img: Union[np.bytes_, np.ndarray]) -> np.ndarray:
    """Decode an HDF5 image value into a BGR uint8 image for OpenCV."""
    if not isinstance(img, np.ndarray):
        buf = base64.b64decode(img)
        img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode image.")
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.ndim == 3 and img.shape[2] == 4:
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=str, help="path to .hdf5/.h5 file")
    parser.add_argument("field", type=str, help="dataset key in hdf5 (e.g. /images or images)")
    parser.add_argument("fps", type=float, nargs="?", default=20, help="output video fps (default: 20)")
    args = parser.parse_args()

    in_path = args.file
    key = args.field
    fps = float(args.fps)
    out_path = f"{in_path}.{key.strip('/').replace('/', '_')}.mp4"

    with h5py.File(in_path, "r") as f:
        if key not in f:
            key_with_slash = "/" + key.lstrip("/")
            if key_with_slash in f:
                key = key_with_slash
            else:
                raise KeyError(f"Key not found: {args.field}. Available keys: {list(f.keys())}")

        dataset = f[key]
        frame_count = dataset.shape[0]
        first = decode_image(dataset[0])
        height, width = first.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        try:
            for index in range(frame_count):
                frame = decode_image(dataset[index])
                if frame.shape[:2] != (height, width):
                    frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
                writer.write(frame)
                if (index + 1) % 200 == 0 or (index + 1) == frame_count:
                    print(f"\rWriting: {index + 1}/{frame_count}", end="")
        finally:
            writer.release()

    print(f"\nSaved video to: {out_path}")


if __name__ == "__main__":
    main()
