import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, Imu
from rclpy.qos import qos_profile_sensor_data
import threading, time, queue
import h5py
import numpy as np
import cv2
from cv_bridge import CvBridge

class DataSaver(Node):
    def __init__(self, h5_path, max_frames=0):
        super().__init__('data_saver')
        self.color_queue  = queue.Queue()
        self.accel_queue  = queue.Queue()
        self.gyro_queue   = queue.Queue()
        self.is_running   = True
        self.max_frames = max_frames
        self.n_color = 0

        self.color_sub = self.create_subscription(
            Image, '/camera/eyeview/color/image_raw', self.color_callback, 10)
        self.accel_sub = self.create_subscription(
            Imu,   '/camera/eyeview/accel/sample',   self.accel_callback,   qos_profile_sensor_data)
        self.gyro_sub = self.create_subscription(
            Imu,   '/camera/eyeview/gyro/sample',    self.gyro_callback,    qos_profile_sensor_data)
        self.writer_thread = threading.Thread(target=self.hdf5_writer, args=(h5_path,))
        self.writer_thread.start()

    def color_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.color_queue.put(('color', (ts, msg)))
        self.n_color += 1
        if self.max_frames and self.n_color > self.max_frames:
            self.is_running = False

    def accel_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.accel_queue.put((ts, msg))

    def gyro_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gyro_queue.put((ts, msg))

    def hdf5_writer(self, out_path):
        bridge = CvBridge()
        h5 = h5py.File(out_path, "w")
        color_list = []
        color_ts_list = []
        accel_list = []
        accel_ts_list = []
        gyro_list = []
        gyro_ts_list = []
        while self.is_running or \
              not self.color_queue.empty() or \
              not self.accel_queue.empty() or \
              not self.gyro_queue.empty():
            while not self.color_queue.empty():
                ts, msg = self.color_queue.get_nowait()[1]
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                color_list.append(img)
                color_ts_list.append(ts)
            while not self.accel_queue.empty():
                ts, msg = self.accel_queue.get_nowait()
                accel_list.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
                accel_ts_list.append(ts)
            while not self.gyro_queue.empty():
                ts, msg = self.gyro_queue.get_nowait()
                gyro_list.append([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
                gyro_ts_list.append(ts)
            time.sleep(0.01)
        color_arr = np.stack(color_list) if color_list else np.zeros([0,480,640,3],np.uint8)
        h5.create_dataset("color", data=color_arr, compression="gzip")
        h5.create_dataset("color_ts", data=np.array(color_ts_list), compression="gzip")
        h5.create_dataset("accel", data=np.array(accel_list), compression="gzip")
        h5.create_dataset("accel_ts", data=np.array(accel_ts_list), compression="gzip")
        h5.create_dataset("gyro", data=np.array(gyro_list), compression="gzip")
        h5.create_dataset("gyro_ts", data=np.array(gyro_ts_list), compression="gzip")
        h5.close()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='ros2_full_data_parallel.h5')
    parser.add_argument('--max_frames', type=int, default=0)
    args = parser.parse_args()
    rclpy.init()
    node = DataSaver(args.output, max_frames=args.max_frames)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Received Keyboard Interrupt, shutting down...")
        node.is_running = False
        node.writer_thread.join()
        print("Data collection finished, file has been written.")

if __name__=='__main__':
    main()