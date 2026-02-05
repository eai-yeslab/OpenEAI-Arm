import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.pub = self.create_publisher(JointState, 'joint_targets', 10)
        self.dt = 1.0 / 50
        self.num_joints = 7

    def send_joint(self, pos):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i+1}" for i in range(self.num_joints)]
        msg.position = [float(x) for x in pos]
        self.pub.publish(msg)

    def move_interpolated(self, start, target, duration=1.0):
        steps = int(duration / self.dt)
        for s in range(1, steps+1):
            ratio = s / steps
            pos = (1 - ratio) * np.array(start) + ratio * np.array(target)
            self.send_joint(pos)
            time.sleep(self.dt)

    def run(self):
        target = np.array([1.0, 0.5, 1.5, -0.5, 0.67, 0.44, 0.04])
        amplitude = 0.2
        freq_hz = 0.25
        test_time = 8.0
        send_freq = 50.0
        dt = 1.0 / send_freq

        time.sleep(2)
        print("Interpolated move to target over 1s ...")
        zeros = np.zeros(7)
        self.move_interpolated(zeros, target, duration=1.0)

        for _ in range(int(0.5/dt)):
            self.send_joint(target)
            time.sleep(dt)

        print("Start cyclic control ...")
        t0 = time.time()
        cycle_steps = int(test_time * send_freq)
        for k in range(cycle_steps):
            t = time.time() - t0
            cycle = []
            for j in range(self.num_joints - 1):
                pos = target[j] + amplitude * np.sin(2 * np.pi * freq_hz * t)
                cycle.append(pos)
            cycle.append(target[-1] + 0.04 * np.sin(2 * np.pi * freq_hz * t))
            self.send_joint(cycle)
            time.sleep(dt)

        print("Done, holding final pose 1s ...")
        for _ in range(int(1/dt)):
            self.send_joint(cycle)
            time.sleep(dt)
        
        self.move_interpolated(target, zeros, duration=1.0)

def main():
    rclpy.init()
    node = ArmController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()