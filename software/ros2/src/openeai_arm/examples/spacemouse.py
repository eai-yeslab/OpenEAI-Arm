import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time

import pyspacemouse

class SpaceMouse:
    def __init__(self):
        if not pyspacemouse.open():
            raise RuntimeError("Could not open SpaceMouse device.")
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self.last_time = time.time()
        self.last_buttons = [0, 0]

    def update(self, on_button_down=None, on_button_up=None, on_button_pressed=None, on_button_released=None):
        state = pyspacemouse.read()
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if state is not None:
            speed_factor = 0.2   # Can be adjusted as needed
            angular_factor = -0.6 # Can be adjusted as needed
            vx, vy, vz = state.x * speed_factor, state.y * speed_factor, state.z * speed_factor
            vroll, vpitch, vyaw = state.roll * angular_factor, state.pitch * angular_factor, state.yaw * angular_factor
            self.position[0] = vx * dt
            self.position[1] = vy * dt
            self.position[2] = vz * dt
            self.orientation[0] = vroll * dt
            self.orientation[1] = vpitch * dt
            self.orientation[2] = vyaw * dt
            
            # Handle buttons. Assume state.buttons integer's low bits represent buttons. If not an integer, change to list similarly
            if hasattr(state, "buttons"):
                if isinstance(state.buttons, int):
                    # Each bit represents a button state
                    curr_buttons = [(state.buttons >> i) & 1 for i in range(2)]
                else:
                    # Assume it's a list
                    curr_buttons = list(state.buttons)
            for idx, (prev, curr) in enumerate(zip(self.last_buttons, curr_buttons)):
                if curr == 1:
                    if on_button_down is not None:
                        on_button_down(idx)
                if curr == 0:
                    if on_button_up is not None:
                        on_button_up(idx)

                if curr == 1 and prev == 0:  # Rising edge trigger
                    if on_button_pressed is not None:
                        on_button_pressed(idx)
                        
                if curr == 0 and prev == 1:  # Falling edge trigger
                    if on_button_released is not None:
                        on_button_released(idx)
            self.last_buttons = curr_buttons
        return tuple(self.position), tuple(self.orientation)

    def close(self):
        pyspacemouse.close()


class SpaceMousePublisher(Node):
    def __init__(self, dof=6):
        super().__init__('spacemouse_joint_publisher')
        self.dof = dof  # Number of joints, customizable
        self.sm = SpaceMouse()
        self.joint_target_publisher = self.create_publisher(JointState, 'joint_targets', 10)
        self.reset_mode_publisher = self.create_publisher(Int32, 'robot_reset', 10)
        self.timer = self.create_timer(0.01, self.publish_joint_state)  # 10Hz
        self.gripper_open = 0
        self.reset_mode = False
        
    def on_button_pressed(self, idx):
        if idx == 0:
            self.gripper_open = 0.08 - self.gripper_open
        elif idx == 1:
            self.reset_mode = True
            
    def on_button_released(self, idx):
        if idx == 1:
            self.reset_mode = False
        
    

    def publish_joint_state(self):
        pos, ori = self.sm.update(on_button_released=self.on_button_released, on_button_pressed=self.on_button_pressed)  # Get absolute coordinates
        # Concatenate action: if dof+1 is greater than 6, pad with zeros, e.g., (7 joints then 7, 6 joints then 6+1=7)
        action = list(pos) + list(ori) + [self.gripper_open]
        needed_length = 7  # Target array length
        if len(action) < needed_length:
            action.extend([0.0] * (needed_length - len(action)))
        else:
            action = action[:needed_length]  # 超长就截断

        msg = JointState()
        now = self.get_clock().now()
        msg.header.stamp.sec = now.seconds_nanoseconds()[0]
        msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]
        msg.name = [f'joint_{i+1}' for i in range(needed_length)]
        msg.position = [float(act) for act in action]
        msg.velocity = [0.0] * needed_length
        msg.effort = [0.0] * needed_length
        self.joint_target_publisher.publish(msg)
        self.get_logger().debug(f"Published joint targets: {msg.position}")
        
        msg = Int32()
        msg.data = int(self.reset_mode)
        self.reset_mode_publisher.publish(msg)

    def destroy_node(self):
        self.sm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMousePublisher(dof=6)  # Modify dof to match your robotic arm's degrees of freedom
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()