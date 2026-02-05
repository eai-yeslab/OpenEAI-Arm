import time
import threading
from threading import Event, Lock, Thread
from typing import Protocol, Sequence
from typing import Dict, Any
import numpy as np

# Dynamixel SDK
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

# ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

# Constants
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


class DynamixelDriverProtocol(Protocol):
    def set_joints(self, joint_angles: Sequence[float]):
        """Set the joint angles for the Dynamixel servos.

        Args:
            joint_angles (Sequence[float]): A list of joint angles.
        """
        ...

    def torque_enabled(self) -> bool:
        """Check if torque is enabled for the Dynamixel servos.

        Returns:
            bool: True if torque is enabled, False if it is disabled.
        """
        ...

    def set_torque_mode(self, enable: bool):
        """Set the torque mode for the Dynamixel servos.

        Args:
            enable (bool): True to enable torque, False to disable.
        """
        ...

    def get_joints(self) -> np.ndarray:
        """Get the current joint angles in radians.

        Returns:
            np.ndarray: An array of joint angles.
        """
        ...

    def close(self):
        """Close the driver."""


class FakeDynamixelDriver(DynamixelDriverProtocol):
    def __init__(self, ids: Sequence[int]):
        self._ids = ids
        self._joint_angles = np.zeros(len(ids), dtype=int)
        self._torque_enabled = False

    def set_joints(self, joint_angles: Sequence[float]):
        if len(joint_angles) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")
        self._joint_angles = np.array(joint_angles)

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self._torque_enabled = enable

    def get_joints(self) -> np.ndarray:
        return self._joint_angles.copy()

    def close(self):
        pass


class DynamixelDriver(DynamixelDriverProtocol):
    def __init__(
        self, ids: Sequence[int], port: str = "/dev/ttyACM0", baudrate: int = 57600
    ):
        """Initialize the DynamixelDriver class.

        Args:
            ids (Sequence[int]): A list of IDs for the Dynamixel servos.
            port (str): The USB port to connect to the arm.
            baudrate (int): The baudrate for communication.
        """
        self._ids = ids
        self._joint_angles = None
        self._lock = Lock()

        # Initialize the port handler, packet handler, and group sync read/write
        self._portHandler = PortHandler(port)
        self._packetHandler = PacketHandler(2.0)
        self._groupSyncRead = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._groupSyncWrite = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # Open the port and set the baudrate
        if not self._portHandler.openPort():
            raise RuntimeError("Failed to open the port")

        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to change the baudrate, {baudrate}")

        # Add parameters for each Dynamixel servo to the group sync read
        for dxl_id in self._ids:
            if not self._groupSyncRead.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )

        # Disable torque for each Dynamixel servo
        # self._torque_enabled = True
        # try:
        #     self.set_torque_mode(self._torque_enabled)
        # except Exception as e:
        #     print(f"port: {port}, {e}")

        self._stop_thread = Event()
        self._start_reading_thread()

    def set_joints(self, joint_angles: Sequence[float]):
        if len(joint_angles) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")

        for dxl_id, angle in zip(self._ids, joint_angles):
            # Convert the angle to the appropriate value for the servo
            position_value = int(angle * 2048 / np.pi)

            # Allocate goal position value into byte array
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(position_value)),
                DXL_HIBYTE(DXL_LOWORD(position_value)),
                DXL_LOBYTE(DXL_HIWORD(position_value)),
                DXL_HIBYTE(DXL_HIWORD(position_value)),
            ]

            # Add goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self._groupSyncWrite.addParam(
                dxl_id, param_goal_position
            )
            if not dxl_addparam_result:
                raise RuntimeError(
                    f"Failed to set joint angle for Dynamixel with ID {dxl_id}"
                )

        # Syncwrite goal position
        dxl_comm_result = self._groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError("Failed to syncwrite goal position")

        # Clear syncwrite parameter storage
        self._groupSyncWrite.clearParam()

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            dxl_id = self._ids[-1]
            # for dxl_id in self._ids:
            dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                # print(dxl_comm_result)
                # print(dxl_error)
                raise RuntimeError(
                    f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
                )
            
            # if enable:    
                
            #     joint_angle = np.deg2rad(45)
            #     # Convert the angle to the appropriate value for the servo
            #     position_value = int(joint_angle * 2048 / np.pi)

            #     # Allocate goal position value into byte array
            #     param_goal_position = [
            #         DXL_LOBYTE(DXL_LOWORD(position_value)),
            #         DXL_HIBYTE(DXL_LOWORD(position_value)),
            #         DXL_LOBYTE(DXL_HIWORD(position_value)),
            #         DXL_HIBYTE(DXL_HIWORD(position_value)),
            #     ]
                
            #     dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
            #         self._portHandler, dxl_id, ADDR_GOAL_POSITION, param_goal_position
            #     )
            #     if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            #         # print(dxl_comm_result)
            #         print(dxl_error)
            #         raise RuntimeError(
            #             f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
            #         )

        self._torque_enabled = enable

    def _start_reading_thread(self):
        self._reading_thread = Thread(target=self._read_joint_angles)
        self._reading_thread.daemon = True
        self._reading_thread.start()

    def _read_joint_angles(self):
        # Continuously read joint angles and update the joint_angles array
        while not self._stop_thread.is_set():
            time.sleep(0.001)
            with self._lock:
                _joint_angles = np.zeros(len(self._ids), dtype=int)
                dxl_comm_result = self._groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"warning, comm failed: {dxl_comm_result}")
                    continue
                for i, dxl_id in enumerate(self._ids):
                    if self._groupSyncRead.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        angle = self._groupSyncRead.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        angle = np.int32(np.uint32(angle))
                        _joint_angles[i] = angle
                    else:
                        raise RuntimeError(
                            f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                        )
                self._joint_angles = _joint_angles
            # self._groupSyncRead.clearParam() # TODO what does this do? should i add it

    def get_joints(self) -> np.ndarray:
        # Return a copy of the joint_angles array to avoid race conditions
        while self._joint_angles is None:
            time.sleep(0.1)
        _j = self._joint_angles.copy()
        return _j / 2048.0 * np.pi

    def close(self):
        self._stop_thread.set()
        self._reading_thread.join()
        self._portHandler.closePort()



class DynamixelNode():
    def __init__(self):
        self.ids = range(1, 9)
        self.directions = [1, -1, -1, -1, -1, 1, 1, 1]
        self.degree = False
        self.dof = 6
        self.offset = np.zeros(self.dof)
        self.gripper_max_degree = 40
        self.action = np.zeros(self.dof + 1)
        self.velocity = np.zeros(self.dof + 1)
        
        self._init_ros()
        self.driver = self._init_driver()
        self._initial_angles = None
        self._prev_angles = None
        
        self.setup()
        
    def _init_driver(self):
        return DynamixelDriver(ids=self.ids, port=self.port)
    
    def _init_ros(self):
        self.node = rclpy.create_node("Gello")
        self.logger_ = self.node.get_logger()
        self.port = self.node.declare_parameter('port', '/dev/ttyACM2').get_parameter_value().string_value
        
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.joint_target_publisher = self.node.create_publisher(JointState, '/joint_targets', 10)
        self.executor_thread = threading.Thread(target=self.run_executor, daemon=True)
        self.executor_thread.start()
        
        
    def run_executor(self):
        self.logger_.info("Running executor...")
        self.executor.spin()

    def setup(self):
        self._initial_angles = self.driver.get_joints()
        self.logger_.info(f"Initial joint angles: {self._initial_angles}")

    def update(self):
        current_angles = self.driver.get_joints()
    
        delta_angles = []
        for i in range(len(self.ids)):
            delta = self._angle_diff(current_angles[i], self._initial_angles[i]) * self.directions[i]
            delta_angles.append(delta)
        
        k = 0.5
        action = np.zeros(self.dof + 1, dtype=np.float32)
        raw_action = np.array(delta_angles[:self.dof])
        target_velocity = raw_action - self.action[:self.dof]
        self.velocity[:self.dof] = k * self.velocity[:self.dof] + (1-k) * target_velocity
        action[:self.dof] = self.action[:self.dof] + self.velocity[:self.dof]
        if self.degree:
            action[:self.dof] = action[:self.dof] * 180 / np.pi
        action[-1] = np.abs(np.rad2deg(delta_angles[-1])) / self.gripper_max_degree
        action[-1] = np.clip(action[-1], 0, 1)
        if self.directions[-1] < 0:
            action[-1] = 1 - action[-1]
        self.action = action.copy()
            
        action[-1] = 0.085 * action[-1] - 0.005
        action[:self.dof] += self.offset

        return action
            
            
    def on_execute(self):
        action = self.update()
        if action is not None:        
            msg = JointState()
        
            now = self.node.get_clock().now()
            msg.header.stamp.sec = now.seconds_nanoseconds()[0]
            msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]
        
            msg.name = [f'joint_{i+1}' for i in range(len(action[:(self.dof + 1)]))]
            msg.position = [float(act) for act in action[:(self.dof + 1)]]
            msg.velocity = [0.0] * (self.dof + 1)
            msg.effort = [0.0] * (self.dof + 1)        

            self.joint_target_publisher.publish(msg)
        

    def _angle_diff(self, current: float, prev: float) -> float:
        diff = current - prev
        if diff > np.pi:
            diff -= 2 * np.pi
        elif diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
def main():
    rclpy.init()
    gello_node = DynamixelNode()
    frequency = 200
    while True:
        tick = time.time()
        gello_node.on_execute()
        tock = time.time()
        t_interval = 1 / frequency - (tock - tick)
        if t_interval > 0:
            time.sleep(t_interval)
        else:
            print(f"Warning: fail to execute under {frequency} Hz.")
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()