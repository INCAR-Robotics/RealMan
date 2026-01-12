import traceback
from incar.messages.primitives_pb2 import Pose, Quaternion, Vector3, Velocity
from incar.messages.robot_command_pb2 import RobotCommand
from incar.messages.robot_state_pb2 import CartesianState, JointState, RobotModuleState, RobotState
from incar.messages.sensor_data_pb2 import SensorData
from incar.webrtc.webrtc_connection import WebRTCConnection

import asyncio
import numpy as np
import socket
import time
from typing import Callable, Dict, List

ROBOT_COMMAND_CHANNEL = "robot_command"
ROBOT_STATE_CHANNEL = "robot_state"
DEFAULT_COMMAND_TIMEOUT = 0.2

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class IncarRobotInterface():
    command_hooks: Dict[str, Callable[[List[float]], None]]
    loop_callbacks: List[Callable[["IncarRobotInterface"], None]] = []
    routine_hooks: List[Callable]
    routine_key: str
    command_timeout: float = DEFAULT_COMMAND_TIMEOUT
    control_dt: float

    _commands: Dict[str, List[float]] = { }
    _state_message: RobotState = RobotState()
    _last_command_timestep: Dict[str, float] = { }

    def __init__(
        self,
        control_dt,
        command_hooks: Dict[str, Callable[[List[float]], None]] = {},
        routine_hooks: List[Callable] = [],
        routine_key: str = "",
        loop_callbacks: List[Callable[["IncarRobotInterface"], None]] = {},
        sensor_names: List[str] = []
    ):
        self._rtc = (WebRTCConnection()
            .add_channel(ROBOT_COMMAND_CHANNEL, lambda msg: self.handle_command_message(ROBOT_COMMAND_CHANNEL, msg))
            .add_channel(ROBOT_STATE_CHANNEL)
        )
        for sensor in sensor_names:
            self._rtc.add_channel(sensor)

        self.control_dt = control_dt
        self.command_hooks = command_hooks
        self.routine_hooks = routine_hooks
        self.routine_key = routine_key
        self.loop_callbacks = loop_callbacks

    def start(self, ip = None, port = 9999):
        if ip is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()

        future = asyncio.wait(
            [
                asyncio.ensure_future(self._rtc.start_connection(ip, port, True)),
                asyncio.ensure_future(self.control_loop())
            ],
            return_when=asyncio.FIRST_EXCEPTION
        )

        try:
            print("In start")
            done, _pending = asyncio.get_event_loop().run_until_complete(future)
            # done, _pending = await future
            for task in done:
                print("task done")
                task.result() # Raises exceptions if any
            for task in _pending:
                print("task pending")
                task.cancel()
        except Exception:
            print("EXCEOTIUN")
            for task in _pending:
                task.cancel()
            raise

    def handle_command_message(self, channel, message: RobotCommand):
        message_obj = RobotCommand()
        message_obj.ParseFromString(message)
        # Buffer commands for control loop
        try:
            for key in message_obj.commands.keys():
                if key in self.command_hooks.keys():
                    self._commands[key] = list(message_obj.commands[key].values)
                    self._last_command_timestep[key] = time.time()

            # Call routines if any
            if message_obj.routines[self.routine_key] > 0:
                if message_obj.routines[self.routine_key] < len(self.routine_hooks):
                    self.routine_hooks[message_obj.routines[self.routine_key]]()
        except Exception as e:
            traceback.print_exc()

    async def control_loop(self):
        print("Waiting to receive first command")
        while self._commands == { }:
            await asyncio.sleep(1)
        print("First command received, starting control loop")

        while self._rtc.get_peer().connectionState == "connected":
            try:
                for key, value in self._commands.items():
                    if self._last_command_timestep[key] - time.time() > self.command_timeout:
                        print(f"[Warning] Command {key} is outdated, skipping")
                        continue
                    self.command_hooks[key](value)

                for callback in self.loop_callbacks:
                    callback(self)
            except Exception as e:
                traceback.print_exc()
            await asyncio.sleep(self.control_dt)

    def set_robot_state(
        self,
        module_name: str,
        ee_pose: List[float] = None,
        ee_vel: List[float] = None,
        joint_pos: List[float] = None,
        joint_vel: List[float] = None,
        joint_effort: List[float] = None
    ):
        module = self._state_message.moduleStates.get_or_create(module_name)

        if ee_pose is not None or ee_vel is not None:
            ee=CartesianState()
            if ee_pose is not None:
                if len(ee_pose) == 7:
                    rot = ee_pose[3:]
                if len(ee_pose) == 6:
                    rot = get_quaternion_from_euler(ee_pose[3], ee_pose[4], ee_pose[5])
                pose = Pose(
                    position=Vector3(
                        x=ee_pose[0],
                        y=ee_pose[1],
                        z=ee_pose[2]
                    ),
                    rotation=Quaternion(
                        x=rot[0],
                        y=rot[1],
                        z=rot[2],
                        w=rot[3]
                    )
                )
                ee.pose.CopyFrom(pose)
            if ee_vel is not None:
                velocity=Velocity(
                    linear=Vector3(
                        x=ee_vel[0],
                        y=ee_vel[1],
                        z=ee_vel[2]
                    ),
                    angular=Vector3(
                        x=ee_vel[3],
                        y=ee_vel[4],
                        z=ee_vel[5],
                    )
                )
                ee.velocity.CopyFrom(velocity)
            module.ee.CopyFrom(ee)

        if joint_pos is not None or joint_vel is not None or joint_effort is not None:
            joints = JointState()
            if joint_pos is not None:
                joints.positions.extend(joint_pos)
            if joint_vel is not None:
                joints.velocities.extend(joint_vel)
            if joint_effort is not None:
                joints.efforts.extend(joint_effort)
            module.joints.CopyFrom(joints)

    def publish_state(self):
        serialized_msg = self._state_message.SerializeToString()
        self._rtc.send_channel(ROBOT_STATE_CHANNEL, serialized_msg)
        self._state_message = RobotState()

    def publish_sensor(self, sensor_name: str, sensor_data: List[float]):
        message = SensorData(data=sensor_data)
        serialized_msg = message.SerializeToString()
        self._rtc.send_channel(sensor_name, serialized_msg)
