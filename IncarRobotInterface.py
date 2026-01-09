from incar.messages.primitives_pb2 import Pose, Quaternion, Vector3, Velocity
from incar.messages.robot_command_pb2 import RobotCommand
from incar.messages.robot_state_pb2 import CartesianState, JointState, RobotModuleState, RobotState
from incar.messages.sensor_data_pb2 import SensorData
from incar.webrtc.webrtc_connection import WebRTCConnection

import asyncio
import socket
import time
from typing import Callable, Dict, List

ROBOT_COMMAND_CHANNEL = ""
ROBOT_STATE_CHANNEL = ""
DEFAULT_COMMAND_TIMEOUT = 0.2

class IncarRobotInterface():
    command_hooks: Dict[str, Callable[[List[float]]]]
    routine_hooks: List[Callable]
    routine_key: str
    command_timeout: float = DEFAULT_COMMAND_TIMEOUT
    control_dt: float

    _commands: Dict[str, List[float]] = { }
    _state_message: RobotState = RobotState()
    _last_command_timestep: Dict[str, float]

    def __init__(
        self,
        control_dt,
        command_hooks: Dict[str, Callable[[List[float]]]] = {},
        routine_hooks: List[Callable] = [],
        routine_key: str = "",
        sensor_names: List[str] = []
    ):
        self._rtc = (WebRTCConnection
            .add_channel(ROBOT_COMMAND_CHANNEL, lambda msg: self.handle_command_message(msg))
            .add_channel(ROBOT_STATE_CHANNEL)
        )
        for sensor in sensor_names:
            self._rtc.add_channel(sensor)

        self.control_dt = control_dt
        self.command_hooks = command_hooks
        self.routine_hooks = routine_hooks
        self.routine_key = routine_key

    def start(self, ip = None, port = 9999):
        if ip is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()

        future = asyncio.wait(
            [
                self._rtc.start_connection(ip, port, True),
                self.control_loop()
            ],
            return_when = asyncio.FIRST_EXCEPTION
        )

        try:
            done, _pending = asyncio.get_event_loop().run_until_complete(future)
            for task in done:
                task.result() # Raises exceptions if any
            for task in _pending:
                task.cancel()
        except Exception:
            for task in _pending:
                task.cancel()
            raise

    def handle_command_message(self, message: RobotCommand):
        message_obj = RobotCommand()
        message_obj.ParseFromString(message)

        # Buffer commands for control loop
        for key in message_obj.commands.keys():
            if key in self.command_hooks.keys():
                self._commands[key](list(message_obj.commands[key].values))

        # Call routines if any
        if message_obj.routines[self.routine_key] > 0:
            if message_obj.routines[self.routine_key] < len(self.routine_hooks):
                self.routine_hooks[message_obj.routines[self.routine_key]]()

    async def control_loop(self):
        print("Waiting to receive first command")
        while self._commands == { }:
            await asyncio.sleep(1)
        print("First command received, starting control loop")

        while self._rtc.get_peer().connectionState == "connected":
            for key, value in self._commands.items():
                if self._last_command_timestep[key] - time.time() > self.command_timeout:
                    print(f"[Warning] Command {key} is outdated, skipping")
                    continue
                self.command_hooks[key](value)

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
        module = RobotModuleState()

        if ee_pose is not None or ee_vel is not None:
            ee=CartesianState()
            if ee_pose is not None:
                pose = Pose(
                    position=Vector3(
                        x=ee_pose[0],
                        y=ee_pose[1],
                        z=ee_pose[2]
                    ),
                    rotation=Quaternion(
                        x=ee_pose[3],
                        y=ee_pose[4],
                        z=ee_pose[5],
                        w=ee_pose[6]
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
            module.CopyFrom(ee)

        if joint_pos is not None or joint_vel is not None or joint_effort is not None:
            joints = JointState()
            if joint_pos is not None:
                joints.positions = joint_pos
            if joint_vel is not None:
                joints.velocities = joint_vel
            if joint_effort is not None:
                joints.efforts = joint_effort
            module.joints.CopyFrom(joints)

        self._state_message.moduleStates[module_name].CopyFrom(module)

    def publish_state(self):
        serialized_msg = self._state_message.SerializeToString()
        self._rtc.send_channel(ROBOT_STATE_CHANNEL, serialized_msg)
        self._state_message = RobotState()

    def publish_sensor(self, sensor_name: str, sensor_data: List[float]):
        message = SensorData(data=sensor_data)
        serialized_msg = message.SerializeToString()
        self._rtc.send_channel(sensor_name, serialized_msg)