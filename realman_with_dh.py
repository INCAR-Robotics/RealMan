import socket
import json
import threading
import time
import traceback
from typing import List
from Robotic_Arm.rm_robot_interface import *
from incar_networking.robot_interface import IncarRobotInterface

HOME_JOINT_POSITIONS = [0.0, -6.4, 0.0, 72.5, 0.0, 80.7, -98.8]
ROUTINE_IS_RUNNING = False

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

GRIPPER_PORT = 1        # 1 = end-effector interface board RS485 (tool flange)
GRIPPER_SLAVE_ID = 1    # DH default Modbus slave address
GRIPPER_BAUD = 115200   # DH AG-95 default
GRIPPER_FORCE_PERCENT = 100  # 20-100%, maps to ~45-160N per jaw

# Gripper commands
REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_INIT_STATE = 0x0200
REG_STATE = 0x0201
REG_CUR_POS = 0x0202

def _gripper_write(robot, addr, value):
    p = rm_peripheral_read_write_params_t(port=GRIPPER_PORT, address=addr, device=GRIPPER_SLAVE_ID, num=1)
    ret = robot.rm_write_single_register(p, value)
    if ret != 0:
        print(f"  WARNING: write 0x{addr:04X}={value} failed, ret={ret}")
    return ret


def _gripper_read(robot, addr):
    p = rm_peripheral_read_write_params_t(port=GRIPPER_PORT, address=addr, device=GRIPPER_SLAVE_ID, num=1)
    ret, val = robot.rm_read_holding_registers(p)
    if ret != 0:
        print(f"  WARNING: read 0x{addr:04X} failed, ret={ret}")
    return val

class RealManRobotWithDH:
    def __init__(self, dt_ms: int, module_name="arm"):
        self.module_name = module_name

        # -----------------------------
        # 1. Create robot connection
        # -----------------------------
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT)

        if not self.handle:
            raise RuntimeError("Failed to create robot arm handle")

        ret = self.robot.rm_set_tool_voltage(3)  # 0=0V, 1=5V, 2=12V, 3=24V
        ret = self.robot.rm_set_modbus_mode(GRIPPER_PORT, GRIPPER_BAUD, 1)
        time.sleep(1)
        _gripper_write(self.robot, REG_INIT, 0x01)

        print("Robot connected.")

        # Gripper Modbus RTU I/O must stay off the 10ms control loop: rm_movev_canfd with
        # follow=True requires a streaming period <=10ms, and a Modbus round-trip over RS485
        # easily blows that budget, silently killing cartesian velocity streaming. Run it on
        # its own thread instead and hand the main loop cached/queued values.
        self._gripper_lock = threading.Lock()
        self._gripper_target = None
        self._gripper_pos = 0
        self._gripper_running = True
        self._gripper_thread = threading.Thread(target=self._gripper_loop, daemon=True)
        self._gripper_thread.start()

        # -----------------------------
        # 2. Motion configuration
        # -----------------------------
        self.robot.rm_set_movev_canfd_init(
            avoid_singularity_flag=1,
            frame_type=1,
            dt=dt_ms
        )

        self.robot.rm_set_arm_max_line_speed(0.25)
        self.robot.rm_set_arm_max_line_acc(3)
        self.robot.rm_set_arm_max_angular_speed(3.14)
        self.robot.rm_set_arm_max_angular_acc(3.14 * 3)

        self.mod = rm_movev_canfd_mode_t()
        self.mod.follow = True
        self.mod.trajectory_mode = 1
        self.mod.radio = 50

        _gripper_write(self.robot, REG_FORCE, GRIPPER_FORCE_PERCENT)

        print("Motion parameters configured.")

        # -----------------------------
        # 3. Configure realtime UDP push
        # -----------------------------
        config = rm_realtime_push_config_t()
        config.cycle = 5               # 5 ms = 200 Hz
        config.enable = True
        config.port = 8089
        config.force_coordinate = -1
        config.ip = b"192.168.1.100"   # PC IP - RealMan requires this be 192.168.1.100 so hardcoded
        config.custom_config = rm_udp_custom_config_t()

        ret = self.robot.rm_set_realtime_push(config)
        if ret != 0:
            print("WARNING: Failed to enable realtime push")

        # -----------------------------
        # 4. Setup UDP listener
        # -----------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", 8089))
        print("UDP listener bound on 0.0.0.0:8089")

    def shutdown(self):
        self._gripper_running = False
        self._gripper_thread.join(timeout=1)
        self.robot.rm_close_modbus_mode(GRIPPER_PORT)
        self.robot.rm_delete_robot_arm()

    def _gripper_loop(self):
        while self._gripper_running:
            target = None
            with self._gripper_lock:
                if self._gripper_target is not None:
                    target = self._gripper_target
                    self._gripper_target = None

            if target is not None:
                _gripper_write(self.robot, REG_POSITION, target)

            pos = _gripper_read(self.robot, REG_CUR_POS)
            with self._gripper_lock:
                self._gripper_pos = pos

            time.sleep(0.05)

    def move_cartesian_velocity(self, velocities: List[float]):
        print("moving cartesian velocity with vel", velocities)
        if ROUTINE_IS_RUNNING: return
        print("moving cartesian velocity after routine check")

        try:
            # Realman has a different coordinate frame than Incar, so we transform
            v = (ctypes.c_float * 6)(
                velocities[0],
                velocities[1],
                velocities[2],
                -velocities[3],
                -velocities[4],
                -velocities[5]
            )
            self.mod.cartesian_velocity = ctypes.pointer(v)
            ret = rm_movev_canfd(self.handle, self.mod)
            if ret != 0:
                print(f"Failure moving robot: {ret}")
        except Exception as e:
            print(e)

    def move_gripper(self, gripper_zero_one: List[float]):
        with self._gripper_lock:
            self._gripper_target = int((1-gripper_zero_one[0])*1000)  # open: 1000, close: 0

    def handle_routines(self, routines: List[float]):
        if ROUTINE_IS_RUNNING: return

        try:
            # Find the index of the first routine that is higher than 0.5
            index = next(i for i, x in enumerate(routines) if x > 0.5)
        except StopIteration:
            return
        
        if index == 0:
            def _execute_routine():
                try:
                    print("Going Home")
                    global ROUTINE_IS_RUNNING
                    ROUTINE_IS_RUNNING = True
                    ret = self.robot.rm_movej(HOME_JOINT_POSITIONS, 20, 0, 0, 1)
                    ROUTINE_IS_RUNNING = False
                    print("Going Home Completed: ", ret)
                except:
                    traceback.print_exc()
        # Add other routines here, e.g. if index == 1: ...
        
        routine_thread = threading.Thread(target=_execute_routine, daemon=True)
        routine_thread.start()

    def set_state(self, interface: IncarRobotInterface):
        try:
            self.sock.setblocking(False)
            latest_data = None
            while True:
                try:
                    data, _ = self.sock.recvfrom(4096)
                    latest_data = data
                except BlockingIOError:
                    break

            if latest_data is None:
                return

            state_json = json.loads(latest_data)

            joint_pos = state_json.get("joint_status", {}).get("joint_position", [0] * 7)
            joint_pos = [x / 1000.0 for x in joint_pos]  # millidegrees -> degrees

            wp = state_json.get("waypoint", {})
            position = wp.get("position", [0, 0, 0])
            position = [x / 1_000_000.0 for x in position]  # micrometers -> meters
            quat = wp.get("quat", [0, 0, 0, 1])
            quat = [x / 1_000_000.0 for x in quat]          # micro units -> normalized
            ee_pose = position + quat  # [x, y, z, qx, qy, qz, qw]

            interface.set_robot_state(self.module_name, ee_pose=ee_pose, joint_pos=joint_pos)

            with self._gripper_lock:
                gripper_pos = self._gripper_pos
            interface.set_robot_state(self.module_name + "gripper", joint_pos=[gripper_pos])

        except Exception as e:
            print(f"publish_state_udp Exception: {e}")

    def set_state_and_publish(self, interface: IncarRobotInterface):
        self.set_state(interface)
        interface.publish_state()