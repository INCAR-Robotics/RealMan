from typing import List
from Robotic_Arm.rm_robot_interface import *
from incar_networking.robot_interface import IncarRobotInterface


class RealManRobot:
    def __init__(self, dt_ms: int):
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm("192.168.1.18", 8080)

        self.robot.rm_set_movev_canfd_init(avoid_singularity_flag=1, frame_type=1, dt = dt_ms)
        self.robot.rm_set_arm_max_line_speed(0.2)
        self.robot.rm_set_arm_max_angular_speed(3.14 / 2)
        self.robot.rm_set_arm_max_angular_acc(3.14*3)
        
        self.mod = rm_movev_canfd_mode_t()
        self.mod.follow = False
        self.mod.trajectory_mode = 1
        self.mod.radio = 50

    def move_cartesian_velocity(self, velocities: List[float]):
        try:
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

    def publish_state(self, interface: IncarRobotInterface):
        ret, state = self.robot.rm_get_current_arm_state()
        if ret == 0:
            interface.set_robot_state("arm", ee_pose=state["pose"])
            interface.set_robot_state("arm", joint_pos=state["joint"])
            interface.publish_state()
        else:
            print(f"Failure reading state: {ret}")

        ret, sensor_data = self.robot.rm_get_force_data()
        if ret == 0:
            interface.publish_sensor("force_torque", sensor_data["force_data"])
        else:
            print(f"Failure reading force sensor: {ret}")


if __name__ == "__main__":
    dt_ms = 10
    robot = RealManRobot(dt_ms)
    interface = IncarRobotInterface(
        dt_ms / 1000,
        command_hooks = {
            "right.commands.arm.ee.velocity": robot.move_cartesian_velocity
        },
        loop_callbacks = [
            robot.publish_state
        ],
        sensor_names= [
            "force_torque"
        ]
    )
    interface.start()