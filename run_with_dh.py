from incar_networking.robot_interface import IncarRobotInterface
from realman_with_dh import RealManRobotWithDH

if __name__ == "__main__":
    try:
        dt_ms = 20
        robot = RealManRobotWithDH(dt_ms)
        interface = IncarRobotInterface(
            dt_ms / 1000,
            command_hooks={
                "right.commands.arm.ee.velocity": robot.move_cartesian_velocity,
                "right.commands.gripper.openclose": robot.move_gripper,
                "right.routines": robot.handle_routines
            },
            loop_callbacks=[
                robot.set_state_and_publish
            ]
        )
        interface.start()
    finally:
        robot.shutdown()