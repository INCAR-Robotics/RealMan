from incar_networking.robot_interface import IncarRobotInterface
from realman import RealManRobot

if __name__ == "__main__":
    dt_ms = 10
    robot = RealManRobot(dt_ms)
    interface = IncarRobotInterface(
        dt_ms / 1000,
        command_hooks={
            "right.commands.arm.ee.velocity": robot.move_cartesian_velocity,
            "right.routines": robot.handle_routines
        },
        loop_callbacks=[
            robot.set_state_and_publish
        ]
    )
    interface.start()
