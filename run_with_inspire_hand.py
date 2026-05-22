import multiprocessing as mp
import traceback

from incar_networking.robot_interface import IncarRobotInterface
from inspire_extension.robot_interface.inspire_interface import write6, read6, openSerial
from realman import RealManRobot


OPEN_POSE = [1740, 1740, 1740, 1740, 1350, 1700]

# Something weird going on when combining InspireHand with RealMan, so we use multiprocessing to
# run the hand in a separate process
class MultiProcessInspire:
    def __init__(self, module_name = "hand"):
        self.module_name = module_name
        self.target_angles = mp.Array('i', 6)
        self.command_flag = mp.Value('b', False)
        self.current_angles = mp.Array('i', 6)
        self.current_forces = mp.Array('i', 6)
        self.hand_process = mp.Process(
            target=self.hand_process_func,
            args=(self.target_angles, self.command_flag, self.current_angles, self.current_forces)
        )
        self.hand_process.start()

    def hand_process_func(self, target_angles, command_flag, current_angles, current_forces):
        try:
            serial = openSerial('/dev/ttyUSB0', 115200)
            write6(serial, 1, 'mode', [0, 0, 0, 0, 0, 0])
            write6(serial, 1, 'speedSet', [1000] * 6)  # Range 0 - 4000
            write6(serial, 1, 'forceSet', [1000] * 6)  # Range 0 - 12000
            write6(serial, 1, 'angleSet', OPEN_POSE)
            print("Hand initialised — fingers opening.")
            while True:
                if command_flag.value:
                    write6(serial, 1, 'angleSet', list(target_angles))
                    command_flag.value = False
                angles = read6(serial, 1, 'angleAct')
                for i in range(6):
                    current_angles[i] = angles[i]
                forces = read6(serial, 1, 'forceAct')
                for i in range(6):
                    current_forces[i] = forces[i]
        except Exception as e:
            print(f"Hand process error: {e}")
        except:
            traceback.print_exc()

    def move_joints(self, joint_positions: list[float]):
        try:
            for i in range(6):
                self.target_angles[i] = int(joint_positions[i])
            self.command_flag.value = True
        except:
            traceback.print_exc()

    def set_state(self, interface: IncarRobotInterface):
        try:
            current_angles = [self.current_angles[i] / 10.0 for i in range(6)]   # -> degrees
            current_forces = [self.current_forces[i] / 1000.0 for i in range(6)] # -> 0-1
            interface.set_robot_state(self.module_name, joint_pos=current_angles, joint_effort=current_forces)
        except:
            traceback.print_exc()


if __name__ == "__main__":
    dt_ms = 10
    robot = RealManRobot(dt_ms)
    hand = MultiProcessInspire()
    interface = IncarRobotInterface(
        dt_ms / 1000,
        command_hooks={
            "right.commands.arm.ee.velocity": robot.move_cartesian_velocity,
            "right.routines": robot.handle_routines,
            "left.commands.hand.inspire": hand.move_joints,
        },
        loop_callbacks=[
            hand.set_state,
            robot.set_state_and_publish
        ]
    )
    interface.start()
