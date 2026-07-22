import time

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e, rm_peripheral_read_write_params_t

ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

GRIPPER_PORT = 1        # 1 = end-effector interface board RS485 (tool flange)
GRIPPER_SLAVE_ID = 1    # DH default Modbus slave address
GRIPPER_BAUD = 115200   # DH AG-95 default

REG_INIT = 0x0100
REG_FORCE = 0x0101
REG_POSITION = 0x0103
REG_INIT_STATE = 0x0200
REG_STATE = 0x0201
REG_CUR_POS = 0x0202

STATE_NAMES = {0: "moving", 1: "reached", 2: "object caught", 3: "object dropped"}


def _write(robot, addr, value):
    p = rm_peripheral_read_write_params_t(port=GRIPPER_PORT, address=addr, device=GRIPPER_SLAVE_ID, num=1)
    ret = robot.rm_write_single_register(p, value)
    if ret != 0:
        print(f"  WARNING: write 0x{addr:04X}={value} failed, ret={ret}")
    return ret


def _read(robot, addr):
    p = rm_peripheral_read_write_params_t(port=GRIPPER_PORT, address=addr, device=GRIPPER_SLAVE_ID, num=1)
    ret, val = robot.rm_read_holding_registers(p)
    if ret != 0:
        print(f"  WARNING: read 0x{addr:04X} failed, ret={ret}")
    return val


def wait_until_settled(robot, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        state = _read(robot, REG_STATE)
        pos = _read(robot, REG_CUR_POS)
        print(f"  state={STATE_NAMES.get(state, state)} pos={pos}")
        if state in (1, 2, 3):
            return state
        time.sleep(0.2)
    print("  WARNING: timed out waiting for gripper to settle")
    return None


def main():
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT)
    if not handle:
        raise RuntimeError("Failed to create robot arm handle")
    print("Robot connected.")

    try:
        ret = robot.rm_set_tool_voltage(3)  # 0=0V, 1=5V, 2=12V, 3=24V
        print("Set tool flange voltage to 24V:", ret)

        ret = robot.rm_set_modbus_mode(GRIPPER_PORT, GRIPPER_BAUD, 1)
        print("Set Modbus RTU mode on tool flange:", ret)

        print("Initializing gripper...")
        _write(robot, REG_INIT, 0x01)
        wait_until_settled(robot, timeout=10.0)

        _write(robot, REG_FORCE, 50)  # 20-100 %

        print("Opening...")
        _write(robot, REG_POSITION, 1000)
        wait_until_settled(robot)

        time.sleep(1)

        print("Closing...")
        _write(robot, REG_POSITION, 0)
        wait_until_settled(robot)

        time.sleep(1)

        print("Opening...")
        _write(robot, REG_POSITION, 1000)
        wait_until_settled(robot)

    finally:
        robot.rm_close_modbus_mode(GRIPPER_PORT)
        robot.rm_delete_robot_arm()
        print("Disconnected.")


if __name__ == "__main__":
    main()
