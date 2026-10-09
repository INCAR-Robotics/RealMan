"""Hard-coded calorimetry routine, extracted from the teleop demo in
datasets/astra_routine/demo_0.

Every waypoint is a moment where the operator pressed 'primary' on the left
controller: the arm.ee.pose at that moment (quaternion converted to the euler
angles rm_movel expects) plus the gripper target teleop was commanding (same
open: 300 / closed: 0 mapping as RealManRobotWithDH.move_gripper). The command
is used rather than the measured gripper position because a grasp reads e.g. 89
(jaws stopped on the object) while the command was fully closed, and replaying
89 would barely squeeze the object.

Poses are in the base frame with the controller's tool frame that was active
during the recording, so that tool frame must be active when this runs.
"""

import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# L2 vision (powder level -> hygrostat size), see astra_calorimetry/code/vision/README.md
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "astra_calorimetry" / "code" / "vision"))
from l2 import L2Vision

CALIBRATED_Z_OFFSET = -0.01

ROUTINE_SPEED = 20          # movej speed percentage, 1~100
LINEAR_SPEED = 0.08         # m/s, peak tool speed of the straight-line moves
ANGULAR_SPEED = 0.5         # rad/s, peak tool rotation speed of the straight-line moves
MIN_MOVE_TIME = 0.5         # s
ARRIVAL_SETTLE_TIME = 0.2   # s, pause after the last streamed pose before checking arrival
ARRIVAL_TOLERANCE = 0.003   # m
GRIPPER_TOLERANCE = 10      # gripper counts, close enough to the target to continue
GRIPPER_SETTLE_TIME = 0.5   # s, gripper position unchanged this long -> stopped on an object
GRIPPER_TIMEOUT = 4.0       # s

# Waypoint 9 picks up the hygrostat that fits the measured powder level
WAYPOINT_NINE_LARGE = [0.2265, 0.1046, 0.0699, -3.1277, -0.0089, 3.1481]
WAYPOINT_NINE_MEDIUM = [0.2425, 0.1046, 0.0699, -3.1277, -0.0089, 3.1481]
WAYPOINT_NINE_SMALL = [0.2585, 0.1046, 0.0699, -3.1277, -0.0089, 3.1481]
WAYPOINT_NINE = {"small": WAYPOINT_NINE_SMALL, "medium": WAYPOINT_NINE_MEDIUM, "large": WAYPOINT_NINE_LARGE}

# ("movej", joints in deg) | ("movel", straight line to [x, y, z, rx, ry, rz] in m / rad) | ("gripper", 0 closed ~ 300 open)
# | ("measure", None) vision reading of the vial in the pocket, aborts the routine on NO-GO
# | ("movel_hygrostat", {hygrostat: pose}) movel to the pose of the measured hygrostat
# | ("movel_level", pose) movel to pose raised in z by the measured powder level
STEPS = [
    ("gripper", 250),  # waypoint  0 (t=  3.7s)
    # ("movej", [114.51, -36.85, -127.23, 65.38, -27.54, 90.18, -24.36]),  # waypoint  0 (t=  3.7s)
    ("movel", [0.2624, -0.2050, 0.1540, 3.1021, 0.0210, -3.0453]),  # waypoint  1 (t= 20.7s)
    ("movel", [0.2624, -0.2050, 0.0940, 3.1021, 0.0210, -3.0453]),  # waypoint  1 (t= 20.7s)
    ("gripper", 150),  # waypoint  2 (t= 23.9s)
    ("movel", [0.2587, -0.1938, 0.1938, -3.0887, -0.0403, -2.9780]),  # waypoint  3 (t= 30.3s)
    ("movel", [0.2431, 0.0204, 0.1288, 3.0684, -0.0074, -3.0444]),  # waypoint  4 (t= 47.3s)
    ("movel", [0.2438, 0.0202, 0.1092, 3.0449, -0.0008, -3.0352]),  # waypoint  5 (t= 57.0s)
    ("gripper", 250),  # waypoint  6 (t= 60.0s)
    ("movel", [0.2424, 0.0231, 0.1523, 3.0739, 0.0047, -3.0560]),  # waypoint  7 (t= 66.9s)
    ("movel", [0.2438, 0.1062, 0.1114, 3.0233, 0.0138, -3.1103]),  # waypoint  8 (t= 73.4s)
    ("measure", None),  # vial is in the pocket and the arm is clear of the camera
    ("movel_hygrostat", WAYPOINT_NINE),  # waypoint  9 (t= 96.8s)
    ("gripper", 0),  # waypoint 10 (t= 99.9s)
    ("movel", [0.2476, 0.1111, 0.1120, -3.1054, -0.0155, 3.1496]),  # waypoint 11 (t=122.5s)
    ("movel", [0.2448, 0.0277, 0.1613, -3.0730, -0.0315, -3.0957]),  # waypoint 12 (t=131.3s)
    # ("movel", [0.2458, 0.0283, 0.0931, 3.1399, -0.0245, -3.0897]),  # waypoint 13 (t=144.4s)
    ("movel_level", [0.2433, 0.0277, 0.0941, 3.1320, -0.0134, -3.0989]),  # waypoint 14 (t=186.3s)
    ("gripper", 21),  # waypoint 15 (t=202.7s)
    ("movel", [0.2462, 0.0327, 0.1265, 3.1321, -0.0144, -3.1143]),  # waypoint 16 (t=226.4s)
    # ("gripper", 21),  # waypoint 16 (t=226.4s)
    ("gripper", 250),  # waypoint 17 (t=231.6s)
    ("movel", [0.2427, 0.0261, 0.1058, 3.1321, -0.0009, -3.0957]),  # waypoint 18 (t=242.2s)
    ("gripper", 150),  # waypoint 19 (t=244.5s)
    ("movel", [0.2455, 0.0283, 0.1838, -3.1133, -0.0448, -3.0997]),  # waypoint 20 (t=249.3s)
    ("movel", [0.2637, -0.2007, 0.1482, -3.0966, -0.0521, -2.9563]),  # waypoint 21 (t=263.4s)
    ("movel", [0.2623, -0.1990, 0.0948, 3.1194, 0.0004, -2.9606]),  # waypoint 22 (t=274.8s)
    ("gripper", 250),  # waypoint 23 (t=286.9s)
    ("movel", [0.2625, -0.2029, 0.1481, -3.1264, -0.0265, -2.8988]),  # waypoint 24 (t=291.3s)
]


def _wait_for_gripper(robot, target):
    # The gripper thread only picks up the new target on its next cycle, so give it
    # a moment before treating an unchanged position as "stopped on an object".
    time.sleep(0.2)
    start = time.time()
    last_pos = robot.get_gripper_position()
    last_change = time.time()
    while time.time() - start < GRIPPER_TIMEOUT:
        pos = robot.get_gripper_position()
        if abs(pos - target) <= GRIPPER_TOLERANCE:
            return
        if pos != last_pos:
            last_pos = pos
            last_change = time.time()
        elif time.time() - last_change > GRIPPER_SETTLE_TIME:
            return
        time.sleep(0.05)
    print(f"  WARNING: gripper did not settle at {target} (at {last_pos})")


def _current_pose(robot):
    """(position, Rotation) of the tool, from the realtime push rather than a TCP
    request, whose reply sometimes times out (ret=-2)."""
    x, y, z, qw, qx, qy, qz = robot.get_ee_pose()
    return np.array([x, y, z]), Rotation.from_quat([qx, qy, qz, qw])


def _rotation(euler):
    # Lowercase 'xyz' (extrinsic) matches the controller's euler convention
    return Rotation.from_euler("xyz", euler)


def _move_linear(robot, target):
    """Straight-line move to target [x, y, z, rx, ry, rz], streamed over CANFD.

    rm_movel is not used: with the tool pointing down, rx and rz sit at the +-pi
    wrap-around, and the controller interpolates euler angles numerically, so a 10
    deg rotation between waypoints on either side of it becomes a ~350 deg wrist
    spin ("joint6 overspeed"). Its blocking wait also times out (ret=-2) on longer
    moves. Interpolating here with a quaternion slerp always takes the short way.
    """
    p0, r0 = _current_pose(robot)
    p1, r1 = np.array(target[:3]), _rotation(target[3:])
    slerp = Slerp([0, 1], Rotation.concatenate([r0, r1]))

    # Minimum-jerk profile: starts and stops at rest, peak speed is 1.875x the average
    distance = np.linalg.norm(p1 - p0)
    angle = (r0.inv() * r1).magnitude()
    duration = max(1.875 * distance / LINEAR_SPEED, 1.875 * angle / ANGULAR_SPEED, MIN_MOVE_TIME)

    dt = robot.dt_ms / 1000.0
    steps = max(1, int(math.ceil(duration / dt)))
    t0 = time.monotonic()
    for k in range(1, steps + 1):
        u = k / steps
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        x, y, z, w = slerp(s).as_quat()
        pose = list(p0 + s * (p1 - p0)) + [w, x, y, z]  # controller wants [w, x, y, z]
        ret = robot.robot.rm_movep_canfd(pose, True, 0, 0)
        if ret != 0:
            raise RuntimeError(f"Pose streaming rejected at {u:.0%} of the move, ret={ret}")
        slack = t0 + k * dt - time.monotonic()
        if slack > 0:
            time.sleep(slack)

    time.sleep(ARRIVAL_SETTLE_TIME)
    error = np.linalg.norm(_current_pose(robot)[0] - p1)
    if error > ARRIVAL_TOLERANCE:
        raise RuntimeError(f"Arm stopped {error * 1000:.1f}mm from the target, check the controller for errors")


def _movej(robot, joints):
    # A blocking movej can return -2 when the controller's reply is late even though
    # the arm moved. Re-issuing a movej to a fixed target is harmless, so retry.
    ret = robot.robot.rm_movej(joints, ROUTINE_SPEED, 0, 0, 1)
    for _ in range(2):
        if ret != -2:
            break
        time.sleep(0.3)
        ret = robot.robot.rm_movej(joints, ROUTINE_SPEED, 0, 0, 1)
    if ret != 0:
        raise RuntimeError(f"movej failed with ret={ret}")


def _measure_vial(vision):
    vision.new_vial()
    result = vision.measure()
    print(f"  Vision: {result}")
    if result["verdict"] != "GO" or result["hygrostat"] not in WAYPOINT_NINE:
        raise InterruptedError(f"Vision NO-GO (status {result['status']}, level {result['level_mm']}mm, "
                           f"hygrostat {result['hygrostat']}), aborting the calorimetry routine")
    return result


def run_calorimetry_routine(robot):
    """Execute STEPS on a RealManRobotWithDH. Blocks until done, raises on a failed move
    or a vision NO-GO."""
    # Open the camera before moving, so a missing camera stops the routine before it starts
    vision = L2Vision()
    try:
        result = None
        for i, (kind, value) in enumerate(STEPS):
            print(f"Calorimetry step {i + 1}/{len(STEPS)}: {kind} {value}")
            if kind == "movej":
                _movej(robot, value)
            elif kind == "movel":
                _move_linear(robot, value)
            elif kind == "gripper":
                robot.set_gripper_target(value)
                _wait_for_gripper(robot, value)
            elif kind == "measure":
                result = _measure_vial(vision)
            elif kind == "movel_hygrostat":
                print(f"  Hygrostat: {result['hygrostat']}")
                _move_linear(robot, value[result["hygrostat"]])
            elif kind == "movel_level":
                target = list(value)
                target[2] += (result["level_mm"] / 1000.0) + CALIBRATED_Z_OFFSET
                print(f"  Raised {result['level_mm']}mm for the powder level: z={target[2]:.4f}")
                _move_linear(robot, target)
            else:
                raise ValueError(f"Unknown routine step: {kind}")
    except InterruptedError as e:
        print(e)
    finally:
        vision.release()
