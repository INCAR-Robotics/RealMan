#!/usr/bin/env python3
"""Interactive teach-and-replay for the RealMan arm + DH gripper.

Record timestamped waypoints (arm joints + gripper opening) from the CLI while
dragging the arm around, then replay them: the recorded waypoints are turned
into a smooth, time-parameterised joint trajectory that is streamed to the
controller over CANFD passthrough while the gripper is driven alongside it.

Run `python3 record_replay.py` and type `help`.
"""

import argparse
import json
import math
import os
import threading
import time
from typing import List, Optional

from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_thread_mode_e,
    rm_euler_t,
    rm_algo_euler2quaternion,
)

# Register map / low level Modbus helpers are shared with the teleop entrypoint
# so the gripper is only described in one place.
from realman_with_dh import (
    ROBOT_IP,
    ROBOT_PORT,
    HOME_JOINT_POSITIONS,
    GRIPPER_PORT,
    GRIPPER_BAUD,
    GRIPPER_FORCE_PERCENT,
    REG_INIT,
    REG_FORCE,
    REG_POSITION,
    REG_CUR_POS,
    _gripper_write,
    _gripper_read,
    _gripper_wait_init,
)

ARM_DOF = 7
GRIPPER_OPEN = 1000
GRIPPER_CLOSED = 0

DEFAULT_DT_MS = 20          # passthrough streaming period
DEFAULT_AUTO_HZ = 20.0      # continuous recording sample rate
MAX_JOINT_SPEED = 45.0      # deg/s, joint mode segments are stretched to respect this
MAX_LINEAR_SPEED = 0.20     # m/s, linear mode tool speed limit
MAX_ANGULAR_SPEED = 1.00    # rad/s, linear mode tool rotation limit

# Orientation counts towards path length so that a pure wrist rotation still
# advances along the path instead of registering as a zero-length segment.
ROT_WEIGHT = 0.10           # metres of path length per radian
MIN_SEGMENT_TIME = 0.05     # s, guards against two waypoints sharing a timestamp
APPROACH_SPEED = 20         # movej speed percentage used to reach the first waypoint
CONTROLLER_RETRIES = 2      # retries for blocking calls that time out waiting for a reply
LONG_REPLAY_WARN = 60.0     # s, ask before replaying anything longer than this

# Continuous recording drops samples the arm did not actually move for, but never
# skips more than this so that deliberate pauses survive into the replay.
AUTO_JOINT_EPS = 0.05       # deg
AUTO_MAX_GAP = 0.5          # s


def _fmt_duration(seconds: float) -> str:
    if seconds < 60:
        return f"{seconds:.2f}s"
    return f"{seconds / 60:.1f}min ({seconds:.0f}s)"


def call_controller(fn, *args, what: str = "command") -> int:
    """Issue a blocking controller call, retrying if the reply never arrives.

    ret=-2 means the controller did not answer in time, not that it rejected the
    command - the arm has often executed it anyway. Every call made through here
    (movej to a fixed target, start/stop drag teach) is idempotent, so re-issuing
    is safe and normally succeeds straight away.
    """
    ret = fn(*args)
    for attempt in range(1, CONTROLLER_RETRIES + 1):
        if ret != -2:
            break
        print(f"  {what}: no reply from controller, retrying "
              f"({attempt}/{CONTROLLER_RETRIES})")
        time.sleep(0.3)
        ret = fn(*args)
    return ret


class Waypoint:
    """One recorded instant: where the arm was and what the gripper was doing."""

    def __init__(self, t: float, joint: List[float], gripper: int,
                 pose: Optional[List[float]] = None,
                 gripper_pos: Optional[int] = None, label: str = ""):
        self.t = t
        self.joint = list(joint)
        # Tool pose [x, y, z, rx, ry, rz] in m / rad, needed for linear replay.
        self.pose = list(pose) if pose else None
        self.gripper = int(gripper)           # value to command on replay
        self.gripper_pos = gripper_pos        # measured opening, kept for reference
        self.label = label

    def to_dict(self):
        return {
            "t": round(self.t, 4),
            "joint": [round(j, 4) for j in self.joint],
            "pose": [round(v, 6) for v in self.pose] if self.pose else None,
            "gripper": self.gripper,
            "gripper_pos": self.gripper_pos,
            "label": self.label,
        }

    @staticmethod
    def from_dict(d):
        return Waypoint(
            t=float(d["t"]),
            joint=[float(j) for j in d["joint"]],
            pose=[float(v) for v in d["pose"]] if d.get("pose") else None,
            gripper=int(d.get("gripper", GRIPPER_OPEN)),
            gripper_pos=d.get("gripper_pos"),
            label=d.get("label", ""),
        )


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------

class GripperController:
    """Owns a dedicated connection + thread for the DH gripper.

    Modbus RTU round trips take tens of milliseconds, far more than the arm's
    passthrough period, so gripper traffic gets its own TCP connection and its
    own thread; the replay loop only ever hands it the latest target.
    """

    def __init__(self):
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        if not self.robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT):
            raise RuntimeError("Failed to create gripper robot arm handle")

        self.robot.rm_set_tool_voltage(3)  # 0=0V, 1=5V, 2=12V, 3=24V
        self.robot.rm_set_modbus_mode(GRIPPER_PORT, GRIPPER_BAUD, 1)
        time.sleep(1)

        _gripper_write(self.robot, REG_INIT, 0x01)
        _gripper_wait_init(self.robot)
        _gripper_write(self.robot, REG_FORCE, GRIPPER_FORCE_PERCENT)

        self._lock = threading.Lock()
        self._target = None
        self._pos = _gripper_read(self.robot, REG_CUR_POS) or GRIPPER_OPEN
        self._last_cmd = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("Gripper ready.")

    def _loop(self):
        while self._running:
            target = None
            with self._lock:
                if self._target is not None:
                    target = self._target
                    self._target = None

            if target is not None:
                _gripper_write(self.robot, REG_POSITION, target)

            pos = _gripper_read(self.robot, REG_CUR_POS)
            if pos is not None:
                with self._lock:
                    self._pos = pos

            time.sleep(0.05)

    def set_target(self, value: int):
        value = max(GRIPPER_CLOSED, min(GRIPPER_OPEN, int(value)))
        with self._lock:
            self._target = value
            self._last_cmd = value

    @property
    def position(self) -> int:
        with self._lock:
            return self._pos

    @property
    def last_command(self) -> Optional[int]:
        with self._lock:
            return self._last_cmd

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=1)
        self.robot.rm_close_modbus_mode(GRIPPER_PORT)
        self.robot.rm_delete_robot_arm()


# ---------------------------------------------------------------------------
# Trajectory generation
# ---------------------------------------------------------------------------

def _monotone_tangents(times: List[float], values: List[float]) -> List[float]:
    """Fritsch-Carlson tangents for shape preserving cubic Hermite interpolation.

    Plain Catmull-Rom overshoots around direction changes, which on a robot means
    the arm swings past a taught waypoint. The Fritsch-Carlson limiter keeps every
    segment inside the interval spanned by its two waypoints.
    """
    n = len(values)
    if n < 2:
        return [0.0] * n

    slopes = [
        (values[i + 1] - values[i]) / (times[i + 1] - times[i])
        for i in range(n - 1)
    ]

    # Zero velocity at both ends: the replay starts and finishes at rest.
    tangents = [0.0] * n
    for i in range(1, n - 1):
        if slopes[i - 1] * slopes[i] <= 0:
            tangents[i] = 0.0  # local extremum, flatten it
        else:
            dt_prev = times[i] - times[i - 1]
            dt_next = times[i + 1] - times[i]
            w1 = 2 * dt_next + dt_prev
            w2 = dt_next + 2 * dt_prev
            tangents[i] = (w1 + w2) / (w1 / slopes[i - 1] + w2 / slopes[i])

    for i in range(n - 1):
        if slopes[i] == 0.0:
            tangents[i] = 0.0
            tangents[i + 1] = 0.0
            continue
        a = tangents[i] / slopes[i]
        b = tangents[i + 1] / slopes[i]
        s = a * a + b * b
        if s > 9.0:
            scale = 3.0 / (s ** 0.5)
            tangents[i] = scale * a * slopes[i]
            tangents[i + 1] = scale * b * slopes[i]

    return tangents


def euler_to_quat(euler: List[float]) -> List[float]:
    """[rx, ry, rz] rad -> [w, x, y, z], using the controller's own convention."""
    q = rm_algo_euler2quaternion(rm_euler_t(*euler))
    return [q.w, q.x, q.y, q.z]


def quat_angle(q0: List[float], q1: List[float]) -> float:
    """Rotation angle between two unit quaternions, in radians."""
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def quat_slerp(q0: List[float], q1: List[float], u: float) -> List[float]:
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:  # take the short way round
        q1 = [-c for c in q1]
        dot = -dot

    if dot > 0.9995:  # nearly identical, lerp and renormalise
        out = [a + (b - a) * u for a, b in zip(q0, q1)]
    else:
        theta = math.acos(max(-1.0, min(1.0, dot)))
        sin_theta = math.sin(theta)
        w0 = math.sin((1.0 - u) * theta) / sin_theta
        w1 = math.sin(u * theta) / sin_theta
        out = [a * w0 + b * w1 for a, b in zip(q0, q1)]

    norm = math.sqrt(sum(c * c for c in out)) or 1.0
    return [c / norm for c in out]


class JointTrajectory:
    """Smooth joint-space path through the waypoints, sampled by time."""

    mode = "joint"

    def __init__(self, times: List[float], waypoints: List[Waypoint]):
        self.times = times
        self.waypoints = waypoints
        self.duration = times[-1]
        self._tangents = [
            _monotone_tangents(times, [wp.joint[j] for wp in waypoints])
            for j in range(ARM_DOF)
        ]

    def _segment(self, t: float) -> int:
        lo, hi = 0, len(self.times) - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.times[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def sample(self, t: float):
        """Return (joint angles, gripper command) at time t."""
        t = max(0.0, min(self.duration, t))
        i = self._segment(t)
        t0, t1 = self.times[i], self.times[i + 1]
        h = t1 - t0
        u = (t - t0) / h

        # Cubic Hermite basis
        u2, u3 = u * u, u * u * u
        h00 = 2 * u3 - 3 * u2 + 1
        h10 = u3 - 2 * u2 + u
        h01 = -2 * u3 + 3 * u2
        h11 = u3 - u2

        a, b = self.waypoints[i], self.waypoints[i + 1]
        joints = []
        for j in range(ARM_DOF):
            m0 = self._tangents[j][i]
            m1 = self._tangents[j][i + 1]
            joints.append(h00 * a.joint[j] + h10 * h * m0 + h01 * b.joint[j] + h11 * h * m1)

        gripper = int(round(a.gripper + (b.gripper - a.gripper) * u))
        return joints, gripper

    def overspeed(self, dt: float):
        """How far the sampled trajectory exceeds the speed limit, as a ratio."""
        peak = 0.0
        prev, _ = self.sample(0.0)
        for k in range(1, max(1, int(self.duration / dt)) + 1):
            joints, _ = self.sample(k * dt)
            peak = max(peak, max(abs(a - b) for a, b in zip(joints, prev)) / dt)
            prev = joints
        return peak / MAX_JOINT_SPEED, f"{peak:.1f} deg/s"

    def send(self, robot, value, follow: bool, trajectory_mode: int, radio: int) -> int:
        return robot.rm_movej_canfd(value, follow, 0, trajectory_mode, radio)


class LinearTrajectory:
    """Straight-line tool path through the waypoints, sampled by time.

    The path is exactly the polyline joining the recorded tool positions (with
    orientation slerped along each leg), so segments are straight in Cartesian
    space rather than in joint space. Timing comes from a monotone cubic fitted
    to cumulative path length, which keeps the recorded arrival time at every
    waypoint, starts and finishes at rest, and - unlike a chain of blocking
    movel calls - does not stop dead at each intermediate waypoint.
    """

    mode = "linear"

    def __init__(self, times: List[float], waypoints: List[Waypoint]):
        self.times = times
        self.waypoints = waypoints
        self.duration = times[-1]

        self._positions = [wp.pose[:3] for wp in waypoints]
        self._quats = [euler_to_quat(wp.pose[3:]) for wp in waypoints]

        # Cumulative path length; position and orientation share one metric so a
        # single scalar parameterises progress along the whole polyline.
        self._lengths = [0.0]
        for i in range(1, len(waypoints)):
            step = math.dist(self._positions[i - 1], self._positions[i])
            step += ROT_WEIGHT * quat_angle(self._quats[i - 1], self._quats[i])
            self._lengths.append(self._lengths[-1] + step)

        # Length is non-decreasing, so the monotone fit cannot double back.
        self._tangents = _monotone_tangents(times, self._lengths)

    def _time_segment(self, t: float) -> int:
        lo, hi = 0, len(self.times) - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.times[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def _distance_at(self, t: float) -> float:
        i = self._time_segment(t)
        t0, t1 = self.times[i], self.times[i + 1]
        h = t1 - t0
        u = (t - t0) / h
        u2, u3 = u * u, u * u * u
        return (
            (2 * u3 - 3 * u2 + 1) * self._lengths[i]
            + (u3 - 2 * u2 + u) * h * self._tangents[i]
            + (-2 * u3 + 3 * u2) * self._lengths[i + 1]
            + (u3 - u2) * h * self._tangents[i + 1]
        )

    def _path_segment(self, distance: float) -> int:
        lo, hi = 0, len(self._lengths) - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self._lengths[mid] <= distance:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def sample(self, t: float):
        """Return ([x, y, z, qw, qx, qy, qz], gripper command) at time t."""
        t = max(0.0, min(self.duration, t))

        distance = self._distance_at(t)
        i = self._path_segment(distance)
        span = self._lengths[i + 1] - self._lengths[i]
        # A zero-length leg means both ends hold the same pose, so u is moot.
        u = (distance - self._lengths[i]) / span if span > 0 else 0.0
        u = max(0.0, min(1.0, u))

        p0, p1 = self._positions[i], self._positions[i + 1]
        position = [a + (b - a) * u for a, b in zip(p0, p1)]
        quat = quat_slerp(self._quats[i], self._quats[i + 1], u)

        # The gripper stays on the recorded time base: a waypoint that only
        # closes the jaws adds no path length and would otherwise be skipped.
        g = self._time_segment(t)
        gt0, gt1 = self.times[g], self.times[g + 1]
        gu = (t - gt0) / (gt1 - gt0)
        a, b = self.waypoints[g], self.waypoints[g + 1]
        gripper = int(round(a.gripper + (b.gripper - a.gripper) * gu))

        return position + quat, gripper

    def overspeed(self, dt: float):
        peak_linear = 0.0
        peak_angular = 0.0
        prev, _ = self.sample(0.0)
        for k in range(1, max(1, int(self.duration / dt)) + 1):
            pose, _ = self.sample(k * dt)
            peak_linear = max(peak_linear, math.dist(prev[:3], pose[:3]) / dt)
            peak_angular = max(peak_angular, quat_angle(prev[3:], pose[3:]) / dt)
            prev = pose
        ratio = max(peak_linear / MAX_LINEAR_SPEED, peak_angular / MAX_ANGULAR_SPEED)
        return ratio, f"{peak_linear:.3f} m/s, {peak_angular:.2f} rad/s"

    def send(self, robot, value, follow: bool, trajectory_mode: int, radio: int) -> int:
        return robot.rm_movep_canfd(value, follow, trajectory_mode, radio)


def _segment_min_time(a: Waypoint, b: Waypoint, mode: str) -> float:
    """Shortest time this segment may take without breaking a speed limit."""
    if mode == "linear":
        distance = math.dist(a.pose[:3], b.pose[:3])
        angle = quat_angle(euler_to_quat(a.pose[3:]), euler_to_quat(b.pose[3:]))
        return max(distance / MAX_LINEAR_SPEED, angle / MAX_ANGULAR_SPEED)
    travel = max(abs(a.joint[j] - b.joint[j]) for j in range(ARM_DOF))
    return travel / MAX_JOINT_SPEED


def plan_times(waypoints: List[Waypoint], speed: float, mode: str) -> List[float]:
    """Scale recorded timestamps by `speed` and stretch segments that are too fast."""
    times = [0.0]
    stretched = 0
    for i in range(1, len(waypoints)):
        recorded = (waypoints[i].t - waypoints[i - 1].t) / speed
        needed = max(MIN_SEGMENT_TIME,
                     _segment_min_time(waypoints[i - 1], waypoints[i], mode))
        if recorded < needed:
            stretched += 1
        times.append(times[-1] + max(recorded, needed))

    if stretched:
        print(f"  note: stretched {stretched} segment(s) to respect the "
              f"{mode} speed limit")
    return times


def missing_pose(waypoints: List[Waypoint]) -> bool:
    return any(wp.pose is None for wp in waypoints)


def build_trajectory(waypoints: List[Waypoint], speed: float, dt: float,
                     mode: str = "joint"):
    """Plan the timing, then slow the whole thing down until it is actually safe.

    Per-segment timing only bounds the *average* speed; the smooth time profile
    peaks roughly 1.5x above that inside a segment, so the planned trajectory is
    sampled and uniformly stretched until its measured peak is within limits.
    """
    build = LinearTrajectory if mode == "linear" else JointTrajectory
    times = plan_times(waypoints, speed, mode)
    traj = build(times, waypoints)

    for _ in range(4):
        scale, measured = traj.overspeed(dt)
        if scale <= 1.01:  # a hair over the limit is rounding, not a real overspeed
            break
        times = [t * scale for t in times]
        traj = build(times, waypoints)
        print(f"  note: slowed replay x{scale:.2f} (peak was {measured})")

    return traj


# ---------------------------------------------------------------------------
# Recorder / player
# ---------------------------------------------------------------------------

class TeachSession:
    def __init__(self, dt_ms: int, use_gripper: bool, follow: bool,
                 trajectory_mode: int, radio: int, mode: str = "joint"):
        self.dt = dt_ms / 1000.0
        self.mode = mode
        self.follow = follow
        self.trajectory_mode = trajectory_mode
        self.radio = radio

        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        if not self.robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT):
            raise RuntimeError("Failed to create robot arm handle")
        print(f"Robot connected at {ROBOT_IP}:{ROBOT_PORT}")

        self.gripper = GripperController() if use_gripper else None

        self.waypoints: List[Waypoint] = []
        self.dragging = False
        self._clock_ref = None  # wall clock of waypoint t=0

    # -- helpers ----------------------------------------------------------

    def shutdown(self):
        if self.dragging:
            self.robot.rm_stop_drag_teach()
        if self.gripper:
            self.gripper.shutdown()
        self.robot.rm_delete_robot_arm()

    def read_state(self):
        """Current (joint angles in deg, tool pose [x,y,z,rx,ry,rz] in m/rad)."""
        ret, state = self.robot.rm_get_current_arm_state()
        if ret != 0:
            print(f"  ERROR: could not read arm state, ret={ret}")
            return None, None
        return list(state["joint"]), list(state["pose"])

    def gripper_command_value(self) -> int:
        """What to replay for the gripper: the last CLI command, else the readback."""
        if not self.gripper:
            return GRIPPER_OPEN
        cmd = self.gripper.last_command
        return cmd if cmd is not None else self.gripper.position

    def capture(self, label: str = "", at: Optional[float] = None) -> Optional[Waypoint]:
        joints, pose = self.read_state()
        if joints is None:
            return None

        now = time.monotonic()
        if not self.waypoints:
            self._clock_ref = now
        elif self._clock_ref is None:
            # Recording resumed after a load: continue right after the last waypoint.
            self._clock_ref = now - self.waypoints[-1].t
        t = at if at is not None else now - self._clock_ref

        wp = Waypoint(
            t=t,
            joint=joints,
            pose=pose,
            gripper=self.gripper_command_value(),
            gripper_pos=self.gripper.position if self.gripper else None,
            label=label,
        )
        self.waypoints.append(wp)
        return wp

    # -- recording --------------------------------------------------------

    def set_drag(self, on: bool):
        if on:
            ret = call_controller(self.robot.rm_start_drag_teach, 0,
                                  what="start drag teach")
            if ret == 0:
                self.dragging = True
                print("Drag teach ON - the arm is free to move by hand.")
            else:
                print(f"  ERROR: rm_start_drag_teach failed, ret={ret}")
        else:
            ret = call_controller(self.robot.rm_stop_drag_teach,
                                  what="stop drag teach")
            if ret == 0:
                self.dragging = False
                print("Drag teach OFF - the arm holds its position.")
            else:
                print(f"  ERROR: rm_stop_drag_teach failed, ret={ret}")

    def auto_record(self, hz: float):
        """Sample continuously in the background while the user keeps typing."""
        period = 1.0 / hz
        stop = threading.Event()

        def sampler():
            last_kept = None
            last_kept_time = 0.0
            next_t = time.monotonic()
            while not stop.is_set():
                joints, _ = self.read_state()
                if joints is not None:
                    moved = last_kept is None or any(
                        abs(joints[j] - last_kept.joint[j]) > AUTO_JOINT_EPS
                        for j in range(ARM_DOF)
                    )
                    gripper_changed = (
                        last_kept is not None
                        and self.gripper_command_value() != last_kept.gripper
                    )
                    stale = (
                        last_kept is not None
                        and time.monotonic() - last_kept_time > AUTO_MAX_GAP
                    )
                    if moved or gripper_changed or stale:
                        wp = self.capture()
                        if wp is not None:
                            last_kept = wp
                            last_kept_time = time.monotonic()

                next_t += period
                stop.wait(max(0.0, next_t - time.monotonic()))

        thread = threading.Thread(target=sampler, daemon=True)
        print(f"Recording at {hz:.0f} Hz. Gripper commands (open/close/g <n>) still "
              f"work; empty line stops.")
        thread.start()
        try:
            while True:
                line = input("rec> ").strip()
                if not line:
                    break
                if not self.handle_gripper_command(line):
                    print("  only open/close/g <n> accepted while recording")
        except (EOFError, KeyboardInterrupt):
            print()
        finally:
            stop.set()
            thread.join(timeout=2)
        if self.waypoints:
            print(f"Stopped. {len(self.waypoints)} waypoint(s), "
                  f"{self.waypoints[-1].t:.2f}s total")
        else:
            print("Stopped. Nothing recorded.")

    def handle_gripper_command(self, line: str) -> bool:
        parts = line.split()
        cmd = parts[0].lower()
        if cmd not in ("open", "close", "g"):
            return False
        if not self.gripper:
            print("  gripper disabled (--no-gripper)")
            return True
        if cmd == "open":
            self.gripper.set_target(GRIPPER_OPEN)
        elif cmd == "close":
            self.gripper.set_target(GRIPPER_CLOSED)
        else:
            if len(parts) < 2:
                print("  usage: g <0-1000>")
                return True
            try:
                self.gripper.set_target(int(parts[1]))
            except ValueError:
                print("  usage: g <0-1000>")
        return True

    # -- replay -----------------------------------------------------------

    def play(self, speed: float = 1.0, confirm: bool = True):
        if len(self.waypoints) < 2:
            print("  need at least 2 waypoints to replay")
            return
        if self.dragging:
            print("  drag teach is still ON - run `hold` first")
            return
        if speed <= 0:
            print("  speed must be > 0")
            return
        if self.mode == "linear" and missing_pose(self.waypoints):
            print("  these waypoints have no tool pose - they predate linear mode, "
                  "so re-record them or switch back with `mode joint`")
            return

        traj = build_trajectory(self.waypoints, speed, self.dt, self.mode)

        pace = "as recorded" if speed == 1.0 else (
            f"{speed:g}x faster" if speed > 1.0 else f"{1 / speed:g}x slower")
        print(f"{len(self.waypoints)} waypoints, {_fmt_duration(traj.duration)} "
              f"(speed x{speed:g} = {pace}, {self.mode} interpolation)")

        # Waypoints recorded one `r` at a time carry all the dwell time in
        # between, which is easy to miss until the arm is crawling: `trim` fixes it.
        if confirm and traj.duration > LONG_REPLAY_WARN:
            answer = input("  that is a long replay - continue? [y/N] "
                           "(`trim 1` shortens the pauses) ").strip().lower()
            if answer not in ("y", "yes"):
                print("  cancelled")
                return

        first = self.waypoints[0]
        print("Moving to the first waypoint...")
        if self.gripper:
            self.gripper.set_target(first.gripper)
        ret = call_controller(self.robot.rm_movej, first.joint, APPROACH_SPEED,
                              0, 0, 1, what="approach movej")
        if ret != 0:
            print(f"  ERROR: approach movej failed, ret={ret} - aborting replay")
            return
        time.sleep(0.5)

        print("Replaying... (Ctrl+C to stop)")
        start = time.monotonic()
        step = 0
        late = 0
        worst_late = 0.0
        aborted = False
        try:
            while True:
                t = step * self.dt
                target, gripper = traj.sample(t)

                ret = traj.send(self.robot, target, self.follow,
                                self.trajectory_mode, self.radio)
                if ret != 0:
                    print(f"  ERROR: passthrough rejected at t={t:.2f}s, ret={ret} "
                          f"- stopping")
                    if self.mode == "linear":
                        # The controller solves IK on every pose it is handed, so a
                        # rejection here usually means unreachable or singular.
                        print("    the tool path may leave the workspace or cross a "
                              "singularity; `mode joint` replays the taught joint "
                              "angles instead")
                    aborted = True
                    break
                if self.gripper:
                    self.gripper.set_target(gripper)

                if t >= traj.duration:
                    break

                step += 1
                slack = (start + step * self.dt) - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    late += 1
                    worst_late = max(worst_late, -slack)
        except KeyboardInterrupt:
            print("\n  interrupted - the arm holds where it stopped")
            return

        elapsed = time.monotonic() - start
        if aborted:
            print(f"Replay aborted after {elapsed:.2f}s - the arm holds where it stopped")
            return
        print(f"Replay finished in {elapsed:.2f}s ({step + 1} cycles)")
        if late:
            print(f"  warning: {late} cycle(s) ran late, worst {worst_late * 1000:.1f}ms - "
                  f"consider a larger --dt-ms")

    def goto(self, index: int):
        wp = self.waypoints[index]
        if self.gripper:
            self.gripper.set_target(wp.gripper)
        ret = call_controller(self.robot.rm_movej, wp.joint, APPROACH_SPEED,
                              0, 0, 1, what="movej")
        print(f"  movej to waypoint {index}: ret={ret}")

    def home(self):
        if self.gripper:
            self.gripper.set_target(GRIPPER_OPEN)
        ret = call_controller(self.robot.rm_movej, HOME_JOINT_POSITIONS,
                              APPROACH_SPEED, 0, 0, 1, what="movej home")
        print(f"  movej home: ret={ret}")

    # -- persistence ------------------------------------------------------

    def save(self, path: str):
        data = {
            "version": 1,
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "robot_ip": ROBOT_IP,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  saved {len(self.waypoints)} waypoint(s) to {path}")

    def load(self, path: str):
        with open(path) as f:
            data = json.load(f)
        self.waypoints = [Waypoint.from_dict(d) for d in data["waypoints"]]
        self._clock_ref = None
        span = _fmt_duration(self.waypoints[-1].t) if self.waypoints else "0s"
        print(f"  loaded {len(self.waypoints)} waypoint(s) from {path}, spanning {span}")

    # -- editing ----------------------------------------------------------

    def list_waypoints(self):
        if not self.waypoints:
            print("  (no waypoints)")
            return
        for i, wp in enumerate(self.waypoints):
            joints = " ".join(f"{j:7.2f}" for j in wp.joint)
            label = f"  {wp.label}" if wp.label else ""
            print(f"  [{i:3d}] t={wp.t:7.2f}s  j=[{joints}]  grip={wp.gripper:4d}{label}")

    def set_mode(self, mode: str):
        if mode not in ("joint", "linear"):
            print("  usage: mode joint|linear")
            return
        if mode == "linear" and self.waypoints and missing_pose(self.waypoints):
            print("  warning: the loaded waypoints have no tool pose, "
                  "re-record them before replaying in linear mode")
        self.mode = mode
        print(f"  interpolation mode: {mode}")

    def reset(self):
        self.waypoints.clear()
        self._clock_ref = None

    def trim(self, max_gap: float):
        """Clamp the pause between consecutive waypoints.

        Recording with `r` bakes in however long you spent dragging the arm between
        presses, so a handful of waypoints can easily span minutes. This keeps the
        recorded ordering and relative pacing but caps each gap.
        """
        if len(self.waypoints) < 2:
            return
        before = self.waypoints[-1].t
        previous_t = self.waypoints[0].t
        self.waypoints[0].t = 0.0
        for i in range(1, len(self.waypoints)):
            gap = self.waypoints[i].t - previous_t
            previous_t = self.waypoints[i].t
            self.waypoints[i].t = self.waypoints[i - 1].t + min(gap, max_gap)
        print(f"  capped gaps at {max_gap}s: {_fmt_duration(before)} -> "
              f"{_fmt_duration(self.waypoints[-1].t)}")

    def retime(self, spacing: float):
        for i, wp in enumerate(self.waypoints):
            wp.t = i * spacing
        print(f"  respaced {len(self.waypoints)} waypoint(s) at {spacing}s apart")

    def set_time(self, index: int, t: float):
        self.waypoints[index].t = t
        self.waypoints.sort(key=lambda wp: wp.t)
        base = self.waypoints[0].t
        for wp in self.waypoints:
            wp.t -= base
        print("  waypoint retimed, re-sorted by timestamp")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

HELP = """
Recording
  free                 drag teach ON  (move the arm by hand)
  hold                 drag teach OFF (arm holds position)
  r [label]            record one waypoint now (timestamp = time since first one)
  auto [hz]            record continuously until you press Enter (default 20 Hz)

Gripper (works during `auto` too; the value is stored with the next waypoint)
  open | close | g N   command the gripper (N = 0 closed .. 1000 open)

Waypoints
  ls                   list waypoints
  del [n]              delete waypoint n (default: the last one)
  clear                delete all waypoints
  t <n> <sec>          set the timestamp of waypoint n
  trim [sec]           cap the pause between waypoints (default 1s) - use this if
                       `r` recorded your thinking time between presses
  retime <sec>         respace all waypoints evenly

Motion
  mode [joint|linear]  how to move between waypoints: joint space (default, always
                       reachable) or straight lines in Cartesian space
  play [speed]         replay the recording (x2 = twice as fast, x0.5 = half speed)
  goto <n>             movej straight to waypoint n
  home                 movej to HOME_JOINT_POSITIONS

Files
  save [file]          save to JSON (default: the --file argument)
  load <file>          load from JSON

  help                 this text
  q / quit             exit
"""


def run_cli(session: TeachSession, default_file: str):
    print(HELP)
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd in ("q", "quit", "exit"):
                return
            elif cmd in ("help", "?", "h"):
                print(HELP)
            elif cmd == "free":
                session.set_drag(True)
            elif cmd == "hold":
                session.set_drag(False)
            elif cmd == "r":
                wp = session.capture(label=" ".join(args))
                if wp:
                    print(f"  [{len(session.waypoints) - 1}] t={wp.t:.2f}s "
                          f"grip={wp.gripper}")
            elif cmd == "auto":
                hz = float(args[0]) if args else DEFAULT_AUTO_HZ
                session.auto_record(hz)
            elif cmd in ("open", "close", "g"):
                session.handle_gripper_command(line)
            elif cmd == "ls":
                session.list_waypoints()
            elif cmd == "del":
                if not session.waypoints:
                    print("  (no waypoints)")
                    continue
                index = int(args[0]) if args else len(session.waypoints) - 1
                session.waypoints.pop(index)
                print(f"  deleted waypoint {index}")
            elif cmd == "clear":
                session.reset()
                print("  cleared")
            elif cmd == "t":
                session.set_time(int(args[0]), float(args[1]))
            elif cmd == "trim":
                session.trim(float(args[0]) if args else 1.0)
            elif cmd == "retime":
                session.retime(float(args[0]))
            elif cmd == "mode":
                if args:
                    session.set_mode(args[0].lower())
                else:
                    print(f"  interpolation mode: {session.mode}")
            elif cmd == "play":
                session.play(float(args[0]) if args else 1.0)
            elif cmd == "goto":
                session.goto(int(args[0]))
            elif cmd == "home":
                session.home()
            elif cmd == "save":
                session.save(args[0] if args else default_file)
            elif cmd == "load":
                session.load(args[0] if args else default_file)
            else:
                print(f"  unknown command: {cmd} (try `help`)")
        except (IndexError, ValueError) as e:
            print(f"  bad arguments: {e}")
        except FileNotFoundError as e:
            print(f"  {e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--file", default="recording.json",
                        help="default file for save/load (default: recording.json)")
    parser.add_argument("--play", action="store_true",
                        help="load --file, replay it once and exit")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="replay speed multiplier (default: 1.0)")
    parser.add_argument("--dt-ms", type=int, default=DEFAULT_DT_MS,
                        help=f"passthrough period in ms (default: {DEFAULT_DT_MS})")
    parser.add_argument("--mode", choices=("joint", "linear"), default="joint",
                        help="interpolation between waypoints (default: joint)")
    parser.add_argument("--no-gripper", action="store_true",
                        help="skip gripper setup (arm only)")
    parser.add_argument("--low-follow", action="store_true",
                        help="use low follow mode - safer if cycles run late")
    parser.add_argument("--trajectory-mode", type=int, default=0,
                        help="high follow mode: 0=passthrough, 1=curve fit, 2=filter")
    parser.add_argument("--radio", type=int, default=0,
                        help="smoothing factor for trajectory modes 1 and 2")
    args = parser.parse_args()

    session = TeachSession(
        dt_ms=args.dt_ms,
        use_gripper=not args.no_gripper,
        follow=not args.low_follow,
        trajectory_mode=args.trajectory_mode,
        radio=args.radio,
        mode=args.mode,
    )
    try:
        if args.play:
            if not os.path.exists(args.file):
                print(f"  {args.file} not found")
                return
            session.load(args.file)
            session.play(args.speed, confirm=False)
        else:
            run_cli(session, args.file)
    finally:
        session.shutdown()
        print("Disconnected.")


if __name__ == "__main__":
    main()
