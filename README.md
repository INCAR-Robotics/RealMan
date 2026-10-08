## Prerequisites
You need access to the incar packages and the following packages installed

```bash
pip install incar_networking
pip install Robotic_Arm
```

If you want to additionally use the inspire hand, make sure that the inspire extension is also installed:
```bash
git clone https://github.com/INCAR-Robotics/inspire_extension.git
cd inspire_extension
pip install .
```

## Run
```bash
python3 run.py
```
Or to run along with the inspire hand
```bash
python3 run_with_inspire_hand.py
```
## Record & replay
Teach a motion by hand and play it back (arm + DH gripper):
```bash
python3 record_replay.py
```
Typical session: `free` (drag teach on) → `auto` to record continuously while you
move the arm, typing `open`/`close` to drive the gripper → Enter to stop → `hold`
→ `save my_move.json` → `play`. Use `r` instead of `auto` to record discrete
waypoints, `help` for the full command list.

Timestamps are recorded as they happened, so waypoints captured one `r` at a time
also capture however long you spent dragging the arm in between - `trim 1` caps
those pauses at a second each. Note that `play 0.5` is *half* speed; use `play 2`
to go faster.

### Joint vs linear
By default the arm interpolates in joint space between waypoints. `mode linear`
(or `--mode linear`) instead moves the tool in straight Cartesian lines, slerping
orientation along each leg, streamed as pose passthrough so the controller solves
IK each cycle. Linear replay can be rejected mid-path if a leg leaves the
workspace or crosses a singularity - joint mode always replays the taught arm
configuration, so it is the safer default. Recordings made before linear mode
existed have no tool pose stored and need re-recording to use it.

Replay from a file without the CLI:
```bash
python3 record_replay.py --file my_move.json --play --speed 0.5
```
