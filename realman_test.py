import time
from Robotic_Arm.rm_robot_interface import *

def cartesian_move():
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm("192.168.1.18", 8080)

    avoid_singularity_flag = 1
    frame_type = 0
    dt = 10
    ret = robot.rm_set_movev_canfd_init(avoid_singularity_flag, frame_type, dt)
    print(ret)
    ret = -1
    mode = -1
    mod = rm_movev_canfd_mode_t()
    velocity = (ctypes.c_float * 6)(0.025, 0, 0, 0, 0, 0)
    mod.cartesian_velocity = ctypes.pointer(velocity) 
    mod.follow = False 
    mod.trajectory_mode = 1  
    mod.radio = 50  
    ret = rm_movev_canfd(handle, mod)
    # print(ret)
    # Call cyclically 400 times, with a 10ms delay each time (10ms = 0.01 seconds)
    for i in range(400):
        rm_movev_canfd(handle, mod)
        time.sleep(0.01)

def get_state():
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm("192.168.1.18", 8080)

    ret, states = robot.rm_get_current_arm_state()
    print(states["pose"])
if __name__ == "__main__":
    get_state()