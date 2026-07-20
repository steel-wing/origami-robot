import mujoco
import mujoco.viewer
import time
import numpy as np

'''
Different tasks is a valuable thing: we want expressivity, yes, but more than
that we want this thing to be able to move in different cool ways that 'hopefully'
use different modes for motion. 

Also, domain randomization: try messing about with the friction coefficients, (0.1, 0.5)
to hopefully find policies that are more robust to different environments. Mess around with
mass, density, motor strength, that kind of thing.
'''

def quat_to_yaw(q):
    # q = [w, x, y, z]
    w, x, y, z = q
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def calculateTheta2(theta1):
  return 2 * np.atan(np.sqrt(2) * np.tan((theta1) / 2))

# if hinges aren't moving as they should, penalize the result
def loss_corrections(left, right, loss, data, model, x_start_dist, y_start_dist, total_rotation):
    # get joint ids
    r_left = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "jaw_hinge_2")]]
    m_left = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "jaw_hinge_4")]]
    l_left = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "jaw_hinge_6")]]
    r_right = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "jaw_hinge_8")]]
    m_right = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "jaw_hinge_10")]]
    l_right = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "jaw_hinge_12")]]

    # distance along the the x or y axes
    x_current_dist = data.qpos[0]
    y_current_dist = data.qpos[1]

    threshold = 0.5
    # if all of the hinges should be moving together
    if left == 4:
        # but the difference between them is bigger than it ever should be, set loss to 0
        if abs(m_left - l_left) > threshold:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0
        elif abs(m_left - r_left) > threshold:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0
        elif abs(l_left - r_left) > threshold:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0

    else:   # if the hinges should NOT be moving together
        # this should trigger if all the hinges move together beyond ~30 degrees
        if r_left * m_left * l_left > 0.125:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0

    # repeat everything for the right side
    if right == 4:
        if abs(m_right - r_right) > threshold:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0
        elif abs(m_right - l_right) > threshold:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0
        elif abs(r_right - l_right) > threshold:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0
    else:
        if r_right * m_right * l_right > 0.125:
            x_start_dist = x_current_dist
            y_start_dist = y_current_dist
            total_rotation = 0

    return x_start_dist, y_start_dist, total_rotation

def bez_sim(x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, 
            left=1, right=2, orient=1, loss=0, 
            display=False, printy=False, friction=0.3):

    # import model
    model = mujoco.MjModel.from_xml_path(r'model\origami\origami_robot.xml')
    data = mujoco.MjData(model)

    # altered friction
    for geom_id in range(model.ngeom):
        model.geom_friction[geom_id, 0] = friction      # sliding
        model.geom_friction[geom_id, 1] = 0.005         # torsional
        model.geom_friction[geom_id, 2] = 0.0001        # rolling

    # re-orient the model
    data.qpos[3:7] = [1 - orient, 0, orient, 0]     # qpos holds joint positions, and the first joint [0:7]
    data.qpos[0:3] = [0, 0, 0]                      # is stored as [x, y, z, a, b(i), c(j), d(k)]
    mujoco.mj_forward(model, data)                  # so we make a=1 for one way, and c=1 for 180 deg flip around y axis

    x_start_dist = 0.0
    y_start_dist = 0.0
    total_rotation = 0.0
    prev_yaw = quat_to_yaw(data.qpos[3:7])

    # define a bezier curve for the motion of the bot
    points = [
        np.array([x1, y1, z1]),
        np.array([x2, y2, z2]),
        np.array([x3, y3, z3]),
        np.array([x4, y4, z4]),
        # add more points here if you want, just be sure to add them to other places wherever relevant
        # especially in the optimizer if that's your intention
    ]
    # bezier geometry
    n = len(points)
    midpoints = [(points[i] + points[(i + 1) % n]) / 2 for i in range(n)]
    segments = [(midpoints[i], points[(i + 1) % n], midpoints[(i + 1) % n]) for i in range(n)]

    def quad_bezier(t, p0, p1, p2):
        return (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2

    # bezier interpolator
    def f(t):
        t = t % n                               # wrap around loop
        i = int(t)                              # select which segment to be a part of
        local_t = t - i                         # normalize to [0, 1)
        p0, p1, p2 = segments[i]                # grab control points for curve
        return quad_bezier(local_t, p0, p1, p2)
    
    # time handling
    dt = 0.02   # this is the servo period, 20ms
    physics_steps = round(dt / model.opt.timestep) # this was a tricky one: the mujoco model has its own internal timestep, so we need to multiply that by a constant factor to get up to real-time speeds
    divisions = 100 # this variable comes from the microcontroller code, and is the inverse of the speed, in a sense. steps/(n*cycle)
    max_cycles = 8  # also from the microcontroller code. arbitrary.
    cycle_time = dt * n * divisions # this simply means how many seconds for one full cycle of the gait
    end = cycle_time * max_cycles # how long this thing should run for, in seconds
    
    # helper with per-timestep logic
    def simulation_step():
        nonlocal x_start_dist, y_start_dist, total_rotation, prev_yaw, current_phase, step
        
        # set initial position
        if step == 0:
            x_start_dist = data.qpos[0]
            y_start_dist = data.qpos[1]

        # ramp up one cycle, hold for (cycles - 2), ramp down for one cycle
        if data.time < cycle_time:
            scale = data.time / cycle_time
        elif data.time < (end - cycle_time):
            scale = 1
        else:
            scale = max(0, (end - data.time) / cycle_time)

        left_target, middle_target, right_target = f(current_phase)
        
        if left == 1:
            data.ctrl[0] = left_target*scale    # l_left
        if left == 2:
            data.ctrl[1] = left_target*scale    # r_left
        if left == 3:
            data.ctrl[2] = left_target*scale    # m_left
        if left == 4:
            data.ctrl[0] = calculateTheta2(left_target*scale)
            data.ctrl[1] = calculateTheta2(left_target*scale)
            data.ctrl[2] = left_target*scale
        data.ctrl[3] = middle_target*scale      # middle
        if right == 1:
            data.ctrl[5] = right_target*scale   # l_left
        if right == 2:
            data.ctrl[4] = right_target*scale   # r_left
        if right == 3:
            data.ctrl[6] = right_target*scale   # m_left
        if right == 4:
            data.ctrl[4] = calculateTheta2(right_target*scale)
            data.ctrl[5] = calculateTheta2(right_target*scale)
            data.ctrl[6] = right_target*scale

        # move physics forward until next servo update
        for _ in range(physics_steps):
            mujoco.mj_step(model, data)

        current_phase = data.time / (dt * divisions)
        
        # rotation and position losses
        current_yaw = quat_to_yaw(data.qpos[3:7])                
        delta = (current_yaw - prev_yaw + np.pi) % (2 * np.pi) - np.pi
        total_rotation += delta
        prev_yaw = current_yaw
        x_start_dist, y_start_dist, total_rotation = loss_corrections(
            left, right, loss, data, model, x_start_dist, y_start_dist, total_rotation
        )
        step += 1

    # keeping track of state and time
    target_time = time.perf_counter()
    current_phase = 0.0
    step = 0

    if display:
        with mujoco.viewer.launch_passive(model, data) as viewer:

            # set initial viewer location
            viewer.cam.lookat = [0., 0., 0.]
            viewer.cam.distance = 1.0
            viewer.cam.azimuth = 0.
            viewer.cam.elevation = -30.0
            
            while viewer.is_running() and data.time < end:
                simulation_step()
                viewer.sync()

                # sleep only for the remaining time, to sync the physics with real time on this machine
                target_time += dt
                remaining = target_time - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
    else:
        # headless sim mode
        while data.time < end:
            simulation_step()

    # outputs
    x_final_dist = data.qpos[0] - x_start_dist
    y_final_dist = data.qpos[1] - y_start_dist

    if printy:
        # functional outputs, in meters and radians
        # print("x", round(x_final_dist, 2))
        # print("y", round(y_final_dist, 2))
        # print("r", round(total_rotation, 2))

        # functional outputs, in centimeters and degrees
        print("x", round(np.sqrt(x_final_dist**2 + y_final_dist**2)*100, 2))
        print("r", round(total_rotation*180/np.pi, 2))


        metrics = [x_final_dist, y_final_dist, total_rotation]
        print(*(round(x, 2) for x in [metrics[loss], x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, left, right, orient, loss]), sep=", ")

    return x_final_dist, y_final_dist, total_rotation


if __name__ == "__main__":
    u_s = 0.3

    # # excellent crab-like rotator (1, 2 -> r=5.64)
    # print("turn")
    bez_sim(0.24, 0.23, 0.87, 1.24, 0.1, 0.21, 2.82, 2.48, 0.22, 0.26, 2.37, 2.99, 1, 2, 1, 2, False, True, u_s)

    # # walker/rotator (1, 1 -> r=1.36)
    # print("walk")
    bez_sim(0.4, 0.4, 0.5, 0.3, 2.5, 2.8, 2.5, 2.5, 0.5, 1.4, 0.5, 0.5, 1, 1, 0, 2, False, True, u_s)

    # # diagonal crawler (3, 1 -> y=0.80)
    # print("run")
    bez_sim(0.56, 1.89, 0.8, 2.8, 0.1, 2.93, 0.25, 0.22, 2.8, 0.25, 0.8, 0.71, 3, 1, 1, 1, False, True, u_s) 

    # # gallop (4, 4 -> y=0.88)
    # print("gallop")
    bez_sim(0.2, 0.2, 2.2, 2.2, 0.9, 0.2, 1.8, 2.2, 0.2, 0.2, 0.2, 0.2, 4, 4, 1, 1, False, True, u_s)


# Friction Analysis

    # turn_d = np.array([])
    # turn_r = np.array([])
    # walk_d = np.array([])
    # walk_r = np.array([])
    # run_d = np.array([])
    # run_r = np.array([])
    # gallop_d = np.array([])
    # gallop_r = np.array([])
    
    # for i in range(1,21):
    #     u_s = float(i)/20.0

    #     tx, ty, tr = bez_sim(0.24, 0.23, 0.87, 1.24, 0.1, 0.21, 2.82, 2.48, 0.22, 0.26, 2.37, 2.99, 1, 2, 1, 2, False, False, u_s)
    #     turn_d = np.append(turn_d, np.sqrt(tx**2 + ty**2))
    #     turn_r = np.append(turn_r, tr)
    #     wx, wy, wr = bez_sim(0.4, 0.4, 0.5, 0.3, 2.5, 2.8, 2.5, 2.5, 0.5, 1.4, 0.5, 0.5, 1, 1, 0, 2, False, False, u_s)
    #     walk_d = np.append(walk_d, np.sqrt(wx**2 + wy**2))
    #     walk_r = np.append(walk_r, wr)
    #     rx, ry, rr = bez_sim(0.56, 1.89, 0.8, 2.8, 0.1, 2.93, 0.25, 0.22, 2.8, 0.25, 0.8, 0.71, 3, 1, 1, 1, False, False, u_s) 
    #     run_d = np.append(run_d, np.sqrt(rx**2 + ry**2))
    #     run_r = np.append(run_r, rr)
    #     gx, gy, gr = bez_sim(0.2, 0.2, 2.2, 2.2, 0.9, 0.2, 1.8, 2.2, 0.2, 0.2, 0.2, 0.2, 4, 4, 1, 1, False, False, u_s)
    #     gallop_d = np.append(gallop_d, np.sqrt(gx**2 + gy**2))
    #     gallop_r = np.append(gallop_r, gr)
    #     print(i)
    
    # import matplotlib.pyplot as plt

    # f_vals = (np.arange(20) + 1) / 20.0
    # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # # scaling, m->cm, rad->deg
    # turn_d *= 100
    # walk_d *= 100
    # run_d *= 100
    # gallop_d *= 100

    # turn_r *= 180 / np.pi
    # walk_r *= 180 / np.pi
    # run_r *= 180 / np.pi
    # gallop_r *= 180 / np.pi

    # ax1.plot(f_vals, turn_d, marker='o', label='Turn')
    # ax1.plot(f_vals, walk_d, marker='s', label='Walk')
    # ax1.plot(f_vals, run_d, marker='^', label='Run')
    # ax1.plot(f_vals, gallop_d, marker='d', label='Gallop')

    # ax1.set_title("Distance vs. Sliding Friction")
    # ax1.set_xlabel(r"$\mu_s$")
    # ax1.set_ylabel("Distance (cm)")
    # ax1.grid(True, alpha=0.3)
    # ax1.legend(loc='center right')

    # ax2.plot(f_vals, turn_r, marker='o', label='Turn')
    # ax2.plot(f_vals, walk_r, marker='s', label='Walk')
    # ax2.plot(f_vals, run_r, marker='^', label='Run')
    # ax2.plot(f_vals, gallop_r, marker='d', label='Gallop')

    # ax2.set_title("Rotation vs. Sliding Friction")
    # ax2.set_xlabel(r"$\mu_s$")
    # ax2.set_ylabel("Rotation (deg)")
    # ax2.grid(True, alpha=0.3)
    # ax2.legend(loc='center right')

    # plt.tight_layout()
    # plt.show()