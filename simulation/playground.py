import mujoco
import mujoco.viewer
import keyboard     # if keyboard.is_pressed('w'):
import time

# Load model
with open(r'model\origami\origami_robot.xml', 'r') as file:
    xml = file.read()

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

data.qpos[0:3] = [0, 0, 0.1]
mujoco.mj_forward(model, data)

STEP = 0.004

# Key bindings: (joint index, delta per step)
key_bindings = {}

dict_ready = False
left = True
right = True

# launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # mode selection: some explanation is necessary
        # when you boot up, you get to pick one key from '2, 3, 4, 5' and one key from '6, 7, 8, 9'
        # the first one you pick from either set is locked in the moment you press it
        # then, '2, 3' handle the key for the first set, '6, 7' handle the key from the second, and '4, 5'
        # always handles the middle hinge, no matter what.
        # if you want to change your selection, press '-' once, then pick from your sets again.
        # you won't be able to move until you've pressed one button from either set, after a reset.

        if keyboard.is_pressed('2') and left:   # l_top
            left = False
            key_bindings['2'] = [(0, +STEP)]
            key_bindings['3'] = [(0, -STEP)]

        if keyboard.is_pressed('3') and left:   # r_top
            left = False
            key_bindings['2'] = [(1, +STEP)]
            key_bindings['3'] = [(1, -STEP)]

        if keyboard.is_pressed('4') and left:   # m_top
            left = False
            key_bindings['2'] = [(2, +STEP)]
            key_bindings['3'] = [(2, -STEP)]

        if keyboard.is_pressed('5') and left:   # all_top
            left = False
            key_bindings['2'] = [(0, +STEP), (1, +STEP), (2, +STEP)]
            key_bindings['3'] = [(0, -STEP), (1, -STEP), (2, -STEP)]

        if keyboard.is_pressed('6') and right:  # l_bottom
            right = False
            key_bindings['6'] = [(5, +STEP)]
            key_bindings['7'] = [(5, -STEP)]

        if keyboard.is_pressed('7') and right:  # r_bottom
            right = False
            key_bindings['6'] = [(4, +STEP)]
            key_bindings['7'] = [(4, -STEP)]

        if keyboard.is_pressed('8') and right:  # m_bottom
            right = False
            key_bindings['6'] = [(6, +STEP)]
            key_bindings['7'] = [(6, -STEP)]

        if keyboard.is_pressed('9') and right:  # all_bottom
            right = False
            key_bindings['6'] = [(4, +STEP), (5, +STEP), (6, +STEP)]
            key_bindings['7'] = [(4, -STEP), (5, -STEP), (6, -STEP)]

        if not dict_ready and not left and not right :
            key_bindings['4'] = [(3, +STEP)]    # center
            key_bindings['5'] = [(3, -STEP)] 
            dict_ready = True

        # reset selection
        if keyboard.is_pressed('-'):
            key_bindings = {}
            dict_ready = False
            left = True
            right = True

        if dict_ready:
            # check all keys each frame
            for key, bindings in key_bindings.items():
                if keyboard.is_pressed(key):
                    for joint_id, delta in bindings:
                        data.ctrl[joint_id] = max(0, min(3.14, data.ctrl[joint_id] + delta))

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)