# Origami Robot

This is the code behind the paper "Thickness-Invariant Origami Robots for Untethered Locomotion".

There are two primary directories here: `robotics` and `simulation`. 

## robotics

<img width="732" height="776" alt="cad_model" src="https://github.com/user-attachments/assets/71d7d19f-49c7-4d10-85c8-51bdeea7cdaa" />

`robotics` contains everything relating to the physical robot. It has the Platformio code used to control the physical robot constructed, with the final version of the code residing in `CMR_ORIGAMI_ROBOT_GAIT.cpp`. It has all CAD files used in the construction of the robot, as well as the custom PCBs. It contains instructions for 3D printing and assembly of parts to recreate the configuration used in the paper (see the `README.md` in the folder). It also has all materials obtained during experimentation, namely, the statistical analysis of the gaits used during testing.

## simulation

<img width="1200" height="600" alt="maybe" src="https://github.com/user-attachments/assets/83a22328-e524-401d-b4a9-10f38fe53177" />

`simulation` contains everything relating to the MuJoCo simulation of the robot. It has three primary files worth inspecting.

`runner.py` is used to observe individual gait profiles, and see how they perform. It is called by `gait_finder.py` to obtain distance and rotation measurements for optimization.

`gait_finder.py` contains the optimization algorithm used to identify gaits for the robot. The `budget` and `steps` variables determine how long it will take to run, and the fidelity of the results provided.

`playground.py` is a simulator for exploring robot motion manually. It uses the number keys on the keyboard to select and control individual hinges of the robot.

See the `README.md` within `simulation` for more information about how to install MuJoCo and run the code.
