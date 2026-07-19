# Overview

There are 12 cells in this robot, six male (containing motors) and six female (containing other components).
These cells are numbered following this figure:

<img width="1281" height="1012" alt="Screenshot 2026-06-26 141248" src="https://github.com/user-attachments/assets/f4fd5201-110d-4403-ac7f-824cf88b5f1d" />


Each cell has hinges that must be glued into place for assembly, and all six female cells have gear inserts required for motion. Hinges and hinge types vary based on location, type of cell, and whether or not the hinge borders a short edge (two of which exist on each cell) or a long edge (one of which exists on each cell). The two short edges are next to each other on one side of the cell, while the long edge sits opposite them. All hinges can reasonably be installed using cyanoacrylate glue.

## Female Cells

There are three variants of female cells in this robot: the standard female cell (`Female Cell.stl`), which is used for cells 3, 6, 8, and 12, the control module female cell (`Female Cell - Microcontroller.stl`), designed to house the microcontroller and power switch, which is used for Cell 2, and the power module female cell (`Female Cell Battery.stl`), which was designed to house batteries and buck converters, and which is used as the connection point for external power. It is used for Cell 9.

Each of the female cells uses `Female Hinge 1.stl`'s to connect to male cells bordering the short edge, and `Female Hinge 2.stl`'s to connect to male cells bordering the long edge.

In between all hinges, the female cells have an L-shaped slot that accepts a gear arm. These gear arms are used to actuate the hinge while the servo rotates.

### Electrical Components

Female Cell 2 houses the microcontroller (an ESP32 Firebeetle 2), a switch, and a battery (a 4-cell, NM 4.8V 200mAh battery).
Female Cell 9 houses batteries, buck converters, and a MOSFET for limiting power. Using electrical tape to hold different components in position is advised.
All other female cells contain PCBs for daisy-chaining power and signal data through the robot. 8-wire JST cables are used for signal transmission, and XT30 cables are used for power transmission.  

### Gear Arms

Like the gears, there are three variants of gear arms. Eight `Gear Arm Standard.stl`'s should be printed, with four being reflections of the other four. These are to be inserted in female hinges bordering short edge hinges. Four `Gear Arm Center.stl`'s should be printed and inserted in female hinges bordering long edge hinges. A final, single `Gear Arm S3170G.stl` should be inserted into the Cell 9, which is a `Female Cell Battery.stl`.

## Male Cells

There are two variants of male cells in this robot: the standard male cell (`Male Cell.stl`), which is used for cells 1, 5, 7, 10, and 11, and the modified male cell (`Male Cell - Modified (S1370G).stl`) designed to fit the Futaba SG3107 servo motor, which is used for Cell 4.

Each of the standard male cells uses one `Male Hinge 1.stl` and one `Male Hinge 2.stl` to connect to a female hinge along the short edge, and two `Male Hinge 2.stl`'s to connect to a female cell along the short edge. These hinges differ only slightly to accommodate the different geometry on different sides of the cell.

The modified male cell (Cell 4) uses modified versions of the male hinges to accommodate its altered geometry. It uses two `Male Hinge 1.stl`'s to connect to Cell 9, and two `Male Hinge 3.stl`'s and two Male Hinge 4's to connect to cells 2 and 6 (one hinge of each type connects to each of the neighboring cells.)

### Servos

Once hinges are installed, motors can be inserted. All motors are Miuzei MG90S's save for the one Futaba S3170G in Cell 4. Follow the numbered figure above for placements. The motors are all secured by a press fit, and all servo motor heads should be as close to their respective hinges as possible (as in, do not insert the servos backwards. Otherwise, the gears will be unable to mesh). Gears should not be attached to servos at this point.

## Pattern Assembly

Once all cells have had hinges glued, electrical components inserted, and the female hinges have gear arms placed, the cells can be arranged following the attached diagram. All hinges are designed to work with 3mm metal pins, which are press fit into each of the from the side. After hinges are functional, signal and power cables can be routed through cells from one component to the next. Signal wires originate at the microcontroller, with specific pins for each motor defined inside of CMR_ORIGAMI_ROBOT.cpp (line 23).

### Servo Calibration Run

Once all electronics are assembled, the buck converters should be powered with a voltage of 7.4 volts, which they will step down to the 6.0V required by the servo motors. Then the microcontroller should be powered on by its 4.8V battery, and allowed to run all motors through one run of a gait. The included `CMR_ORIGAMI_ROBOT_GAIT.cpp` will correctly position all servos at the end of the run, meaning that, so long as no hinges are actuated during assembly, gears may now be applie to the motors.

### Gear Insertion

There are three gear variants used in this model, to match the three gear arm variants. Therefore, eight `Short Edge Gear.stl`'s will be used on all short edge hinges, four `Long Edge Gear.stl`'s will be used on the four long edges, and one `S1370G Gear.stl` will be used on the centermost hinge, between Cell 4 (`Male Cell - Modified (S1370G).stl`) and Cell 9 (`Female Cell Battery.stl`).

To correctly insert all gears, it can be helpful to have a thin metal shim or spatula, and a thin pair of tweezers. It is also helpful to use a few rubber bands to hold the entire robot assembly in the closed position. Using the shim to apply a force through the gear towards the servo, slide each gear over the head of its respective servo motor, and once it gets close, press hard to ensure it slides into position. Further care should be taken to ensure that the teeth of the gear mesh with the teeth of the adjacent gear arm.

Once all gears are in position any rubber bands can be removed and the robot is ready for motion.
