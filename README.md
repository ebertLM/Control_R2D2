# Control_R2D2
Inverse dynamics control for trajectory tracking of a 3DoF manipulator

# Operational Space Inverse Dynamics Control for a 3-DOF Robot

This repository contains the implementation codes for the bachelor's thesis: *"Design and tuning of an operational space inverse dynamics controller for precise planned trajectory tracking of a three-degree-of-freedom robot"*.

## Project Structure

The codes are organized by the type of planned trajectory (circular and rectangular). Both types share the same implementation architecture. The structure is described below using the circular trajectory `Tracking_circle_1` as an example:

### Folder: `Control_Dinamica Inversa_ESP32/Tracking_circle_1`

#### Control and Implementation Files (ESP32)
* **`Tracking_circle_1.ino`**: Main ESP32 sketch. It contains the core project design, including the control loop and trajectory planning.
* **`CIK_Robot3DOF.h`**: Main class implementing the Inverse Dynamics Control algorithm.
* **`RobotCinematica.h`**: Methods for the characterization and calculation of the robot's kinematics.
* **`RobotDinamica.h`**: Methods for obtaining and calculating the equations of the robot's dynamic model.
* **`parameters.h`**: Definition of the manipulator's physical and design parameters.
* **`FuncionesAuxiliares.h`** / **`.cpp`**: Auxiliary functions for managing communication and data telemetry.

#### Telemetry and Analysis Files (MATLAB)
* **`main_circle_1.m`**: Main telemetry script. It handles the initiation of the control system, command sending, real-time data reception, and process termination.
* **`processRealData.m`**: Processes the raw data obtained from telemetry and performs comparative calculations of the control algorithm. It prepares structured data for visualization.
* **`Graphics.m`** (Assumed): Generates the result plots based on the processed data.
