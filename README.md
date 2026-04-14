# Operational Space Inverse Dynamics Control for a 3-DOF Robot

This repository contains the implementation codes for the bachelor's thesis: *"Diseño y sintonización de un controlador por dinámica inversa en el espacio operacional para la precisión del seguimiento de trayectorias planificadas de un robot de tres grados de libertad"* [Design and tuning of an operational space inverse dynamics controller for precise planned trajectory tracking of a three-degree-of-freedom robot].

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
* **`main_circle_1.m`**: Main telemetry script. It handles the initiation of the control system, command sending, real-time data reception, and process termination. It must be run to inizializate the control of the Tracking_circle_1.ino.
* **`processRealData.m`**: Processes the raw data obtained from telemetry and performs comparative calculations of the control algorithm. It prepares structured data for visualization.
* **`Graphics.m`**: Generates the result plots based on the processed data.
--------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------
### Folder: `Simulation_Matlab`

This folder contains the MATLAB simulation codes for the inverse dynamics control applied to the planned trajectory tracking.

#### Main Simulation Scripts
* **`Tracking_circle_XYZ_1.m` / `Tracking_circle_XYZ_2.m`**: Main simulation scripts for tracking the circular trajectories.
* **`Tracking_rectangle_XYZ_1.m` / `Tracking_rectangle_XYZ_2.m`**: Main simulation scripts for tracking the rectangular trajectories.

#### Control and Modeling Classes
* **`CIK_Robot3DOF.m`**: Class implementing the Inverse Dynamics Control algorithm.
* **`RobotCinematica.m`**: Class implementing the kinematic characterization of the robot.
* **`RobotDinamica.m`**: Class implementing the algorithms used to obtain the robot's dynamic equations.
* **`load_parameters.m`**: Script containing the physical and design parameters of the manipulator.

#### Trajectory Planning
* **`TrajectoryPlanningCircle.m`**: Class implementing the circular trajectory planning algorithm.
* **`TrajectoryPlanningRectangle.m`**: Class implementing the rectangular trajectory planning algorithm.

#### Visualization and Analysis
* **`RobotVisualization.m`**: Class implementing the 3D graphical simulation of the robot in motion.
* **`Graphics.m`**: Script responsible for generating the control and trajectory planning performance plots.
