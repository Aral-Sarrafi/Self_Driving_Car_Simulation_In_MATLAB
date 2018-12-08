# Self-Driving Car Simulation In MATLAB
This repository contains a series of self-driving car simulations in MATLAB. The simulations are mainly focused on Controlls, Sensor Fusion, State Estimation and Localization in the context of Self_Driving Cars.

**NOTE: Run the Main.m file in each of the the folders for the simulation to start.**

The code associated with each of simulations are categorized in different folders. The code is standalone and should run on any MATLAB version without requiring additional dependencies. The simulations are listed below:

# 1. Lane Keeping Assist System Simulation for an autonomous vehicle in MATLAB/SIMULINK
This project uses the principlas of computer vision and control to simulate a lane keeping assist system for self driving cars in simulink.
The computer vision toolbox in simulink is used to detect the lane lines, and a PID controller is utulized to drive the vehicle in between the lane lines.

<img src = "Tracking_Trajectory/Tracking.gif" align="center" width = "420" hight = "420"> <img src = "Tracking_Trajectory/Zoomed.gif" align="center" width = "420" hight = "420">

# 2. PID Controller Design for Tracking
<img src = "Tracking_Trajectory/Tracking.gif" align="center" width = "420" hight = "420"> <img src = "Tracking_Trajectory/Zoomed.gif" align="center" width = "420" hight = "420">


# 3. A Hybrid Automaton Design
The dynamics of the car is implemented and a PID controller drives the car towards the specified Goal while avoiding the obstacle in the map.
The dynamics of the car and the controller are all implanted in Car.m file using object-oriented programming in MATLAB. The main function runs the car model and plots the results to generate the GIF file for this simulation.

<img src = "Path Planning/car-heading-1.5708.gif" align="center" width = "420" hight = "420"> <img src = "Path Planning/car-heading0.gif" align="center" width = "420" hight = "420">
