# bicycle-feedback-linearization-ros

This repository contains both the code and the report of the  final assignment of a course on [control of mobile robots](https://www11.ceda.polimi.it/schedaincarico/schedaincarico/controller/scheda_pubblica/SchedaPublic.do?&evn_default=evento&c_classe=837146&lang=IT&__pj0=0&__pj1=8379d6c35eccfe1c998db9b2de7c0e1c).

## Introduction 
In this project, I developed a trajectory-tracking controller for a car-like robot, modeled with the bicycle model.<br/>

<image width=300 height=300 src=https://github.com/user-attachments/assets/24161273-1794-4659-938e-fbd22894d791>
<image width=700 height=300 src=https://github.com/user-attachments/assets/235a15a2-6b35-4652-baa5-c94485eeee07>

Both kinematic and dynamic (linear and fiala tyre) models are analyzed. <br/>
The control scheme is based on an inner feedback linearization (based on the kinematic model), transforming the system model to the canonical unicycle.</br> 
Then an outer x and y PI independent velocity controllers regulate the system position, to track the reference trajectory. 

<image width=700 height=300 src=https://github.com/user-attachments/assets/1b9c50da-7dd6-49b0-9fb3-81b9586a44d3>

For further details on the model equations and parameters, on the reference trajectory and on the control scheme, refer to my [report](https://github.com/AlePuglisi/bicycle-feedback-linearization-ros/blob/main/Trajectory_Tracking_Report.pdf)

## Objective
Define bicycle model kinematic and dynamic equations, and implement a ROS-based simulation and control software. <br/>
Use it to tune the PI trajectory tracking controllers and analyze the performance. <br/>
(Notice that the feedback linearization law is based on the kinematic model, for the dynamic model, a perfect linearization is not possible in this way, and performances are not the best)

In this course, we focus on control and modeling, so instead of using classical simulators like Gazebo, we use C++ library odeint for explicit differential equation resolution. 


Look at the [assignment](https://github.com/AlePuglisi/bicycle-feedback-linearization-ros/blob/main/assignment.pdf) for more details.

## Packages description

<image width=700 height=200 src=https://github.com/user-attachments/assets/6ad6011d-76cf-4787-b158-0272aa942465>
<br/>

The packages of this project are: 

- ``car_simulator``:<br/>
Responsible for kin/dyn model simulation.
- ``car_traj_control``:<br/>
Provide the control feedback linearization + PI control law. 

>[!NOTE]
> For a clean code, the executables are organized as follows:<br/>
> Executables of ``car_simulator`` are:
> - "simulator", with the ROS node functions implementation
> - "simulator"_ode, with the modeling and odeint related functions
> - "simulator"_node, just initialize and run the node<br/>
>
> Executables of ``car_traj_control`` are:<br/>
> - car_fblin, implement feedback linearization
> - car_traj_control, ROS node functions implementation and control law computation
> - car_traj_control_node, just initialize and run the node


## Run the Code
First, clone this repo in your ros workspace 
```
git clone https://github.com/AlePuglisi/bicycle-feedback-linearization-ros.git
```
build and source your workspace, and you are ready to use it!

```
# Terminal 0, always remember roscore!
roscore
```

- Simulation test:
  
- Control and simulation:
  
- Performance analysis and plot:
  

## Conclusion







