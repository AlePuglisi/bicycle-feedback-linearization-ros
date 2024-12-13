# bicycle-feedback-linearization-ros

This repository contains both the code and the report of the  final assignment of a course on [control of mobile robots](https://www11.ceda.polimi.it/schedaincarico/schedaincarico/controller/scheda_pubblica/SchedaPublic.do?&evn_default=evento&c_classe=837146&lang=IT&__pj0=0&__pj1=8379d6c35eccfe1c998db9b2de7c0e1c).

Take a look at [assignment](https://github.com/AlePuglisi/bicycle-feedback-linearization-ros/blob/main/assignment.pdf) for more details on the task.

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


## Packages description

## Run the Code

## Conclusion







