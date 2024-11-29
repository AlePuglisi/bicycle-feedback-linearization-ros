import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# check if dyn model 
dyn = False 

# State published by the simulator
vehicleState_time = []

vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []

vehicleState_steer = []
vehicleState_velocity = []

vehicleState_phisicalsteer = []
vehicleState_phisicalvelocity = []

vehicle_ForceFront = []
vehicle_ForceRear = []

control_time = []

desired_traj_x = []
desired_traj_y = []

desired_traj_xp = []
desired_traj_yp = []

vehicleState_xp = []
vehicleState_yp = []

tracking_error_x = []
tracking_error_y = []


for topic, msg, t in bag.read_messages():
    if topic == "/state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_velocity.append(msg.data[4])
        vehicleState_steer.append(msg.data[5])

        if len(msg.data) > 6:
            dyn = True
            vehicle_ForceFront.append(msg.data[6])
            vehicle_ForceRear.append(msg.data[7])
        
    if topic == "/control_state":
        control_time.append(msg.data[0])

        desired_traj_x.append(msg.data[1])
        desired_traj_y.append(msg.data[2])

        desired_traj_xp.append(msg.data[3])
        desired_traj_yp.append(msg.data[4])

        vehicleState_xp.append(msg.data[5])
        vehicleState_yp.append(msg.data[6])

        tracking_error_x.append(msg.data[7])
        tracking_error_y.append(msg.data[8])
        

bag.close()

# Plot data

# TRAJECTORY
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y,'b',label="real")
plt.plot(desired_traj_x,desired_traj_y,'r--',label="desired")
plt.plot(vehicleState_x[0],vehicleState_y[0],'bo')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'bx')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.title("CAR DESIRED TRAJECTORY")

# CONTROL ACTION
plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_steer,"r")
plt.xlabel("Time [s]")
plt.ylabel("Steer act [rad]")
plt.title("CAR COMMAND")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_velocity, "r")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")

# TRACKING ERRORS
ex_max = max(tracking_error_x)
ey_max = max(tracking_error_y)

plt.figure(3)
plt.subplot(121)
plt.plot(control_time,tracking_error_x,"r", label="error_xp")
plt.xlabel("Time [s]")
plt.ylabel("x error [m]")
plt.legend()
plt.text(5,0,"max e_x =" + str(ex_max), bbox=dict(facecolor='red', alpha=0.5), fontsize=25)
plt.title("TRACKING ERROR")
plt.subplot(122)
plt.plot(control_time,tracking_error_y,"r", label="error_yp")
plt.xlabel("Time [s]")
plt.ylabel("y error [m]")
plt.legend()
plt.text(5,0,"max e_y =" + str(ey_max), bbox=dict(facecolor='red', alpha=0.5), fontsize=25)

# CAR STATES
plt.figure(4)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x)
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.title("CAR STATES")
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y)
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")

# TYRE FORCE
if dyn:
    plt.figure(5)
    plt.subplot(211)
    plt.plot(vehicleState_time,vehicle_ForceFront,"r")
    plt.xlabel("Time [s]")
    plt.ylabel("Front tyre Force [N]")
    plt.title("CAR TYRE FORCE")
    plt.subplot(212)
    plt.plot(vehicleState_time,vehicle_ForceRear, "r")
    plt.xlabel("Time [s]")
    plt.ylabel("Rear tyre Force [N]")

plt.show()
