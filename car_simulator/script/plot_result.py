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

vehicle_ForceFront = []
vehicle_ForceRear = []

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

bag.close()

# Plot data

#CAR TRAJECTORY
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y,'b')
plt.plot(vehicleState_x[0],vehicleState_y[0],'bo')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'bx')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("CAR TRAJECTORY")

# CAR COMMAND
plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_steer,"r")
plt.xlabel("Time [s]")
plt.ylabel("Steer act [rad]")
plt.title("CAR COMMANDS")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_velocity, "r")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")

# CAR STATE
plt.figure(3)
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
    plt.figure(4)
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
