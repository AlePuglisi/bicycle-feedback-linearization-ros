#include "car_simulator/car_simulator_dyn.h"

#include <unistd.h>

void car_simulator_dyn::Prepare(void)
{
    // Get parameters from ROS parameter server
    std::string FullParamName;

    // car dynamical model 
    FullParamName = ros::this_node::getName()+"/car_model";
    if (false == Handle.getParam(FullParamName, car_model))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle parameters
    FullParamName = ros::this_node::getName()+"/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/b";
    if (false == Handle.getParam(FullParamName, b))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/m";
    if (false == Handle.getParam(FullParamName, m))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/mu";
    if (false == Handle.getParam(FullParamName, mu))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Cf";
    if (false == Handle.getParam(FullParamName, Cf))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Cr";
    if (false == Handle.getParam(FullParamName, Cr))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Iz";
    if (false == Handle.getParam(FullParamName, Iz))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle initial state
    FullParamName = ros::this_node::getName()+"/r0";
    if (false == Handle.getParam(FullParamName, r0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Beta0";
    if (false == Handle.getParam(FullParamName, Beta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/psi0";
    if (false == Handle.getParam(FullParamName, psi0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    

    // ROS Topics definition
    command_subscriber = Handle.subscribe("/cmd", 1, &car_simulator_dyn::command_MessageCallback, this);
    state_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    
    /* Create car_simulator_dyn class */
    switch (car_model)
    {
        case 0: // linears
            sim = new car_simulator_dyn_ode(dt, car_simulator_dyn_ode::LINEAR);
            break;

        case 1: // fiala with saturation
            sim = new car_simulator_dyn_ode(dt, car_simulator_dyn_ode::FIALA_SATURATION);
            break;

        case 2: // fiala without saturation
            sim = new car_simulator_dyn_ode(dt, car_simulator_dyn_ode::FIALA_NO_SATURATION);
            break;

        default:
            sim = new car_simulator_dyn_ode(dt, car_simulator_dyn_ode::LINEAR);
            break;
    }

    // Initialize the car_simulator_dyn
    sim->setInitialState(r0, Beta0, x0, y0, psi0);
    sim->setVehicleParams(m, a, b, Cf, Cr, mu, Iz);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str()); // Notification
}

void car_simulator_dyn::RunPeriodically(void)
{
     ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        usleep(1000);
    }
}

void car_simulator_dyn::Shutdown(void)
{
    // Delete ode object
    delete sim;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_simulator_dyn::command_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]
    // Set vehicle commands 
    sim->setReferenceCommand(msg->data.at(1), msg->data.at(2)); 
}

void car_simulator_dyn::PeriodicTask(void)
{
    /*  Integrate the model */
    sim->integrate();

    /*  Extract measurement from car_simulator_dyn */
    double x, y, theta;
    sim->getPose(x, y, theta);

    double ay, yawrate, vy;
    sim->getLateralDynamics(ay, yawrate, vy);

    double sideslip;
    sim->getSideSlip(sideslip);

    double slip_front, slip_rear;
    sim->getSlip(slip_front, slip_rear);

    double force_front, force_rear;
    sim->getLateralForce(force_front, force_rear);

    double velocity_act, steer_act;
    sim->getCommand(velocity_act, steer_act);

    double time;
    sim->getTime(time);

    /*  Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    /*  Publish vehicle state */
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(velocity_act);
    vehicleStateMsg.data.push_back(steer_act);
    vehicleStateMsg.data.push_back(force_front);
    vehicleStateMsg.data.push_back(force_rear);

    vehicleStateMsg.data.push_back(yawrate);
    vehicleStateMsg.data.push_back(vy);
    vehicleStateMsg.data.push_back(ay);
    vehicleStateMsg.data.push_back(sideslip);
    vehicleStateMsg.data.push_back(slip_front);
    vehicleStateMsg.data.push_back(slip_rear);
    state_publisher.publish(vehicleStateMsg);

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}