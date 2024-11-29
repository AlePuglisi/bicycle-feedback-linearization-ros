#include  "car_simulator/car_simulator_kin.h"

#include <unistd.h>

void car_simulator_kin::Prepare(void)
{
    // Get parameters from ROS parameter server
    std::string FullParamName;

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle parameters
    FullParamName = ros::this_node::getName()+"/L";
    if (false == Handle.getParam(FullParamName, L))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle initial state
    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta0";
    if (false == Handle.getParam(FullParamName, theta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());



    // ROS Topics definition
    command_subscriber = Handle.subscribe("/cmd", 1,  &car_simulator_kin::command_MessageCallback, this);
    state_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    // Create car_simulator_kin class
    sim = new car_simulator_kin_ode(dt);

    // Initialize the car_simulator_kin
    sim->setInitialState(x0, y0, theta0);
    sim->setVehicleParams(L);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str()); // Notification
}

void car_simulator_kin::RunPeriodically(void)
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

void car_simulator_kin::Shutdown(void)
{
    // Delete ode object
    delete sim;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_simulator_kin::command_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]
    // Set vehicle commands 
    sim->setReferenceCommand(msg->data.at(1), msg->data.at(2)); 
}

void car_simulator_kin::PeriodicTask(void)
{
    /*  Integrate the model */
    sim->integrate();

    /*  Extract measurement from car_simulator_kin */
    double x, y, theta;
    sim->getPose(x, y, theta);

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

    state_publisher.publish(vehicleStateMsg);

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}