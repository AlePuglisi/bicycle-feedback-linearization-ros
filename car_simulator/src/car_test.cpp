#include "car_simulator/car_test.h"

void car_test::Prepare(void)
{
    // Get parameters from ROS parameter server
    std::string FullParamName;

    /* ROS topics */
    command_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/cmd", 1);

    // Initialize node state
    // value of open loop controller input a little bit irrealistic, too short 
    // choose so fast to try to have a more accurate responce even in presence of small actuators delay
    RunPeriod = 0.001; 
    steer = speed = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_test::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void car_test::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_test::PeriodicTask(void)
{
     /* Vehicle commands */
    // simple step input test 
    if(ros::Time::now().toSec()<=5.0)
    {
        speed = 1.0;
        steer = 0.0;
    }
    else if(ros::Time::now().toSec()<=10.0)
    {
        speed = 2.0;
        steer = 0.0;
    }
    else{
        speed = 2.0;
        steer = 2.0;
    }

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(speed);
    msg.data.push_back(steer);
    
    command_publisher.publish(msg);
}