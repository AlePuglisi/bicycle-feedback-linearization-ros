#include "car_traj_control/car_traj_control.h"


void car_traj_control::Prepare(void)
{
    // Get parameters from ROS parameter server
    std::string FullParamName;

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/car_model";
    if (false == Handle.getParam(FullParamName, car_model))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/amplitude";
    if (false == Handle.getParam(FullParamName, amplitude))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle parameters
    FullParamName = ros::this_node::getName()+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/L";
    if (false == Handle.getParam(FullParamName, L))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

        
    // Controller Gains
    FullParamName = ros::this_node::getName()+"/Kpx";
    if (false == Handle.getParam(FullParamName, Kpx))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpy";
    if (false == Handle.getParam(FullParamName, Kpy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Controller integral time constants
        FullParamName = ros::this_node::getName()+"/Tix";
    if (false == Handle.getParam(FullParamName, Tix))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Tiy";
    if (false == Handle.getParam(FullParamName, Tiy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Ts";
    if (false == Handle.getParam(FullParamName, Ts))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // fblin parameter
    FullParamName = ros::this_node::getName()+"/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // initial controller internal states
    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/theta0";
    if (false == Handle.getParam(FullParamName, theta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    /* ROS topics */
    command_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/cmd", 1);
    state_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/control_state", 1);

    state_subscriber = Handle.subscribe("/state", 1, &car_traj_control::state_MessageCallback, this);

    // reduce T when Fiala tyre model 
    if (car_model != 0)
        T = 0.75*T;

    // Initialize node state
    RunPeriod = Ts; 
    ex = 0.0;
    ey = 0.0;

    fblin = new car_fblin(P_dist, L);
    fblin->setState(x0,y0,theta0);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_traj_control::state_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // set current state of position P, feedback information
    fblin->setState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void car_traj_control::RunPeriodically(float Period)
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

void car_traj_control::Shutdown(void)
{
    delete fblin; // delete instance of fblinearizer

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_traj_control::PeriodicTask(void)
{

    // Trajectory computation for reference P point, 8 shape
    xref    = amplitude*std::sin(T*ros::Time::now().toSec());
    dxref   = T*amplitude*std::cos(T*ros::Time::now().toSec());
    yref    = amplitude*std::sin(T*ros::Time::now().toSec())*std::cos(T*ros::Time::now().toSec());
    dyref   = T*amplitude*(std::pow(std::cos(T*ros::Time::now().toSec()),2.0)-std::pow(std::sin(T*ros::Time::now().toSec()),2.0));

    // compute control action
    fblin->referenceTransform(xref,yref,xpref,ypref);
    fblin->stateTransform(xp,yp);

    /* Vehicle commands in terms of point P speed*/
    // as Proportional Integral + feedforeward action 
    
    ex += (xpref-xp); // cumulative error x
    ey += (ypref-yp); // cumulative error y

    Vxp = (xpref-xp)*Kpx + (Ts*ex)*Kpx/Tix + dxref; 
    Vyp = (ypref-yp)*Kpy + (Ts*ey)*Kpy/Tiy + dyref;

    //Vxp = (xpref-xp)*Kpx +  dxref;  only proportional 
    //Vyp = (ypref-yp)*Kpy +  dyref;

    /* TEST fblin correctness */
    /*
    if(ros::Time::now().toSec()<=5.0)
    {
        Vxp = 1.0;
        Vyp = 0.0;
    }
    else if(ros::Time::now().toSec()<=10.0)
    {
        Vxp = 2.0;
        Vyp = 0.0;
    }
    else{
        Vxp = 2.0;
        Vyp = 2.0;
    }
    */
   
    fblin->commandTransform(Vxp, Vyp, V, phi);
    /* TEST disturbances */
    /*
    if(ros::Time::now().toSec()>=5.0 && ros::Time::now().toSec()<=10.0)
    {   
          // set up random generator
        std::mt19937 generator(rd());
        std::normal_distribution<double> distribution(0.0, 0.5);
        double white_noise = distribution(generator);
        V += white_noise;
        phi += white_noise*white_noise;
    }
    */

    // compute traking errors
    double e_xp, e_yp;
    e_xp = xpref-xp;
    e_yp = ypref-yp;

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray ctrl_msg;
    ctrl_msg.data.push_back(ros::Time::now().toSec());
    
    ctrl_msg.data.push_back(V);
    ctrl_msg.data.push_back(phi);

    ctrl_msg.data.push_back(Vxp);
    ctrl_msg.data.push_back(Vyp);
    
    command_publisher.publish(ctrl_msg);

    // controller state 
    std_msgs::Float64MultiArray state_msg;
    state_msg.data.push_back(ros::Time::now().toSec());
    state_msg.data.push_back(xref);
    state_msg.data.push_back(yref);

    state_msg.data.push_back(xpref);
    state_msg.data.push_back(ypref);

    state_msg.data.push_back(xp);
    state_msg.data.push_back(yp);

    state_msg.data.push_back(e_xp);
    state_msg.data.push_back(e_yp);
    
    state_publisher.publish(state_msg);

}