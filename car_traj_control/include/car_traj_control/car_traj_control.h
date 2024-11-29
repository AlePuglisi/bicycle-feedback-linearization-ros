#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include "car_traj_control/car_fblin.h"
#include <random>

class car_traj_control
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher command_publisher, state_publisher;
    ros::Subscriber state_subscriber;

    // Trajectory parameters
    double amplitude, T, L;
    // fblin parameter
    double P_dist;
    
    // Node periodic task 
    void PeriodicTask(void);

    /* Node state variables */
    int car_model;
    double x0, y0, theta0;
    double xref, yref, dxref, dyref;
    double xp, yp, xpref, ypref; // current point position
    double Vxp, Vyp, V, phi; // commands

     /* Proportional gains of controllers */
    double Kpx, Kpy;
    double Tix, Tiy;
    double ex, ey;
    double Ts;

    car_fblin* fblin; //fb linearizer controller

    std::random_device rd; // random device to test system against disturbance

     // ROS topic Callback function 
    void state_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr&msg);

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};