#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

#include "car_simulator_kin_ode.h"

class car_simulator_kin
{
    private:
     ros::NodeHandle Handle;

     // Ros Topics
     ros::Subscriber command_subscriber;
     ros::Publisher state_publisher;
     ros::Publisher clock_publisher;

     // Variables to store ROS paramter server
     double dt;
     double x0, y0, theta0, phi0;
     double L;

     // ROS topic Callback function 
     void command_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr&msg);

     // Node periodic task to execute
     void PeriodicTask(void);

     // Node state variables
     car_simulator_kin_ode* sim;

    public:
     
     void Prepare(void);

     void RunPeriodically(void);

     void Shutdown(void);

};

