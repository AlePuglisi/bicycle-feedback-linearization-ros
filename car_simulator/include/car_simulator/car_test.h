#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>


class car_test
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher command_publisher;
    
    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double speed, steer;

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};