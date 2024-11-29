#include "car_traj_control/car_traj_control.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_traj_control");
  
  car_traj_control car_traj_control_node;
   
  car_traj_control_node.Prepare();
  
  car_traj_control_node.RunPeriodically(car_traj_control_node.RunPeriod);
  
  car_traj_control_node.Shutdown();
  
  return (0);
}