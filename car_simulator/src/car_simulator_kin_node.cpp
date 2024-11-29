#include "car_simulator/car_simulator_kin.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_simulator_kin");
  
  car_simulator_kin sim_kin_node;
   
  sim_kin_node.Prepare();
  
  sim_kin_node.RunPeriodically();
  
  sim_kin_node.Shutdown();
  
  return (0);
}