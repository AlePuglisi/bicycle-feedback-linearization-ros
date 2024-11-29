#include "car_simulator/car_simulator_dyn.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_simulator_dyn");
  
  car_simulator_dyn sim_dyn_node;
   
  sim_dyn_node.Prepare();
  
  sim_dyn_node.RunPeriodically();
  
  sim_dyn_node.Shutdown();
  
  return (0);
}