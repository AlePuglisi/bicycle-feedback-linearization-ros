#include "car_simulator/car_test.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_test");
  
  car_test car_test_node;
   
  car_test_node.Prepare();
  
  car_test_node.RunPeriodically(car_test_node.RunPeriod);
  
  car_test_node.Shutdown();
  
  return (0);
}