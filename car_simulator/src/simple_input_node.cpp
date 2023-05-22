#include "car_simulator/simple_input.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  simple_input simple_input_node;
   
  simple_input_node.Prepare();
  
  simple_input_node.RunPeriodically(simple_input_node.RunPeriod);
  
  simple_input_node.Shutdown();
  
  return (0);
}