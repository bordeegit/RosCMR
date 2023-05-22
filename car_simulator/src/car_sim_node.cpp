#include "car_simulator/car_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_sim car_sim_node;
   
  car_sim_node.Prepare();
  
  car_sim_node.RunPeriodically();
  
  car_sim_node.Shutdown();
  
  return (0);
}