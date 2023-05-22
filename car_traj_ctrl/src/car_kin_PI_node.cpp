#include "car_traj_ctrl/car_kin_PI.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_kin_PI car_kin_PI_node;
   
  car_kin_PI_node.Prepare();
  
  car_kin_PI_node.RunPeriodically(car_kin_PI_node.RunPeriod);
  
  car_kin_PI_node.Shutdown();
  
  return (0);
}

