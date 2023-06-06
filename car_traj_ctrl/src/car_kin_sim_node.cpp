#include "car_traj_ctrl/car_kin_sim.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_kin_sim car_kin_sim_node;
   
  car_kin_sim_node.Prepare();
  
  car_kin_sim_node.RunPeriodically();
  
  car_kin_sim_node.Shutdown();
  
  return (0);
}