#include "car_traj_ctrl/car_kin_linctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  car_kin_linctrl car_kin_linctrl_node;
   
  car_kin_linctrl_node.Prepare();
  
  car_kin_linctrl_node.RunPeriodically(car_kin_linctrl_node.RunPeriod);
  
  car_kin_linctrl_node.Shutdown();
  
  return (0);
}

