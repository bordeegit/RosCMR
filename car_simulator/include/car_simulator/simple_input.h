#ifndef SIMPLE_INPUT_H_
#define SIMPLE_INPUT_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#define RUN_PERIOD_DEFAULT 0.01

#define NAME_OF_THIS_NODE "simple_input"


class simple_input
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;
    
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

#endif /* simple_input_H_ */
