#ifndef CAR_SIM_H_
#define CAR_SIM_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

#include "car_ode.h"

#define NAME_OF_THIS_NODE "car_sim"


class car_sim
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleCommand_subscriber;
    ros::Publisher vehicleState_publisher;
    ros::Publisher clock_publisher;

    /* Parameters from ROS parameter server */
    double dt;
    int tyre_model;
    double r0, beta0, x0, y0, psi0;
    double m, a, b, Cf, Cr, mu, Iz;

    /* ROS topic callbacks */
    void vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    car_ode* simulator;

  public:

    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};


#endif 