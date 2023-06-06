#ifndef CAR_KIN_PI_H_
#define CAR_KIN_PI_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <car_traj_ctrl/car_kin_fblin.h>

#include <dynamic_reconfigure/server.h>
#include <car_traj_ctrl/gainPIConfig.h>

#define NAME_OF_THIS_NODE "car_kin_PI"


class car_kin_PI
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleState_subscriber;
    ros::Publisher vehicleCommand_publisher, controllerState_publisher, refTrajectory_publisher;

    /* Dynamic Reconfigure*/
    dynamic_reconfigure::Server<car_traj_ctrl::gainPIConfig> server;
    dynamic_reconfigure::Server<car_traj_ctrl::gainPIConfig>::CallbackType f;

    /* Parameters from ROS parameter server */
    double P_dist, l;
    double a, T;
    double xP=0.0, yP=0.0;
    double xref, yref=0.0, dxref, dyref; 
    double Kpx, Kpy;
    double Tix, Tiy;
    double Ts;
    double err_xP, err_yP;
    double integral_x = 0, integral_y = 0; 
    double err_X_max = 0, err_Y_max = 0;

    /* ROS topic callbacks */
    void vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Dynamic Reconfigure callback*/
    void DynReconfigCallback(car_traj_ctrl::gainPIConfig &config, uint32_t level);

    /* Node periodic task */
    void PeriodicTask(void);

    /* Trajectory generation function*/  
    void trajectoryGeneration_eight(void);
    void trajectoryGeneration_step(void);
    void trajectoryGeneration_sin(void);

    /* Control*/
    void control_FFPI(double& xPref, double& yPref, double& vPx,double& vPy);
    void controlSaturation(double& v, double& phi);

    /* Node state variables */
    car_kin_fblin* controller;

  public:
    float RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* CAR_KIN_PI_H_ */
