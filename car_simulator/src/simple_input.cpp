#include "car_simulator/simple_input.h"

void simple_input::Prepare(void)
{
    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);

    /* Initialize node state */
    RunPeriod = RUN_PERIOD_DEFAULT;

    steer = speed = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void simple_input::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.3fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void simple_input::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void simple_input::PeriodicTask(void)
{
    /* Vehicle commands */
    if (ros::Time::now().toSec()<=5.0)
    {
        speed = 1.0;
        steer = 0.0;
    }
    else if (ros::Time::now().toSec()>5.0 && ros::Time::now().toSec()<= 10.0)
    {
        speed = 1.0;
        steer = 0.3;
    }
    else
    {
        speed = 1.0;
        steer = 0.0;
    }

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(speed);
    msg.data.push_back(steer);
    vehicleCommand_publisher.publish(msg);
}
