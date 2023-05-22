#include "car_traj_ctrl/car_kin_PI.h"

#include <unistd.h>


void car_kin_PI::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Controller parameters
    FullParamName = ros::this_node::getName()+"/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/a";
    if (false == Handle.getParam(FullParamName, a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpx";
    if (false == Handle.getParam(FullParamName, Kpx))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/Kpy";
    if (false == Handle.getParam(FullParamName, Kpy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Ipx";
    if (false == Handle.getParam(FullParamName, Ipx))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Ipy";
    if (false == Handle.getParam(FullParamName, Ipy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleState_subscriber = Handle.subscribe("/car_state", 1, &car_kin_PI::vehicleState_MessageCallback, this);
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);
    refTrajectory_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/ref_traj", 1);


    /* Create controller class */
    controller = new car_kin_fblin(P_dist);

    // Initialize controller parameters
    controller->set_carParam(l);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_kin_PI::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void car_kin_PI::Shutdown(void)
{
    // Delete controller object
    delete controller;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_kin_PI::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    /*  Set vehicle state */
    controller->set_carState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void car_kin_PI::PeriodicTask(void)
{
    /*  Generate trajectory */
    double xref, yref;

    double xPref, yPref;
    double vPx, vPy;
    const double pi = 3.14159265358979323846;
    const double t = ros::Time::now().toSec();

    xref = a*std::sin((2*pi/T)*t);
    yref = a*std::sin((2*pi/T)*t)*std::cos((2*pi/T)*t);

    /*  Tranforms in P  */
    controller->output_transformation(xP, yP);    
    controller->reference_transformation(xref, yref, xPref, yPref);

    //ROS_INFO("xP: %f, yP: %f, xPref: %f, yPref: %f", xP,yP,xPref,yPref);

    /*  Generate input commands, P Controller*/
    //TODO: add feed-forward term, add integral term
    vPx = Kpx*(xPref - xP);
    vPy = Kpy*(yPref - yP);

    //ROS_INFO("xP error: %.2f, yP error: %.2f", xPref - xP, xPref - xP );
    /*  Compute the control action */
    double v, phi;
    controller->control_transformation(vPx, vPy, v, phi);

    double time;
    time = ros::Time::now().toSec();

    /*  Publish vehicle commands */
    std_msgs::Float64MultiArray vehicleCommandMsg;
    vehicleCommandMsg.data.push_back(time);
    vehicleCommandMsg.data.push_back(v);
    vehicleCommandMsg.data.push_back(phi);
    vehicleCommand_publisher.publish(vehicleCommandMsg);

    /*  Publish controller state */
    std_msgs::Float64MultiArray controllerStateMsg;
    controllerStateMsg.data.push_back(time);
    controllerStateMsg.data.push_back(vPx);
    controllerStateMsg.data.push_back(vPy);
    controllerStateMsg.data.push_back(v);
    controllerStateMsg.data.push_back(phi);
    controllerState_publisher.publish(controllerStateMsg);

    /*  Publish reference trajectory */
    std_msgs::Float64MultiArray refTrajectoryMsg;
    refTrajectoryMsg.data.push_back(time);
    refTrajectoryMsg.data.push_back(xref);
    refTrajectoryMsg.data.push_back(yref);
    refTrajectory_publisher.publish(refTrajectoryMsg);

}
