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

    FullParamName = ros::this_node::getName()+"/Tix";
    if (false == Handle.getParam(FullParamName, Tix))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Tiy";
    if (false == Handle.getParam(FullParamName, Tiy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Ts";
    if (false == Handle.getParam(FullParamName, Ts))
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

void car_kin_PI::trajectoryGeneration_eight(void){
    
    const double pi = 3.14159265358979323846;
    const double t = ros::Time::now().toSec();
    double w = 2*pi/T; 
    xref = a*std::sin(w*t);
    dxref = w*a*std::cos(w*t);
    yref = a*std::sin(w*t)*std::cos(w*t);
    dyref = w*a*(std::pow(std::cos(w*t),2.0)-std::pow(std::sin(w*t),2.0));
    //dxref = 0;
    //dyref = 0;

    //TODO: can we assume that velocity of P is the same as the velocity of the robot?
    //      at least the reference (for the feed-forward)

}

void car_kin_PI::trajectoryGeneration_step(void){

    const double t = ros::Time::now().toSec();
    xref = t/2;

    /*
    if (t <= 10){
        xref = t;
    }
    else{
        xref = 0;
    }
    */

    // Goes up/down every 10 secs
    if(std::fmod(t,20) < 10 ){
        yref = 0;
    }
    else{
        yref = 0.5;
    }
    dxref = 0;
    dyref = 0;

}

void car_kin_PI::trajectoryGeneration_sin(void){

    const double t = ros::Time::now().toSec();
    xref = t;
    yref = sin(t);
    dxref = 1;
    dyref = cos(t);

}

void car_kin_PI::control_FFPI(double& xPref, double& yPref, double& vPx,double& vPy){

    err_xP = xPref - xP;
    err_yP = yPref - yP;

    if (err_X_max < std::fabs(err_xP) || err_Y_max < std::fabs(err_yP)) {
        if (err_X_max < std::fabs(err_xP)) {
            err_X_max = std::fabs(err_xP); 
        } else {
            err_Y_max = std::fabs(err_yP);
        }
        ROS_INFO("Last maximum error (X, Y) : (%.4f, %.4f)", err_X_max, err_Y_max);
    }

    integral_x += err_xP*Ts;
    integral_y += err_yP*Ts;
    vPx = dxref + Kpx*err_xP + integral_x*Tix;
    vPy = dyref + Kpy*err_yP + integral_y*Tiy;

    // Controller Form 
    // vP = dref + Kp*(err_xP + 1/Ti*integral_x)

}

void car_kin_PI::controlSaturation(double& v, double& phi){

    double V_sat = 6, Phi_sat = 2;
    
    if(v>V_sat){
        v = V_sat;
    }else if (v<-V_sat){
        v = -V_sat;
    }

    if(phi>Phi_sat){
        phi = Phi_sat;
    }else if (phi<-Phi_sat){
        phi = -Phi_sat;
    }

}

void car_kin_PI::PeriodicTask(void)
{

    //TODO: add computation of maximum error

    /*  Generate trajectory */
    trajectoryGeneration_eight();
    //trajectoryGeneration_step();
    //trajectoryGeneration_sin();

    /*  Tranforms in P  */
    double xPref, yPref;
    double vPx, vPy;

    controller->output_transformation(xP, yP);    
    controller->reference_transformation(xref, yref, xPref, yPref);

    /*  Generate reference commands, FF+PI Controller*/
    control_FFPI(xPref, yPref, vPx, vPy);

    /*  Compute the control action */
    double v, phi;
    controller->control_transformation(vPx, vPy, v, phi);
    

    /* Saturation */
    //controlSaturation(v, phi);


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
    controllerStateMsg.data.push_back(err_xP);
    controllerStateMsg.data.push_back(err_yP);
    controllerStateMsg.data.push_back(integral_x);
    controllerStateMsg.data.push_back(integral_y);
    controllerState_publisher.publish(controllerStateMsg);

    /*  Publish reference trajectory */
    std_msgs::Float64MultiArray refTrajectoryMsg;
    refTrajectoryMsg.data.push_back(time);
    refTrajectoryMsg.data.push_back(xref);
    refTrajectoryMsg.data.push_back(yref);
    refTrajectory_publisher.publish(refTrajectoryMsg);

}
