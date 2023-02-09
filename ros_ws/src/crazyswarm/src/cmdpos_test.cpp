// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <math.h>
#include <signal.h>
// ROS includes
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>

//Crazyswarm includes
#include "crazyswarm/Position.h"
#include "crazyswarm/Takeoff.h"
#include "crazyswarm/Land.h"
#include "crazyswarm/Hover.h"
#include "crazyswarm/GoTo.h"



#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

//Robots parameters ------------------------------------------------------
double SAFETY_DIST = 1.0;
const double MAX_LIN_VEL = 1.5;         //set to turtlebot max velocities
const double MAX_ANG_VEL = 3.0;
//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed

class Controller
{

public:
    Controller() : nh_priv_("~")
    {
    //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
    // poseSub_ = nh_.subscribe("/cf1/pose", 1, &Controller::poseCallback, this);
    posPub_ = nh_.advertise<crazyswarm::Position>("/cf1/cmd_position",1);
    stopPub_ = nh_.advertise<std_msgs::Empty>("/cf1/stop", 1);
    hoverPub_ = nh_.advertise<crazyswarm::Hover>("/cf1/cmd_hover", 1);
    takeoffClient_ = nh_.serviceClient<crazyswarm::Takeoff>("/cf1/takeoff");
    landClient_ = nh_.serviceClient<crazyswarm::Land>("/cf1/land");
    moveClient_ = nh_.serviceClient<crazyswarm::GoTo>("/cf1/go_to");
    // timer_ = nh_.createTimer(ros::Duration(10.0), std::bind(&Controller::loop, this));
    ros::Rate rate(10);
    
    //rclcpp::on_shutdown(std::bind(&Controller::stop,this));

    //----------------------------------------------------------- init Variables ---------------------------------------------------------
    // Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);
    // double theta = 0.0;
    
    time(&this->timer_init_count);
    time(&this->timer_final_count);
    }
    ~Controller()
    {
        stop();
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    //void stop(int signum);
    void stop();
    void test_print();
    void up(Eigen::VectorXd pose, double target_z);
    void forward(Eigen::VectorXd pose, double target_x);
    void down(Eigen::VectorXd pose);
    Eigen::VectorXd getPose();
    void moveTo(Eigen::VectorXd p, double yaw, ros::Duration t);
    void loop();
    void takeoff(double targetHeight, ros::Duration t);
    void land(ros::Duration t);
    void moveHovering(double vx, double vy, double yawRate, double zDistance);
    



private:
    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //------------------------- Publishers and subscribers ------------------------------
    ros::Publisher posPub_;
    ros::Publisher stopPub_;
    ros::Publisher hoverPub_;
    ros::Timer timer_;
    ros::ServiceClient takeoffClient_;
    ros::ServiceClient landClient_;
    ros::ServiceClient moveClient_;
    tf::TransformListener listener_;


    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    


    
};

void Controller::up(Eigen::VectorXd pose, double target_z)
{
    double z = 0.05;
    while (z < target_z)
    {
        std::cout << "z: " << z << "\n";
        crazyswarm::Position msg;
        msg.x = pose(0);
        msg.y = pose(1);
        msg.z = pose(2)+z;
        posPub_.publish(msg);
        z += 0.05;
        ros::Duration(0.2).sleep();
    }
}

void Controller::down(Eigen::VectorXd pose)
{
    double z = 0.05;
    while (z < 0.5)
    {
        std::cout << "z: " << z << "\n";
        crazyswarm::Position msg;
        msg.x = pose(0) + 0.5;
        msg.y = pose(1);
        msg.z = pose(2) - z;
        posPub_.publish(msg);
        z += 0.05;
        ros::Duration(0.2).sleep();
    }
}

void Controller::forward(Eigen::VectorXd pose, double target_x)
{
    double x = 0.05;
    while (x < target_x)
    {
        std::cout << "x: " << x << "\n";
        crazyswarm::Position msg;
        msg.x = pose(0) + x;
        msg.y = pose(1);
        msg.z = pose(2) + 0.5;
        posPub_.publish(msg);
        x += 0.05;
        ros::Duration(0.2).sleep();
    }
}

void Controller::stop()
{
    std_msgs::Empty msg;
    stopPub_.publish(msg);
}

void Controller::moveTo(Eigen::VectorXd p, double yaw, ros::Duration t)
{
    crazyswarm::GoTo srv;
    srv.request.relative = true;
    srv.request.goal.x = p(0);
    srv.request.goal.y = p(1);
    srv.request.goal.z = p(2);
    srv.request.yaw = yaw;
    srv.request.duration = t;
    moveClient_.call(srv);
}

void Controller::moveHovering(double vx, double vy, double yawRate, double zDistance)
{
    crazyswarm::Hover msg;
    msg.vx = vx;
    msg.vy = vy;
    msg.yawrate = yawRate;
    msg.zDistance = zDistance;
    hoverPub_.publish(msg);
}

Eigen::VectorXd Controller::getPose()
{
    Eigen::VectorXd p(4);
    tf::StampedTransform transform;
    try
    {
        listener_.waitForTransform("/cf1", "/world", ros::Time(0), ros::Duration(1.0));
        listener_.lookupTransform("/cf1", "/world", ros::Time(0), transform);
        p(0) = transform.getOrigin().x();
        p(1) = transform.getOrigin().y();
        p(2) = transform.getOrigin().z();
        p(3) = tf::getYaw(transform.getRotation());
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    return p;

}

void Controller::takeoff(double targetHeight, ros::Duration t)
{
    std::cout << "Taking off" << std::endl;
    crazyswarm::Takeoff srv;
    srv.request.height = targetHeight;
    srv.request.duration = t;
    takeoffClient_.call(srv);
    std::cout << "Takeoff complete!" << std::endl;
}

void Controller::land(ros::Duration t)
{
    std::cout << "Landing" << std::endl;
    crazyswarm::Land srv;
    srv.request.height = 0.0;
    srv.request.duration = t;
    landClient_.call(srv);
    std::cout << "Landing complete!" << std::endl;
}

void Controller::loop()
{
    Eigen::VectorXd p0 = getPose();
    // Eigen::VectorXd p2, p3;
    // theta = p(3);
    ros::Duration takeoff_time(3.0);
    std::cout<<"pose: " << p0.transpose() <<std::endl;
    // std::cout << "Moving up" << std::endl;
    // up(p0, 0.5);
    // Eigen::Vector3d p1;
    // p1(0) = p0(0);
    // p1(1) = p0(1);
    // p1(2) = p0(2)+0.5;
    // moveTo(p1);
    // // ros::Duration(5.0).sleep();
    // std::cout << "Moving forward" << std::endl;
    // Eigen::Vector3d p2;
    // p2(0) = p0(0)+0.5;
    // p2(1) = p0(1);
    // p2(2) = p0(2)+0.5;
    // // p2 << p0(0)+0.5, p0(1), p0(2)+0.5;
    // moveTo(p2);
    // ros::Duration(5.0).sleep();
    // Eigen::Vector3d p3;
    // p3(0) = p0(0)+0.5;
    // p3(1) = p0(1);
    // p3(2) = p0(2);
    // std::cout << "Moving down" << std::endl;
    // // p3 << p0(0)+0.5, p0(1), p0(2);
    // moveTo(p3);
    // ros::Duration(5.0).sleep();
    takeoff(0.5, takeoff_time);
    ros::Duration(5.0).sleep();
    Eigen::VectorXd p1 = getPose();
    std::cout<<"Hovering pose: " << p1.transpose() <<std::endl;
    std::cout << "Moving forward" << std::endl;
    // moveHovering(0.0, 0.0, 0.5, 0.5);
    // forward(p1, 0.5);
    Eigen::VectorXd p2(3);
    p2(0) = 0.0;
    p2(1) = 0.5;
    p2(2) = 0.0;
    moveTo(p2, 0.0, ros::Duration(3.0));
    ros::Duration(5.0).sleep();
    std::cout << "Move completed" << std::endl;

    // Eigen::VectorXd p2(3);
    // p1 = getPose();
    p2(0) = 0.0;
    p2(1) = 0.0;
    p2(2) = 0.0;
    std::cout << "Starting rotation" << std::endl;
    moveTo(p2, 3.14, ros::Duration(3.0));
    ros::Duration(5.0).sleep();
    std::cout << "Rotation completed" << std::endl;

    // Eigen::VectorXd p2(3);
    // p1 = getPose();
    p2(0) = 0.0;
    p2(1) = -0.5; 
    p2(2) = 0.0; 
    std::cout << "Coming back" << std::endl;
    moveTo(p2, 0.0, ros::Duration(3.0));
    ros::Duration(5.0).sleep();
    std::cout << "Finished" << std::endl;
    // down(p0);
    land(takeoff_time);
}


//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

std::shared_ptr<Controller> globalobj_signal_handler;     //the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int){
    std::cout<<"signal handler function CALLED"<<std::endl;
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, nodeobj_wrapper_function);

    ros::init(argc, argv, "cmdpos_test", ros::init_options::NoSigintHandler);
    auto node = std::make_shared<Controller>();

    // globalobj_signal_handler = node;    //to use the ros function publisher, ecc the global pointer has to point to the same node object.


    // rclcpp::spin(node);

    // rclcpp::sleep_for(100000000ns);
    // rclcpp::shutdown();

    // while(!node_shutdown_request)
    // {
    //     ros::spinOnce();
    // }
    node->loop();
    node->stop();

    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}
