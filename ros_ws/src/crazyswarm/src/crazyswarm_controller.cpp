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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
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
    stopPub_ = nh_.advertise<std_msgs::Empty>("/stop", 1);
    takeoffSub_ = nh_.subscribe("/controller/takeoff", 1, &Controller::takeoff, this);          // Subcribe to takeoff topic -- float32 targetHeight
    landSub_ = nh_.subscribe("/controller/land", 1, &Controller::land, this);                  // Subcribe to land topic -- std_msgs/Empty
    moveSub_ = nh_.subscribe("/controller/go_to", 1, &Controller::moveTo, this);                  // Subcribe to move topic -- geometry_msgs/Pose (relative)
    velSub_ = nh_.subscribe("/controller/cmd_vel", 1, &Controller::velControl, this);                  // Subcribe to cmd_vel topic -- geometry_msgs/Twist (local frame)
    takeoffClient_ = nh_.serviceClient<crazyswarm::Takeoff>("/takeoff");
    landClient_ = nh_.serviceClient<crazyswarm::Land>("/land");
    moveClient_ = nh_.serviceClient<crazyswarm::GoTo>("/go_to");
    
    //rclcpp::on_shutdown(std::bind(&Controller::stop,this));

    //----------------------------------------------------------- init Variables ---------------------------------------------------------
    
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
    void moveTo(geometry_msgs::Pose p);
    void takeoff(std_msgs::Float32 targetHeight);
    void land(std_msgs::Empty msg);
    void velControl(geometry_msgs::Twist msg);
    



private:
    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //------------------------- Publishers and subscribers ------------------------------
    ros::Publisher stopPub_;
    ros::Subscriber takeoffSub_;
    ros::Subscriber landSub_;
    ros::Subscriber moveSub_;
    ros::Subscriber velSub_;
    ros::ServiceClient takeoffClient_;
    ros::ServiceClient landClient_;
    ros::ServiceClient moveClient_;


    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

};


void Controller::stop()
{
    std_msgs::Empty msg;
    stopPub_.publish(msg);
}

void Controller::moveTo(geometry_msgs::Pose p)
{
    crazyswarm::GoTo srv;
    srv.request.relative = true;
    srv.request.goal.x = p.position.x;
    srv.request.goal.y = p.position.y; 
    srv.request.goal.z = p.position.z; 
    srv.request.yaw = tf::getYaw(p.orientation);
    srv.request.duration = ros::Duration(3.0);
    moveClient_.call(srv);
}

void Controller::takeoff(std_msgs::Float32 targetHeight)
{
    std::cout << "Taking off" << std::endl;
    crazyswarm::Takeoff srv;
    srv.request.height = targetHeight.data;
    srv.request.duration = ros::Duration(3.0);
    takeoffClient_.call(srv);
    std::cout << "Takeoff complete!" << std::endl;
}

void Controller::land(std_msgs::Empty msg)
{
    std::cout << "Landing" << std::endl;
    crazyswarm::Land srv;
    srv.request.height = 0.0;
    srv.request.duration = ros::Duration(3.0);
    landClient_.call(srv);
    std::cout << "Landing complete!" << std::endl;
}

void Controller::velControl(geometry_msgs::Twist msg)
{
    crazyswarm::GoTo srv;
    srv.request.relative = true;
    srv.request.duration = ros::Duration(3.0);

    srv.request.goal.x = msg.linear.x * 3.0;
    srv.request.goal.y = msg.linear.y * 3.0;
    srv.request.goal.z = msg.linear.z * 3.0;
    srv.request.yaw = msg.angular.z * 3.0;

    moveClient_.call(srv);
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

    ros::init(argc, argv, "crazyswarm_controller", ros::init_options::NoSigintHandler);
    auto node = std::make_shared<Controller>();

    // globalobj_signal_handler = node;    //to use the ros function publisher, ecc the global pointer has to point to the same node object.


    // rclcpp::spin(node);

    // rclcpp::sleep_for(100000000ns);
    // rclcpp::shutdown();

    while(!node_shutdown_request)
    {
        ros::spinOnce();
    }
    node->stop();

    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}
