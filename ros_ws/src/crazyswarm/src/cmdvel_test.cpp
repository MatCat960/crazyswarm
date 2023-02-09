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

#include "crazyswarm/VelocityWorld.h"



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
    velPub_ = nh_.advertise<crazyswarm::VelocityWorld>("/cf1/cmd_velocity_world",1);
    stopPub_ = nh_.advertise<std_msgs::Empty>("/cf1/stop", 1);
    timer_ = nh_.createTimer(ros::Duration(5.0), std::bind(&Controller::loop, this));
    
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
    void test_print();
    void up();
    void forward();
    void down();
    void loop();
    
    



private:
    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //------------------------- Publishers and subscribers ------------------------------
    ros::Publisher velPub_;
    ros::Publisher stopPub_;
    ros::Timer timer_;
    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    
};

void Controller::up()
{
    crazyswarm::VelocityWorld msg;
    msg.vel.x = 0.0;
    msg.vel.y = 0.0;
    msg.vel.z = 0.5;
    velPub_.publish(msg);
}

void Controller::down()
{
    crazyswarm::VelocityWorld msg;
    msg.vel.x = 0.0;
    msg.vel.y = 0.0;
    msg.vel.z = -0.5;
    velPub_.publish(msg);
}

void Controller::forward()
{
    crazyswarm::VelocityWorld msg;
    msg.vel.x = 0.5;
    msg.vel.y = 0.0;
    msg.vel.z = 0.0;
    velPub_.publish(msg);
}

void Controller::stop()
{
    std_msgs::Empty msg;
    stopPub_.publish(msg);
}

void Controller::loop()
{
    up();
    ros::Duration(1.0).sleep();
    forward();
    ros::Duration(1.0).sleep();
    down();
    ros::Duration(1.0).sleep();
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
