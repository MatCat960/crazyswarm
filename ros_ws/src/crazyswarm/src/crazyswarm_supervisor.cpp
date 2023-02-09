// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <cmath>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

//------------------------------------------------------------------------
const int debug = 0;
//------------------------------------------------------------------------
const int shutdown_timer = 5;           //count how many seconds to let the robots stopped before shutting down the node
//------------------------------------------------------------------------
//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed
//------------------------------------------------------------------------

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}

class Controller
{
public:
    Controller() : nh_priv_("~")
    {
        ROS_INFO("Node Initialization");
        //-------------------------------------------------------- ROS parameters -----------------------------------------------------------
        this->nh_priv_.getParam("ids", ids_);
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);
        this->nh_priv_.getParam("SAVE_POS", SAVE_POS);

        //sensing range single robot (= halfe edge of the local sensing box)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);
        this->nh_priv_.getParam("ROBOT_FOV", ROBOT_FOV);

        // Parameters for Gaussian

        // Area parameter
        // this->nh_priv_.getParam("AREA_SIZE_x", AREA_SIZE_x);
        // this->nh_priv_.getParam("AREA_SIZE_y", AREA_SIZE_y);
        // this->nh_priv_.getParam("AREA_LEFT", AREA_LEFT);
        // this->nh_priv_.getParam("AREA_BOTTOM", AREA_BOTTOM);

        //view graphical voronoi rapresentation - bool
        //-----------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
        for (int i = 1; i < ROBOTS_NUM+1; i++)
        {
            realposePub_.push_back(nh_.advertise<geometry_msgs::Pose>("/supervisor/robot" + std::to_string(i) + "/realpose", 1));
            neighposePub_.push_back(nh_.advertise<geometry_msgs::PoseArray>("/supervisor/robot" + std::to_string(i) + "/pose", 1));
        }
        controller_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::Emulate_Vision, this));
        //-----------------------------------------------------------------------------------------------------------------------------------
        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM+1);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM+1);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM+1);
        //------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------- Graphics window -----------------------------------------------
        //---------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------open log file -----------------------------------------------
        //---------------------------------------------------------------------------------------------------------------
        ROBOT_FOV_rad = 0.5 * ROBOT_FOV /180 * M_PI;
    }

    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
        //ros::shutdown();
    }

    void stop();
    void getPoses();
    void Emulate_Vision();

    //Graphics -- draw coverage

    //open write and close LOG file

private:
    std::vector<int> ids_{1,2,3};
    int ROBOTS_NUM = 3;
    double ROBOT_RANGE = 5;
    double ROBOT_FOV = 100.0;
    double ROBOT_FOV_rad;
    bool SAVE_POS = false;
    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    // std::vector<Vector2<double>> seeds_xy;

    

    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //-----------------------------------------------------------------------------------
    //------------------------- Publishers and subscribers ------------------------------
    std::vector<ros::Publisher> neighposePub_;
    std::vector<ros::Publisher> realposePub_;
    ros::Timer controller_timer_;
    tf::TransformListener listener_;
    //-----------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    // double AREA_SIZE_x = 3.0;
    // double AREA_SIZE_y = 3.0;
    // double AREA_LEFT = -1.5;
    // double AREA_BOTTOM = -1.5;
    //------------------------------------------------------------------------------------

};

void Controller::stop()
{
    ROS_INFO("shutting down the supervisor controller");
    
    if (SAVE_POS)
    {
        ROS_INFO("Saving final position of robots");
        std::ofstream myfile;
        myfile.open ("/home/mattia/final_pos.txt");
        for (int i = 0; i < ROBOTS_NUM+1; i++)
        {
            myfile << pose_x(i) << " " << pose_y(i) << " " << pose_theta(i) << std::endl;
        }
        myfile.close();
        ROS_INFO("Final position of robots saved");
    }
    

    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::getPoses()
{
    this->pose_x(0) = 100.0;
    this->pose_y(0) = 100.0;
    this->pose_theta(0) = 100.0;
    for (int i = 1; i < ROBOTS_NUM+1; i++)
    {
        tf::StampedTransform transform;
        try
        {
            listener_.waitForTransform("/world", "/cf"+std::to_string(i), ros::Time(0), ros::Duration(1.0));
            listener_.lookupTransform("/world", "/cf"+std::to_string(i), ros::Time(0), transform);
            this->pose_x(i) = transform.getOrigin().x();
            this->pose_y(i) = transform.getOrigin().y();
            // p(2) = transform.getOrigin().z();
            this->pose_theta(i) = tf::getYaw(transform.getRotation());
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    // Publish real global position of robots
    for (int i = 1; i < ROBOTS_NUM+1; i++)
    {
        geometry_msgs::Pose realpose;
        realpose.position.x = this->pose_x(i);
        realpose.position.y = this->pose_y(i);
        realpose.position.z = 0.0;
        realpose.orientation = tf::createQuaternionMsgFromYaw(this->pose_theta(i));
        realposePub_[i-1].publish(realpose);
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------Rendering functions - for SFML graphical visualization-----------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Controller::Emulate_Vision(){

    // Update current pose of robots
    getPoses();


    // fake position for non detected robots
    geometry_msgs::Pose fake_pose;
    fake_pose.position.x = 100.0;
    fake_pose.position.y = 100.0;
    fake_pose.position.z = 0.0;

    for (int i = 1; i < ROBOTS_NUM+1; ++i)
    {
        if (this->pose_x(i) == 0.0)
        {   
            std::cout << "Waiting for initialization of robot " << i << std::endl;
            continue;
        }
        geometry_msgs::PoseArray neighbors;
        
        neighbors.header.stamp = ros::Time::now();
        std::stringstream ss;
        ss << "/cf"<< i;
        neighbors.header.frame_id = ss.str();

        for (int j = 1; j < ROBOTS_NUM+1; ++j)
        { 
            if (i != j)
            {
                // Vector2<double> distance_vect = {(this->pose_x[j] - this->pose_x[i]), (this->pose_y[j] - this->pose_y[i])};
                double dist_x = this->pose_x(j) - this->pose_x(i);
                double dist_y = this->pose_y(j) - this->pose_y(i);
                double dist = sqrt(pow(dist_x,2) + pow(dist_y,2));

                if (dist <= ROBOT_RANGE)
                {
                    geometry_msgs::Pose msg;
                    //the distance_vect is already the point neighbor expressed in local coordinates
                    // msg.position.x = cos(this->pose_theta(i))*distance_vect.x + sin(this->pose_theta(i))*distance_vect.y;
                    // msg.position.y = -sin(this->pose_theta(i))*distance_vect.x + cos(this->pose_theta(i))*distance_vect.y;
                    msg.position.x = dist_x * cos(this->pose_theta(i)) + dist_y * sin(this->pose_theta(i));
                    msg.position.y = -dist_x * sin(this->pose_theta(i)) + dist_y * cos(this->pose_theta(i));
                    msg.orientation.w = 1.0;

                    // Filter robots that are not in the FOV
                    double angle = abs(atan2(msg.position.y, msg.position.x));
                    if (angle <= ROBOT_FOV_rad &&  msg.position.x > 0.0)
                        {
                            neighbors.poses.push_back(msg);
                            std::cout << "robot " << i << " sees robot " << j << " at " << msg.position.x << " " << msg.position.y << std::endl;
                        }
                    else
                    {
                        // Filter robots outside FOV
                        neighbors.poses.push_back(fake_pose);
                    }
                } else {
                    // Filter robots outside range
                    neighbors.poses.push_back(fake_pose);
                }
            } else
            {
                // Fake pose in self position
                neighbors.poses.push_back(fake_pose);
            }
        }

        // std::cout<<"robot "<<i<<" has "<<neighbors.poses.size()<<" neighbours"<<std::endl;
        this->neighposePub_[i-1].publish(neighbors);
    }
}

/*******************************************************************************
* Main function
*******************************************************************************/
//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_coverage_supervisor", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    //Controller node_controller;
    auto node_controller = std::make_shared<Controller>();

    while (!node_shutdown_request){
        ros::spinOnce();
    }
    node_controller->stop();

    //ros::spin();
    //do pre-shutdown tasks
    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}