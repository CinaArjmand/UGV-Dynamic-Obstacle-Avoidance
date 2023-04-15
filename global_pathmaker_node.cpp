#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

#include "a_star_planner/AStar.hpp"

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackLocalPath(const nav_msgs::Path& msg);
        void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);
        void CallbackOdom(const nav_msgs::Odometry& msg);

        ros::NodeHandle nh_;
        
    private:
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subLocalPath;
        ros::Subscriber subGoalPose;
        ros::Subscriber subOdom;

        ros::Publisher pubAstarPath; 
        ros::Publisher pubCollisionPoints;

        nav_msgs::Path m_LocalPath;
	geometry_msgs::PoseStamped currentPose;
        geometry_msgs::PoseStamped m_GoalPose;
        geometry_msgs::PoseStamped OnBodyCoord;
       
        bool bNewGoalPose;
        int lookahead_idx;
};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    lookahead_idx = nh_.param("look_ahead_in_local_path", 3);
    subOccupancyGrid = nh_.subscribe("/semantics/costmap_generator/occupancy_grid",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subLocalPath = nh_.subscribe("/Path/LocalWaypoint/Fitting",1, &AstarPlanner::CallbackLocalPath, this);
    subOdom = nh_.subscribe("/lio_sam/mapping/odometry",1, &AstarPlanner::CallbackOdom, this);
    subGoalPose = nh_.subscribe("/move_base_simple/goal",1, &AstarPlanner::CallbackGoalRviz, this);

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/global_path", 1, true);
    pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}

void AstarPlanner::CallbackOdom(const nav_msgs::Odometry& msg)
{
    currentPose.pose = msg.pose.pose;
    currentPose.header.frame_id = "base_link";
    
}

void AstarPlanner::CallbackGoalRviz(const geometry_msgs::PoseStamped& msg)
{
    m_GoalPose = msg;
    std::cout << "GP : "<< m_GoalPose << std::endl;
    m_GoalPose.pose.position.z = 0.665;
    bNewGoalPose = true;
}

void AstarPlanner::CallbackLocalPath(const nav_msgs::Path& msg)
{
    m_LocalPath = msg;
    if(msg.poses.size() > lookahead_idx && nh_.param("use_local_path_for_a_star", true))
    {
        bNewGoalPose = true;
    }
}

void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
    if(!bNewGoalPose)
        return;
    double x = currentPose.pose.orientation.x;
    double y = currentPose.pose.orientation.y;
    double z = currentPose.pose.orientation.z;
    double w = currentPose.pose.orientation.w;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    double roll, pitch, veh_yaw;
    m.getRPY(roll, pitch, veh_yaw);
    OnBodyCoord.pose.position.x =
          (m_GoalPose.pose.position.x - currentPose.pose.position.x) *
              cos(veh_yaw) +
          (m_GoalPose.pose.position.y - currentPose.pose.position.y) *
              sin(veh_yaw);
      OnBodyCoord.pose.position.y =
          (m_GoalPose.pose.position.x - currentPose.pose.position.x) *
              -sin(veh_yaw) +
          (m_GoalPose.pose.position.y - currentPose.pose.position.y) *
              cos(veh_yaw);

    double target_x = OnBodyCoord.pose.position.x; //longitudinal
    double target_y = OnBodyCoord.pose.position.y; //lateral

    int row, col, rotate_row, rotate_col;
    unsigned int grid_x_size = msg.info.width;
    unsigned int grid_y_size = msg.info.height;

    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({grid_x_size, grid_y_size});
    // Heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(false);

    generator.clearCollisions();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    int origin_grid_x = -msg.info.origin.position.x / msg.info.resolution + 1;
    int origin_grid_y = -msg.info.origin.position.y / msg.info.resolution + 1;
    int target_grid_x = (target_x) / msg.info.resolution + origin_grid_x;
    int target_grid_y = (target_y) / msg.info.resolution + origin_grid_y;
    //cut the goal pose in the grid map
    if(target_grid_y < 0)
        target_grid_y = 0;
    else if(target_grid_y > grid_y_size)
        target_grid_y = grid_y_size;    


    auto _path = generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y});

    std::cout << "origin : "<< origin_grid_x << ", " << origin_grid_y << std::endl;
    std::cout << "target : "<< target_grid_x << ", " << target_grid_y << std::endl;
    std::cout << "path size: " << _path.size() << std::endl;

    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time();
    AStartPathMsg.header.frame_id = "base_link";
    for(int i = _path.size()-1; i > 0; i--) 
    {
        auto coordinate = _path[i];
        geometry_msgs::PoseStamped poseBuf;
        poseBuf.pose.position.x = (coordinate.x - origin_grid_x) * msg.info.resolution; 
        poseBuf.pose.position.y = (coordinate.y - origin_grid_y) * msg.info.resolution;
        AStartPathMsg.poses.push_back(poseBuf);

    }
    pubAstarPath.publish(AStartPathMsg);

}

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "gp_planner");
    ros::NodeHandle nh;
    AstarPlanner planner(nh);
    ros::spin();
    return 0;

}
