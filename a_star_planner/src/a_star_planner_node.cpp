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

#include "a_star_planner/AStar.hpp"

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackLocalPath(const visualization_msgs::Marker& msg);
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

	visualization_msgs::Marker m_LocalPath;
	nav_msgs::Odometry currentPose;
        geometry_msgs::PoseStamped m_GoalPose;
        geometry_msgs::PoseStamped OnBodyCoord;
        bool bNewGoalPose;
        int lookahead_idx;
        int nearest_idx;
};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    lookahead_idx = nh_.param("look_ahead_in_local_path", 14);
    subOdom = nh_.subscribe("/odom",1, &AstarPlanner::CallbackOdom, this);
    subOccupancyGrid = nh_.subscribe("/semantics/costmap_generator/occupancy_grid",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subLocalPath = nh_.subscribe("/waypoint_markers",1, &AstarPlanner::CallbackLocalPath, this);

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/a_star", 1, true);
    pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}

void AstarPlanner::CallbackOdom(const nav_msgs::Odometry& msg)
{
    currentPose = msg;
}

void AstarPlanner::CallbackGoalRviz(const geometry_msgs::PoseStamped& msg)
{
    bNewGoalPose = true;
}

void AstarPlanner::CallbackLocalPath(const visualization_msgs::Marker& msg)
{
    m_LocalPath = msg;
      m_GoalPose.pose.position.x = m_LocalPath.points.at(lookahead_idx).x;
      m_GoalPose.pose.position.y = m_LocalPath.points.at(lookahead_idx).y;
    bNewGoalPose = true;
	
}

void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
    if(!bNewGoalPose)
        return;



 double dist [m_LocalPath.points.size()];
    for (int i = 0; i < m_LocalPath.points.size(); i++) {
	double tx = m_LocalPath.points.at(i).x;
	double ty = m_LocalPath.points.at(i).y;
	double ix = currentPose.pose.pose.position.x;
	double iy = currentPose.pose.pose.position.y;
        dist [i] = sqrt( pow(tx-ix, 2) + pow(ty-iy, 2) );
}
	int index = 0;

    for(int i = 1; i < m_LocalPath.points.size(); i++)
    {
        if(dist[i] < dist[index])
            index = i;             
    }

    nearest_idx = index;
    std::cout << "NEAREST INDEX: " << nearest_idx << std::endl;

if(m_LocalPath.points.size() < nearest_idx+lookahead_idx)
    {
	m_GoalPose.pose.position.x = m_LocalPath.points.at(nearest_idx+lookahead_idx-651).x;
    m_GoalPose.pose.position.y = m_LocalPath.points.at(nearest_idx+lookahead_idx-651).y;
}
    
else {
    m_GoalPose.pose.position.x = m_LocalPath.points.at(nearest_idx+lookahead_idx).x;
    m_GoalPose.pose.position.y = m_LocalPath.points.at(nearest_idx+lookahead_idx).y;
}
    bNewGoalPose = true;
    
    double x = currentPose.pose.pose.orientation.x;
    double y = currentPose.pose.pose.orientation.y;
    double z = currentPose.pose.pose.orientation.z;
    double w = currentPose.pose.pose.orientation.w;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    double roll, pitch, veh_yaw;
    m.getRPY(roll, pitch, veh_yaw);
    OnBodyCoord.pose.position.x =
          (m_GoalPose.pose.position.x - currentPose.pose.pose.position.x) *
              cos(veh_yaw) +
          (m_GoalPose.pose.position.y - currentPose.pose.pose.position.y) *
              sin(veh_yaw);
      OnBodyCoord.pose.position.y =
          (m_GoalPose.pose.position.x - currentPose.pose.pose.position.x) *
              -sin(veh_yaw) +
          (m_GoalPose.pose.position.y - currentPose.pose.pose.position.y) *
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

    int origin_grid_x = -msg.info.origin.position.x / msg.info.resolution +5;
    int origin_grid_y = -msg.info.origin.position.y / msg.info.resolution +1;
    int target_grid_x = (target_x) / msg.info.resolution + origin_grid_x;
    int target_grid_y = (target_y) / msg.info.resolution + origin_grid_y;
    //cut the goal pose in the grid map
    if(target_grid_y < 0)
        target_grid_y = 0;
    else if(target_grid_y > grid_y_size)
        target_grid_y = grid_y_size;    

    //add the collision 
    int buf_size = nh_.param("grid_buf_size", 1);
    for (int grid_x = 0; grid_x < grid_x_size; grid_x++) {
        for (int grid_y = 0; grid_y < grid_y_size; grid_y++) {
            if (msg.data[grid_y * grid_x_size + grid_x] > 40) {
                generator.addCollision({grid_x, grid_y}, buf_size);

                pcl::PointXYZI pointTmp;
                pointTmp.x = grid_x* msg.info.resolution +
                                msg.info.resolution / 2 +
                                msg.info.origin.position.x;
                pointTmp.y = grid_y* msg.info.resolution+
                                msg.info.resolution / 2 +
                                msg.info.origin.position.y;

                cloud_in_ptr->points.push_back(pointTmp);

            }
        }    
    }

    if(generator.detectCollision({target_grid_x, target_grid_y}))
    {
int new_target_grid_x;
int new_target_grid_y;
   	 std::cout << "yes" << std::endl;
        for(int i = 1; i <  (652-(lookahead_idx+nearest_idx)); i++)
        {
   	 std::cout << "i: " << i << std::endl;
	m_GoalPose.pose.position.x = m_LocalPath.points[lookahead_idx+nearest_idx+(3*i)].x;
	m_GoalPose.pose.position.y = m_LocalPath.points[lookahead_idx+nearest_idx+(3*i)].y;

 OnBodyCoord.pose.position.x =
          (m_GoalPose.pose.position.x - currentPose.pose.pose.position.x) *
              cos(veh_yaw) +
          (m_GoalPose.pose.position.y - currentPose.pose.pose.position.y) *
              sin(veh_yaw);
      OnBodyCoord.pose.position.y =
          (m_GoalPose.pose.position.x - currentPose.pose.pose.position.x) *
              -sin(veh_yaw) +
          (m_GoalPose.pose.position.y - currentPose.pose.pose.position.y) *
              cos(veh_yaw);


            new_target_grid_x = (OnBodyCoord.pose.position.x) / msg.info.resolution + origin_grid_x;
            new_target_grid_y = (OnBodyCoord.pose.position.y) / msg.info.resolution + origin_grid_y;     
               	 std::cout << "New: " << new_target_grid_x <<", "<< new_target_grid_y << std::endl;
            if(generator.detectCollision({new_target_grid_x, new_target_grid_y}) == false)
            {
                target_grid_x = new_target_grid_x;
                target_grid_y = new_target_grid_y;
   	 std::cout << "New: " << new_target_grid_x <<", "<< new_target_grid_y << std::endl;
                break;
            }       
		
        }
    }


    //visualize the collision grid
    sensor_msgs::PointCloud2 collisionCloudMsg;
    pcl::toROSMsg(*cloud_in_ptr, collisionCloudMsg);
    collisionCloudMsg.header.frame_id = "base_link";
    collisionCloudMsg.header.stamp = ros::Time::now();
    pubCollisionPoints.publish(collisionCloudMsg);

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
        poseBuf.pose.position.z = 0.665;
        AStartPathMsg.poses.push_back(poseBuf);
    }
    pubAstarPath.publish(AStartPathMsg);

}

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "a_star_planner");
    ros::NodeHandle nh;
    AstarPlanner planner(nh);
    ros::spin();
    return 0;

}
