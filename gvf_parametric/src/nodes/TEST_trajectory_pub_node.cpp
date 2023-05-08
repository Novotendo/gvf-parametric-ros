#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TEST_trajectory_pub_node");
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_("~");

    ros::Publisher path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 1);

    // std::vector<double> waypoints_X = {0.0, 0.8, 1.4, 0.8, 0.0, -0.8, -1.4, -0.8, 0.0};
    // std::vector<double> waypoints_Y = {0.8, 1.6, 0.8, 0.0, 0.8,  1.6,  0.8,  0.0, 0.8};

    std::vector<double> waypoints_X = {0.0, 2.0, 3.0, 2.0, 0.0, -2.0, -3.0, -2.0, 0.0};
    std::vector<double> waypoints_Y = {0.0, 2.0, 0.0, -2.0, 0.0, 2.0, 0.0, -2.0, 0.0};

    nav_msgs::Path path;
    geometry_msgs::PoseStamped waypoint;

    for(int i = 0 ; i < waypoints_X.size() ; i++)
    {
        waypoint.pose.position.x = waypoints_X[i];
        waypoint.pose.position.y = waypoints_Y[i];
        path.poses.push_back(waypoint);
    }

    while(ros::ok())
    {
        ros::Duration(3).sleep();
        ros::spinOnce();
        path_pub_.publish(path);
        ros::shutdown();
    }

    return 0;
}