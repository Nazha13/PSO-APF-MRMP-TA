#include <ros/ros.h>
// Library Include Section
#include <nav_msgs/OccupancyGrid.h> // For Ros Maps
#include <dynamicvoronoi/dynamicvoronoi.h> // For VD Layer

ros:Subscriber sMap;

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "my_node_name");

    // Create the object (Runs the Constructor)
    MyCustomNode node;

    // Keep running until Ctrl+C
    ros::spin();

    return 0;
}