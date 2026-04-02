#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Graph.h> // The message type
#include <iostream> // For std::cout
#include <vector> // For std::vector 

struct Node {
    double x;
    double y;
};

bool map_received = false;

// THE CALLBACK
// This function runs automatically every time a message hits "/segments"
void graphCallback(const tuw_multi_robot_msgs::Graph::ConstPtr& msg) {
    
    std::cout << "--- NEW GRAPH RECEIVED ---" << std::endl;
    std::cout << "Current Node: " << msg->vertices[0].id << std::endl;
    std::vector<Node> nodes;

    Node n;

    std::cout << "Path Points for Node 0 to Next Node : " << std::endl;

    for(int j = 0; j < msg->vertices[0].path.size(); j++){
        n.x = msg->vertices[0].path[j].x;
        n.y = msg->vertices[0].path[j].y;
        nodes.push_back(n);

        std::cout << "(" << n.x << ", " << n.y << ")" << std::endl;
    }

    map_received = true;
}

// THE MAIN
int main(int argc, char **argv) {


    // 1. Initialize ROS
    ros::init(argc, argv, "simple_graph_listener");
    
    // 2. Create the Handle
    ros::NodeHandle nh;

    // 3. Subscribe
    // Topic: "/segments"
    // Buffer: 10 (save last 10 messages if we are busy)
    // Function: graphCallback
    ros::Subscriber sub = nh.subscribe("/segments", 10, graphCallback);

    std::cout << "Listening for Voronoi Graph..." << std::endl;

    ros::Rate rate(10); // 10 Hz

    while (ros::ok() && !map_received) {
        ros::spinOnce(); // Check for new messages
        rate.sleep(); // Sleep to maintain the loop rate
    }

    if(!ros::ok()) return 0;

    return 0;
}