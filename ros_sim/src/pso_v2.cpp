// Libraries declarations
#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Graph.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>

// Global variables
bool map_received = false;
bool start_received = false;
bool goal_received = false;

// Map parameters to fix VD segment and map sync issues
double map_width_pixels = 608.0;
double map_height_pixels = 729.0;
double resolution = 0.05;
double offset_x = (map_width_pixels * resolution) / 2.0;
double offset_y = (map_height_pixels * resolution) / 2.0;

double c1 = 2; // cognitive coefficient
double c2 = 1; // social coefficient
double w = 0.75; // particle inertia weight  
double r1; 
double r2; 

int swarm_size = 100;
int max_iter = 200;

double start_yaw = 0.0;

// Sturct definitions
struct Edge {
    int id;
    double weight;
};

struct Node {
    int id;
    double x,y;

    std::vector<Edge> neighbours;
    int neighbours_count;
};

struct Particle {
    std::vector<double> pos_i;
    std::vector<double> velo;

    double score;
    double pBest_score;
    std::vector<double> pBestPos;
};

struct Global {
    std::vector<double> g_pos;
    double g_score;
};

struct Coordinate {
    double x, y;
};

struct PosNode {
    int id;
    double distance;
};

//  Global struct declarations
std::vector<Node> nodes;

Coordinate real_start;
Coordinate real_goal;

std::vector<int> start_candidates;
std::vector<int> goal_candidates;

// Function declarations

// Function to calculate distance between two nodes that make up the segments or roads
double dist(Node* a, Node* b){
    return std::sqrt(std::pow(b->x - a->x,2) + std::pow(b->y - a->y,2));
}

// Function to listen to roadmap topic and store it in a variable that can be used by PSO
void roadmapCallback(const tuw_multi_robot_msgs::Graph::ConstPtr& msg) {

    nodes.clear();
    
    for(int i = 0; i < msg->vertices.size(); ++i){
        Node n;

        n.id = msg->vertices[i].id;
        n.x = msg->vertices[i].path[0].x - 13.313419;
        n.y = msg->vertices[i].path[0].y - 14.406021;
        n.neighbours_count = msg->vertices[i].successors.size() + msg->vertices[i].predecessors.size();
        nodes.push_back(n);
    }

    for(int i = 0; i < msg->vertices.size(); ++i){
        for(int j = 0; j < msg->vertices[i].successors.size(); ++j){
            Edge e;

            e.id = msg->vertices[i].successors[j];

            e.weight = dist(&nodes[i], &nodes[e.id]);
            nodes[i].neighbours.push_back(e);
        }
        for(int j = 0; j < msg->vertices[i].predecessors.size(); ++j){
            Edge e;

            e.id = msg->vertices[i].predecessors[j];

            e.weight = dist(&nodes[i], &nodes[e.id]);
            nodes[i].neighbours.push_back(e);
        }
    }

    map_received = true;
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    real_start.x = msg->pose.pose.position.x;
    real_start.y = msg->pose.pose.position.y;
    start_yaw = tf::getYaw(msg->pose.pose.orientation);

    start_received = true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    real_goal.x = msg->pose.position.x;
    real_goal.y = msg->pose.position.y;

    goal_received = true;
}

// Function to allow particle to explore the roadmap and return fitness value (distance + penalty)
double calculateFitness(Particle* p, Node* graph){
    double totalDistance = 0.0;
    double totalTurnPenalty = 0.0;

    double w_1 = 1.0;
    double w_2 = 2.0;

    int startNode_i = nodes.size() - 2;
    int goalNode_i = nodes.size() - 1;

    std::vector<bool> visited(nodes.size(), false); // Track visited nodes so it does not oscillate

    // Hypothetical robot is not moving at start
    int currentNode = startNode_i;
    visited[currentNode] = true;

    double currentYaw = start_yaw;

    int steps = 0;
    int max_steps = 300;

    while(currentNode != goalNode_i && steps < max_steps){
        Node* n = &graph[currentNode];

        int bestNode = currentNode;
        double highestValue = 0;
        double tempDist= 0;
        double tempAngleDiff = 0;
        double tempYaw = currentYaw;

        for(int i = 0; i < n->neighbours_count; ++i){
            if(p->pos_i[n->neighbours[i].id] > highestValue && !visited[n->neighbours[i].id]){
                Node* neighbourNode = &graph[n->neighbours[i].id];

                double angleToNeighbour = std::atan2(neighbourNode->y - n->y, neighbourNode->x - n->x);
                double angleDiff = std::abs(std::atan2(std::sin(angleToNeighbour - currentYaw), std::cos(angleToNeighbour - currentYaw)));
                
                highestValue = p->pos_i[n->neighbours[i].id];
                bestNode = n->neighbours[i].id;
                tempDist = n->neighbours[i].weight;

                tempAngleDiff = angleDiff;
                tempYaw = angleToNeighbour;
            }
        }

        currentNode = bestNode;
        visited[currentNode] = true;
        totalDistance += tempDist;
        totalTurnPenalty += tempAngleDiff;
        currentYaw = tempYaw;

        steps++;
    }
    if(currentNode == goalNode_i){
        return (totalDistance * w_1) + (totalTurnPenalty * w_2);
    }
    else{
        return (totalDistance * w_1) + (totalTurnPenalty * w_2) + 10000.0;
    }
}

// Function to return best sequence of nodes from start to goal based on gBest
std::vector<int> calculateFinale(Global* g, Node* graph){

    std::vector<int> path;

    std::vector<bool> visited(nodes.size(), false); // Track visited nodes so it does not oscillate

    int startNode_i = nodes.size() - 2;
    int goalNode_i = nodes.size() - 1;

    // Hypothetical robot is not moving at start
    int currentNode = startNode_i;
    path.push_back(currentNode);
    visited[currentNode] = true; 
    
    int steps = 0;
    int max_steps = 300;

    while(currentNode != goalNode_i && steps < max_steps){
        Node* n = &graph[currentNode];

        int bestNode = currentNode;
        double highestValue = 0;

        for(int i = 0; i < n->neighbours_count; ++i){
            int neighbour_id = n->neighbours[i].id;

            if(!visited[neighbour_id] && g->g_pos[neighbour_id] > highestValue){
                highestValue = g->g_pos[neighbour_id];
                bestNode = neighbour_id;
            }
        }

        currentNode = bestNode;
        path.push_back(currentNode);
        visited[currentNode] = true;
        steps++;
    }
    
    return path;
}

// Function to return random double value between min and max
double randdbl(double min, double max){
    return min + (double)std::rand()/RAND_MAX * (max - min);
}

// Function to return k closest nodes to a given point
std::vector<int> topKNodes(std::vector<Node>& nodes, Coordinate point, int k){
    std::vector<PosNode> posNodes;

    for(int i = 0; i < nodes.size(); ++i){
        double distance = std::sqrt(std::pow(point.x - nodes[i].x, 2) + std::pow(point.y - nodes[i].y, 2));

        PosNode pn;
        pn.id = i;
        pn.distance = distance;
        posNodes.push_back(pn);
    }

    std::sort(posNodes.begin(), posNodes.end(), [](const PosNode& a, const PosNode& b) {
        return a.distance < b.distance;
    });

    std::vector<int> topKIds;
    for(int i = 0; i < std::min((int)posNodes.size(), k); ++i){
        topKIds.push_back(posNodes[i].id);
    }

    return topKIds;
}

// Function to make start and goal nodes of the generated graph
void mergeSGG(){
    start_candidates = topKNodes(nodes, real_start, 5);
    goal_candidates = topKNodes(nodes, real_goal, 5);

    // Start section
    Node startNode;
    startNode.id = nodes.size();
    startNode.x = real_start.x;
    startNode.y = real_start.y;
    startNode.neighbours_count = 0;

    for(int i = 0; i < start_candidates.size(); ++i){
        Edge e;
        e.id = start_candidates[i];
        e.weight = std::sqrt(std::pow(nodes[start_candidates[i]].x - real_start.x, 2) + std::pow(nodes[start_candidates[i]].y - real_start.y, 2));
        startNode.neighbours.push_back(e);
        startNode.neighbours_count++;
    }
    nodes.push_back(startNode);

    // Goal section
    Node goalNode;
    goalNode.id = nodes.size();
    goalNode.x = real_goal.x;
    goalNode.y = real_goal.y;
    goalNode.neighbours_count = 0;

    nodes.push_back(goalNode);

    for(int i = 0; i < goal_candidates.size(); ++i){

        Node* n = &nodes[goal_candidates[i]];
        Edge e;
        e.id = goalNode.id;
        e.weight = std::sqrt(std::pow(n->x - goalNode.x, 2) + std::pow(n->y - goalNode.y, 2));
        n->neighbours.push_back(e);
        n->neighbours_count++;
    }
}

// Program starts here
int main(int argc, char **argv) {

    // Initialize ROS, randomize seed, and variables
    ros::init(argc, argv, "pso_node");    
    ros::NodeHandle nh;
    std::srand(std::time(0));

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 10, startCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    ros::Subscriber sub = nh.subscribe("/segments", 10, roadmapCallback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/pso_path", 10, true);

    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        ros::spinOnce(); // Check for new messages

        if (map_received && start_received && goal_received) {
           
            mergeSGG(); // Add start and goal nodes to the graph

            Particle swarm[swarm_size];
            Global gBest;

            for(int i = 0; i < swarm_size; ++i){
                swarm[i].pos_i.resize(nodes.size());
                swarm[i].velo.resize(nodes.size());
                swarm[i].pBestPos.resize(nodes.size());
            }

            gBest.g_pos.resize(nodes.size());

            gBest.g_score = 100000.0; // distance large enough

            // PSO : Initialization section
            // Init Value (each particles) at t = 0
            for(int i = 0; i < swarm_size; ++i){
                for(int j = 0; j < nodes.size(); ++j){
                    swarm[i].pos_i[j] = randdbl(0.0,1.0);
                    swarm[i].velo[j] = 0.0;
                    swarm[i].pBestPos[j] = swarm[i].pos_i[j];
                }
                swarm[i].pBest_score = calculateFitness(&swarm[i],nodes.data());
                swarm[i].score = calculateFitness(&swarm[i],nodes.data());
            }

            // Init Value of gBest amongst all particles
            for(int i = 0 ; i < swarm_size ; ++i){
                if(swarm[i].score < gBest.g_score){
                    gBest.g_score = swarm[i].score;

                    for(int j = 0; j < nodes.size() ; ++j){
                        gBest.g_pos[j] = swarm[i].pos_i[j];
                    }
                }
            }

            // Iteration loop (start at t = 1)
            for(int i = 0; i < max_iter; ++i){
                // Particle loop
                for(int j = 0; j < swarm_size; ++j){
                    for(int k = 0; k < nodes.size(); ++k){
                        r1 = randdbl(0,1);
                        r2 = randdbl(0,1);

                        // PSO velocity and position updates
                        swarm[j].velo[k] = w*swarm[j].velo[k] + c1*(swarm[j].pBestPos[k] - swarm[j].pos_i[k])*r1 + c2*(gBest.g_pos[k] - swarm[j].pos_i[k])*r2;
                        swarm[j].pos_i[k] += swarm[j].velo[k];
                    }
                    swarm[j].score = calculateFitness(&swarm[j],nodes.data());

                    if(swarm[j].score < swarm[j].pBest_score){
                        swarm[j].pBest_score = swarm[j].score;

                        for(int k = 0; k < nodes.size(); ++k){
                            swarm[j].pBestPos[k] = swarm[j].pos_i[k];
                        }
                    }

                    if(swarm[j].score < gBest.g_score){
                        gBest.g_score = swarm[j].score;

                        for(int k = 0; k < nodes.size(); ++k){
                            gBest.g_pos[k] = swarm[j].pos_i[k];
                        }
                    }

                }
            }

            std::vector<int> final_path = calculateFinale(&gBest, nodes.data());

            for(int i = 0; i < final_path.size(); ++i){
                std::cout << "( " << nodes[final_path[i]].x << ", " << nodes[final_path[i]].y << " )" << std::endl;
            }

            // Uncomment to debug PSO

            // for(int i = 0 ; i < gBest.g_pos.size() ; ++i){
                // std::cout << gBest.g_pos[i] << std::endl;
            // }

            /*std::cout << gBest.g_score << std::endl;*/

            // Path smoothing and publishing section

            // Init path publisher
            nav_msgs::Path final_pso_path;
            final_pso_path.header.frame_id = "map";
            final_pso_path.header.stamp = ros::Time::now();

            // Storing full solution from start to goal
            std::vector<Coordinate> path_p;

            for(int i = 0; i < final_path.size(); ++i){
                Coordinate temp;
                temp.x = nodes[final_path[i]].x;
                temp.y = nodes[final_path[i]].y;
                path_p.push_back(temp);
            }

            double t_step = 0.002;

            int steps = 1.0 / t_step;

            // Path smoothing w/ Bezier curves by midpoint method
            if (path_p.size() >= 2){

                Coordinate p0 = path_p[0];
                Coordinate p1 = path_p[1];

                double mid_x = (p0.x + p1.x) / 2.0;
                double mid_y = (p0.y + p1.y) / 2.0;

                for (int j = 0; j <= steps; ++j){
                    double t = j * t_step;

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();

                    // Linear Bezier curve calculation to draw straight line from start to midpoint
                    double u = 1 - t;

                    pose.pose.position.x = u * p0.x + t * mid_x;
                    pose.pose.position.y = u * p0.y + t * mid_y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.w = 1.0; 
                    final_pso_path.poses.push_back(pose);
                }
            }

            for (int i = 1; i < path_p.size() - 1; ++i){

                
                Coordinate p0 = path_p[i - 1];
                Coordinate p1 = path_p[i];
                Coordinate p2 = path_p[i + 1];

                double m1_x = (p0.x + p1.x) / 2.0;
                double m1_y = (p0.y + p1.y) / 2.0;

                double m2_x = (p1.x + p2.x) / 2.0;
                double m2_y = (p1.y + p2.y) / 2.0;

                for (int j = 0; j <= steps; ++j){
                    double t = j * t_step;

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();

                    // Bezier curve calculation
                    double u = 1 - t;
                    double tt = t * t;
                    double uu = u * u;
                    double uut = 2 * u * t;

                    pose.pose.position.x = uu * m1_x + uut * p1.x + tt * m2_x;
                    pose.pose.position.y = uu * m1_y + uut * p1.y + tt * m2_y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.w = 1.0; 
                    final_pso_path.poses.push_back(pose);
                }
            }

            if (path_p.size() >= 2){

                Coordinate p0 = path_p[path_p.size() - 2];
                Coordinate p1 = path_p.back();

                double mid_x = (p0.x + p1.x) / 2.0;
                double mid_y = (p0.y + p1.y) / 2.0;

                for (int j = 0; j <= steps; ++j){
                    double t = j * t_step;

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();

                    // Linear Bezier curve calculation to draw straight line from start to midpoint
                    double u = 1 - t;

                    pose.pose.position.x = u * mid_x + t * p1.x;
                    pose.pose.position.y = u * mid_y + t * p1.y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.w = 1.0; 
                    final_pso_path.poses.push_back(pose);
                }
            }

            

            path_pub.publish(final_pso_path);
            start_received = false;
            goal_received = false;
        }
        rate.sleep(); // Sleep to maintain the loop rate
    }
    return 0;
}