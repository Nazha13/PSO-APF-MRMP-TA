#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <base_local_planner/goal_functions.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <pluginlib/class_list_macros.h>

struct Coordinate {
    double x, y;
};

struct robotState {
    Coordinate pos;
    double yaw;
    double v_linear;
    double v_angular;
};

namespace APF {

class APFPlanner : public nav_core::BaseLocalPlanner {
    private:
        bool initialized_ = false;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        tf2_ros::Buffer* tf_;
        costmap_2d::Costmap2DROS* costmap_ros_;

        // APF variables
        double k_att = 3.0;
        double f_att_x = 0.0;
        double f_att_y = 0.0;
        double k_rep = 3.0;
        double f_rep_x = 0.0;
        double f_rep_y = 0.0;
        double d0 = 0.85;

        double k_side = 1.5;
        int b = 0;
        double alpha = 13.7;

        double last_obs_x_ = 0.0;
        double last_obs_y_ = 0.0;
        int last_b_ = 0;

        ros::Time last_obs_time_;

        int current_progress_id = 0;

        double lookahead_dist = 0.65;
        double target_yaw = 0.0;

        int dodge_dir = 0;

        bool is_aligning_ = true;

        bool is_stuck_ = false;

    public:
        APFPlanner() {}
        APFPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
            initialize(name, tf, costmap_ros);
        }

        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
            if(!initialized_){
                ros::NodeHandle private_nh("~/" + name);

                tf_ = tf;
                costmap_ros_ = costmap_ros;

                initialized_ = true;
                ROS_INFO("APF Planner Plugin initialized.");
            }
        }

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
            global_plan_ = plan;
            current_progress_id = 0;

            is_aligning_ = true;

            is_stuck_ = false;

            return true;
        }

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
            if(global_plan_.empty()) return false;

            // reset forces
            f_att_x = 0.0; f_att_y = 0.0;
            f_rep_x = 0.0; f_rep_y = 0.0;

            // reset counter
            b = 0;

            // reset obs vars
            double min_dist_obs = d0;
            double closest_obs_x = 0.0;
            double closest_obs_y = 0.0;
            bool found_any_obs = false;

            // read the robot's current position
            geometry_msgs::PoseStamped global_pose;
            costmap_ros_->getRobotPose(global_pose);
            double robot_x = global_pose.pose.position.x;
            double robot_y = global_pose.pose.position.y;
            double robot_yaw = tf2::getYaw(global_pose.pose.orientation);

            // read the robot's goal (crucial for final check)
            double goal_x = global_plan_.back().pose.position.x;
            double goal_y = global_plan_.back().pose.position.y;
            double goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);

            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            
            if (!base_local_planner::transformGlobalPlan(
                    *tf_, 
                    global_plan_, 
                    global_pose, 
                    *(costmap_ros_->getCostmap()), 
                    costmap_ros_->getGlobalFrameID(), 
                    transformed_plan)) {
                
                ROS_WARN("APF Debugger: TF Transform failed. Halting robot for safety.");
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return false; 
            }

            double target_x = robot_x;
            double target_y = robot_y;

            int closest_index = 0;
            double min_dist = 1000000.0;

            // Read costmap_grid
            costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
            int size_x = costmap->getSizeInCellsX();
            int size_y = costmap->getSizeInCellsY();

            for(int i = current_progress_id; i < transformed_plan.size(); ++i){
                double wp_x = transformed_plan[i].pose.position.x;
                double wp_y = transformed_plan[i].pose.position.y;

                double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);

                if(dist_to_wp < min_dist){
                    min_dist = dist_to_wp;
                    closest_index = i;
                }
            }

            // Scan for obstacles
            for(int i = 0; i < size_x; ++i){
                for(int j = 0; j < size_y; ++j){

                    if(costmap->getCost(i,j) >= 50){

                        double obs_x;
                        double obs_y;
                        costmap->mapToWorld(i, j, obs_x, obs_y);

                        double dist_to_obs = std::sqrt(std::pow(obs_x - robot_x, 2) + std::pow(obs_y - robot_y, 2));

                        if(dist_to_obs < d0 && dist_to_obs > 0.35){
                            // Obstacle count
                            b++;

                            if(dist_to_obs < min_dist_obs){
                                min_dist_obs = dist_to_obs;
                                closest_obs_x = obs_x;
                                closest_obs_y = obs_y;
                                found_any_obs = true;
                            }
                        }
                    }
                }
            }

            current_progress_id = closest_index;

            int best_id = current_progress_id;
            double min_dist_lookahead = 1000000.0;
            double search_dist = lookahead_dist;

            double final_x = transformed_plan.back().pose.position.x;
            double final_y = transformed_plan.back().pose.position.y;
            double dist_to_final = std::hypot(final_x - robot_x, final_y - robot_y);

            // Dynamic lookahead
            if(found_any_obs == true){
                search_dist = lookahead_dist - 0.6; // 0.8 (normal mode), 0.4 (cautious mode)
            }

            if(dist_to_final < 0.4){
                target_x = final_x;
                target_y = final_y;
            }
            else {

                int cell_R = std::ceil((lookahead_dist / 2) / costmap->getResolution());

                for(int i = current_progress_id; i < transformed_plan.size(); ++i){
                    double wp_x = transformed_plan[i].pose.position.x;
                    double wp_y = transformed_plan[i].pose.position.y;
                    double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);
                
                    unsigned int mx, my;
                    bool is_safe_ = true;
                    if (costmap->worldToMap(wp_x, wp_y, mx, my)) {
                        for(int dx = -cell_R; dx <= cell_R; ++dx){
                            for(int dy = -cell_R; dy <= cell_R; ++dy){

                                int check_x = mx + dx;
                                int check_y = my + dy;

                                if(check_x >= 0 && check_x < size_x && check_y >= 0 && check_y < size_y){
                                    if(costmap->getCost(check_x, check_y) >= 50){
                                        is_safe_ = false;
                                        break;
                                    }
                                }
                            }
                            if(is_safe_ == false) break;
                        }
                    }

                    if(is_safe_ == false){
                        continue;
                    }

                    best_id = i;

                    if(dist_to_wp >= search_dist){
                        break;
                    }
                }
                target_x = transformed_plan[best_id].pose.position.x;
                target_y = transformed_plan[best_id].pose.position.y;
            }

            // double check_dist = 1.5; // reasonable distance to scan

            // for(int i = closest_index; i < transformed_plan.size(); ++i){ // scan for loop
            //     double wp_x = transformed_plan[i].pose.position.x;
            //     double wp_y = transformed_plan[i].pose.position.y;

            //     double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);
            //     if(dist_to_wp > check_dist) break;

            //     unsigned int mx,my;
            //     if(costmap->worldToMap(wp_x, wp_y, mx, my)){
            //         if(costmap->getCost(mx,my) >= 128){
            //             lookahead_dist = 0.6;
            //             break;
            //         }
            //     }

            // }

            // for(int i = closest_index; i < transformed_plan.size(); ++i){ // target waypoint for loop
            //     double wp_x = transformed_plan[i].pose.position.x;
            //     double wp_y = transformed_plan[i].pose.position.y;

            //     double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);
                
            //     if(dist_to_wp > lookahead_dist){
            //         target_x = wp_x;
            //         target_y = wp_y;

            //         // target_yaw = std::atan2(target_y - robot_y, target_x - robot_x);
            //         break;
            //     }
            // }

            // if (target_x == 0.0 && target_y == 0.0) {
            //     target_x = transformed_plan.back().pose.position.x;
            //     target_y = transformed_plan.back().pose.position.y;
            // }



            // Attractive force calculation
            f_att_x = k_att * (target_x - robot_x);
            f_att_y = k_att * (target_y - robot_y);

            double att_mag = std::hypot(f_att_x, f_att_y);

            double max_att_mag = k_att * lookahead_dist;

            if(att_mag > max_att_mag){
                double scale = max_att_mag / att_mag;

                f_att_x *= scale;
                f_att_y *= scale;
            }

            // Repulsive force calculation

            double memory_duration = 1.0;

            if (found_any_obs == true) {
                // 1. Update the memory with the fresh data
                last_obs_x_ = closest_obs_x;
                last_obs_y_ = closest_obs_y;
                last_b_ = b;
                last_obs_time_ = ros::Time::now();
            } 
            else {
                // 2. We are blind! Check if we saw something recently
                double time_since_last_obs = (ros::Time::now() - last_obs_time_).toSec();
    
                if (time_since_last_obs < memory_duration) {
                    // 3. Trigger Object Permanence!
                    found_any_obs = true;
                    closest_obs_x = last_obs_x_;
                    closest_obs_y = last_obs_y_;
                    b = last_b_; // Restore the side force multiplier
        
                    // Re-calculate the distance using the robot's CURRENT moving position 
                    // against the obstacle's LAST KNOWN world position
                    min_dist_obs = std::hypot(closest_obs_x - robot_x, closest_obs_y - robot_y);
        
                    // If we've driven far enough away, naturally let it go
                    if (min_dist_obs > d0) {
                        found_any_obs = false; 
                    }
                }
            }

            if(found_any_obs == true){
                if(min_dist_obs < 0.15) min_dist_obs = 0.15;

                // Repulsive force resultant calculation
                double f_rep = k_rep * ((1.0 / min_dist_obs) - (1.0 / d0)) * std::pow((1.0 / min_dist_obs), 2);
                        
                // X and Y components of repulsive force
                f_rep_x = f_rep * ((robot_x - closest_obs_x) / min_dist_obs);
                f_rep_y = f_rep * ((robot_y - closest_obs_y) / min_dist_obs);
            }

            // Side force calculation
            double f_side = k_side * b * (1.0 / alpha);

            double f_side_x = 0.0;
            double f_side_y = 0.0;
            double rep_mag = std::hypot(f_rep_x, f_rep_y);

            if(rep_mag > 0.001){
                // normalized frep components
                double nfrep_x = f_rep_x / rep_mag;
                double nfrep_y = f_rep_y / rep_mag;

                // rotation options (left or right)
                double cw_x = nfrep_y;
                double cw_y = -nfrep_x;

                double ccw_x = -nfrep_y;
                double ccw_y = nfrep_x;

                // dot operation
                double dot_cw = (cw_x * f_att_x) + (cw_y * f_att_y);
                double dot_ccw = (ccw_x * f_att_x) + (ccw_y * f_att_y);

                double tandir_x, tandir_y;

                if (dodge_dir == 0){
                    if (dot_cw > dot_ccw) {
                        dodge_dir = 1;
                    } else {
                        dodge_dir = -1;
                    }
                }

                if (dodge_dir == 1){
                    tandir_x = cw_x;
                    tandir_y = cw_y;
                }

                else if (dodge_dir == -1){
                    tandir_x = ccw_x;
                    tandir_y = ccw_y;
                }

                double f_side_cap = rep_mag * 1.5;
                if(f_side > f_side_cap) f_side = f_side_cap;

                // force calculation
                f_side_x = tandir_x * f_side;
                f_side_y = tandir_y * f_side;
            }

            // Total force calculation
            double f_total_x = f_att_x + f_rep_x + f_side_x;
            double f_total_y = f_att_y + f_rep_y + f_side_y;

            // Yaw calculation
            target_yaw = std::atan2(f_total_y, f_total_x);

            // Velocity calculation
            double max_vel = 0.4;
            double vel_x = f_total_x;
            double vel_y = f_total_y;
            double k_theta = 1.2;
            double max_angular_vel = 0.5;
            double min_angular_vel = -0.5;

            double vel_mag = std::hypot(f_total_x, f_total_y);

            if(vel_mag > max_vel){

                double scale = max_vel / vel_mag;

                vel_x *= scale;
                vel_y *= scale;
            }

            // Driving the path
            double raw_yaw_diff = target_yaw - robot_yaw;
            double yaw_diff = std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));
            double angular_vel = k_theta * yaw_diff;

            double local_vel_x = vel_x * std::cos(robot_yaw) + vel_y * std::sin(robot_yaw);
            double local_vel_y = -vel_x * std::sin(robot_yaw) + vel_y * std::cos(robot_yaw);

            double align_tolerance = 0.26; //prev 0.26 (15 degrees)

            // Alignment logic
            if(is_aligning_ == true){
                if(std::abs(yaw_diff) > align_tolerance){
                    local_vel_x = 0.0;
                    local_vel_y = 0.0;
                }
                else {
                    is_aligning_ = false;
                }
            }

            double dist_to_goal = std::sqrt(std::pow(goal_x - robot_x, 2) + std::pow(goal_y - robot_y, 2));

            // Final alignment
            if(dist_to_goal < 0.2){
                local_vel_x = 0.0;
                local_vel_y = 0.0;

                double raw_final_error = goal_yaw - robot_yaw;
                double final_error = std::atan2(std::sin(raw_final_error), std::cos(raw_final_error));
                
                angular_vel = k_theta * final_error;
            }

            if(angular_vel > max_angular_vel) angular_vel = max_angular_vel;
            if(angular_vel < min_angular_vel) angular_vel = min_angular_vel;

            // local minima fallback
            double stuck_mag_thresh = 0.05; 
            double f_mag = std::hypot(f_total_x, f_total_y);

            is_stuck_ = false;

            // Don't replan after reaching goal pose
            if(dist_to_goal < 0.4){
                is_stuck_ = false;

                cmd_vel.linear.x = local_vel_x;
                cmd_vel.angular.z = angular_vel;
                return true;
            }

            // Replan if and only if magnitude falls below threshold
            if(f_mag <= stuck_mag_thresh){
                is_stuck_ = true;
            }

            if(is_stuck_ == true){
                return false;
            }

            cmd_vel.linear.x = local_vel_x;
            cmd_vel.linear.y = local_vel_y;
            cmd_vel.angular.z = angular_vel;

            ROS_INFO("Current waypoint: %d | Fside: %.2f | Fatt: %.2f | Frep: %.2f | Force: %.2f", current_progress_id, f_side, att_mag, rep_mag, f_mag);



            return true;
        }

        bool isGoalReached(){

            // simple check so it doesn't compute before the global plan is set
            if(global_plan_.empty()) return false;
            
            // read the robot's current position
            geometry_msgs::PoseStamped global_pose;
            costmap_ros_->getRobotPose(global_pose);
            double robot_x = global_pose.pose.position.x;
            double robot_y = global_pose.pose.position.y;
            double robot_yaw = tf2::getYaw(global_pose.pose.orientation);

            // read the goal position
            double goal_x = global_plan_.back().pose.position.x;
            double goal_y = global_plan_.back().pose.position.y;
            double goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);

            double raw_yaw_diff = goal_yaw - robot_yaw;
            double yaw_diff = std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));

            double dist_to_goal = std::sqrt(std::pow(goal_x - robot_x, 2) + std::pow(goal_y - robot_y, 2));

            if(dist_to_goal < 0.2 && std::abs(yaw_diff) < 0.26) {
                return true;
            }

            return false;
        }
        

};

}

PLUGINLIB_EXPORT_CLASS(APF::APFPlanner, nav_core::BaseLocalPlanner);