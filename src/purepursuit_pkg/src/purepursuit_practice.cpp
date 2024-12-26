#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cpprobotics_types.h"
#include "cubic_spline.h"
#include "geometry_msgs/PoseStamped.h"

#define base_dis 3
#define L 1.868
ros::Publisher purepursuit_;
ros::Publisher path_;
nav_msgs::Path path;
ros::Subscriber splinepath;
ros::Subscriber car_vel;
ros::Subscriber car_pose;
int point_num = 0;
cpprobotics::Vec_f ref_x;
cpprobotics::Vec_f ref_y;
float car_velocity = 0;     // base car velocity
float pre_dis = 0;          // base car pre_dis
float k = 0.1;              // base velocity coefficient
std::array<float, 3> cal_rpy;
using namespace std;

array<float, 3> cal_quaternion_2_euler(const float x, const float y, const float z, const float w){
    cal_rpy = {0, 0, 0};
    cal_rpy[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    cal_rpy[1] = asin(2 * (w * y - z * x));
    cal_rpy[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));       // car yaw
    return cal_rpy;
}

void pp_calculate(const geometry_msgs::PoseStamped &datas){
    auto current_x = datas.pose.position.x;
    auto current_y = datas.pose.position.y;
    auto current_z = 0.0;

    auto currentquaternion_x = datas.pose.orientation.x;
    auto currentquaternion_y = datas.pose.orientation.y;
    auto currentquaternion_z = datas.pose.orientation.z;
    auto currentquaternion_w = datas.pose.orientation.w;

    cal_rpy = cal_quaternion_2_euler(currentquaternion_x, currentquaternion_y,
                                    currentquaternion_z, currentquaternion_w);

    int index;
    int temp_index;
    vector<float> min_dis_point_;
    for(int i = 0; i < point_num; i++){
        float point_x = ref_x[i];
        float point_y = ref_y[i];
        float current_dis = sqrt(pow(point_x - current_x, 2) + pow(point_y - current_y, 2));
        min_dis_point_.push_back(current_dis);
    }
    // search min point
    auto smallest = min_element(min_dis_point_.begin(), min_dis_point_.end());
    index = distance(min_dis_point_.begin(), smallest);
    cout << "index = " << index << endl;
    // search Ld point
    for(int i = index; i < point_num; i++){
        float dis = sqrt(pow(ref_x[index] - ref_x[i], 2) + pow(ref_y[index] - ref_y[i], 2));
        if(dis < pre_dis){
            temp_index = i;
        }else{
            break;
        }
    }
    index = temp_index;

    // calculate alpha
    float front_alpha = atan2(ref_y[index] - current_y, ref_x[index] - current_x) - cal_rpy[2];
    // calculate distance of current point 2 target point
    float dl = sqrt(pow(ref_x[index] - current_x, 2) + pow(ref_y[index] - current_y, 2));

    if(dl > 0.2){
        // calculate fron tyer theta
        float front_theta = atan(2 * L * sin(front_alpha) / dl);
        // publish
        geometry_msgs::Twist velocity_msg;
        velocity_msg.linear.x = 3;      // velocity = 3 m/s
        velocity_msg.angular.z = front_theta;
        purepursuit_.publish(velocity_msg);
        //publish car pose
        geometry_msgs::PoseStamped current_pose_stamped;
        current_pose_stamped.pose.position.x = current_x;
        current_pose_stamped.pose.position.y = current_y;
        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(front_theta);
        current_pose_stamped.pose.orientation.x = currentquaternion_x;
        current_pose_stamped.pose.orientation.y = currentquaternion_y;
        current_pose_stamped.pose.orientation.z = currentquaternion_z;
        current_pose_stamped.pose.orientation.w = currentquaternion_w;
        current_pose_stamped.header.stamp = ros::Time::now();
        current_pose_stamped.header.frame_id = "world";
        path.poses.push_back(current_pose_stamped);
    }else{
        geometry_msgs::Twist velocity_msg;
        velocity_msg.linear.x = 0;
        velocity_msg.angular.z = 0;
        purepursuit_.publish(velocity_msg);
    }

    path_.publish(path);
}

void velocity_process(const geometry_msgs::TwistStamped &datas){
    car_velocity = datas.twist.linear.x;        // x direction speed
    pre_dis = k*car_velocity + base_dis;
}

// pre process the path point
void path_point_count(const nav_msgs::Path &datas){
    point_num = datas.poses.size();
    for(int i = 0; i < point_num; i++){
        ref_x.push_back(datas.poses[i].pose.position.x);
        ref_y.push_back(datas.poses[i].pose.position.y);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pp_practice_node");
    ros::NodeHandle n;
    purepursuit_ = n.advertise<geometry_msgs::Twist>("/classb_car/cmd_vel", 20);
    path_ = n.advertise<nav_msgs::Path>("/localpath", 100, true);

    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();

    splinepath = n.subscribe("/splinepoints", 20, path_point_count);
    car_vel = n.subscribe("/classb_car/velocity", 20, velocity_process);
    car_pose = n.subscribe("/classb_car/rear_pose", 20, pp_calculate);

    ros::spin();
    return 0;
}