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

#define L 1.868

ros::Publisher stanley_;
ros::Publisher path_pub;
ros::Subscriber splinePath;
ros::Subscriber car_vel;
ros::Subscriber car_pose;
nav_msgs::Path path;

cpprobotics::Vec_f ref_x;
cpprobotics::Vec_f ref_y;
float k = 0.5;
float current_car_velocity;
int total_point_num = 0;

using namespace std;
std::array<float, 3> cal_rpy;

float cal_index_yaw(int index){
    float dy = ref_y[index + 1] - ref_y[index];
    float dx = ref_x[index + 1] - ref_x[index];
    float ref_yaw = atan2(dy, dx);
    return ref_yaw;
}

array<float, 3> cal_quaternion_2_euler(const float x, const float y, const float z, const float w){
    cal_rpy = {0, 0, 0};
    cal_rpy[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    cal_rpy[1] = asin(2 * (w * y - z * x));
    cal_rpy[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));       // car yaw
    return cal_rpy; 
}

void stanley_calculate(const geometry_msgs::PoseStamped &datas){
    auto current_front_x = datas.pose.position.x;
    auto current_front_y = datas.pose.position.y;
    auto current_front_z = 0;
    //cout << "current_x_y = " << current_front_x << " , " << current_front_y << endl;
    auto currentquaternion_x = datas.pose.orientation.x;
    auto currentquaternion_y = datas.pose.orientation.y;
    auto currentquaternion_z = datas.pose.orientation.z;
    auto currentquaternion_w = datas.pose.orientation.w;

    cal_rpy = cal_quaternion_2_euler(currentquaternion_x, currentquaternion_y, currentquaternion_z, currentquaternion_w);

    int index;
    int temp_index;
    vector<float> min_dis_point_;
    for(int i = 0; i < total_point_num; i++){
        float point_x = ref_x[i];
        float point_y = ref_y[i];
        float current_dis = sqrt(pow(point_x - current_front_x, 2) + pow(point_y - current_front_y, 2));
        min_dis_point_.push_back(current_dis);
    }
    auto smallest = min_element(min_dis_point_.begin(), min_dis_point_.end());
    index = distance(min_dis_point_.begin(), smallest);
    cout << "index = " << index << endl;
    auto min_ref_x = ref_x[index];
    auto min_ref_y = ref_y[index];

    // current err
    float err_y = sqrt(pow(min_ref_x - current_front_x, 2) + pow(min_ref_y - current_front_y, 2));
    //float err_y = min_dis_point_[index];
    cout << "lat err = " << err_y << endl;
    // calculate ref_yaw
    float ref_min_yaw = cal_index_yaw(index);
    // calculate err_yaw
    float err_yaw = ref_min_yaw - cal_rpy[2];
    // initial d
    float d_ = current_car_velocity / k;

    float dx = ref_x[index] - current_front_x;
    float dy = ref_y[index] - current_front_y;
    float L_R = dy*cos(ref_min_yaw) - dx*sin(ref_min_yaw);
    if(L_R < 0){
        err_y = -err_y;
    }
    float ey_yaw = atan2(err_y, d_);
    float total_err_yaw = err_yaw + ey_yaw;
    cout << "car front tyre theta = " << total_err_yaw << endl;
    cout << "err_yaw = " << err_yaw << endl;
    cout << "ey_yaw = " << ey_yaw << endl;
    float dl = sqrt(pow(min_ref_x - current_front_x, 2) + pow(min_ref_y - current_front_y, 2));
    if(dl > 0.02 && index <= 86){
        // publish
        geometry_msgs::Twist velocity_msg;
        velocity_msg.linear.x = 3;      // velocity = 3 m/s
        velocity_msg.angular.z = total_err_yaw;
        stanley_.publish(velocity_msg);
        //publish car pose
        geometry_msgs::PoseStamped current_pose_stamped;
        current_pose_stamped.pose.position.x = current_front_x;
        current_pose_stamped.pose.position.y = current_front_y;
        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(total_err_yaw);
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
        stanley_.publish(velocity_msg);
    }

    path_pub.publish(path);

}

void velocity_process(const geometry_msgs::TwistStamped &datas){
    current_car_velocity = datas.twist.linear.x;
    //cout << "car velocity = " <<current_car_velocity << endl;
}

void process_path_point(const nav_msgs::Path &datas){
    total_point_num = datas.poses.size();
    //cout << "total point num = " << total_point_num << endl;
    //cout << "total_point" << total_point_num << endl;
    for(int i = 0; i < total_point_num; i++){
        ref_x.push_back(datas.poses[i].pose.position.x);
        ref_y.push_back(datas.poses[i].pose.position.y);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "stanley_way");
    ros::NodeHandle n;
    // set a publish variate
    stanley_ = n.advertise<geometry_msgs::Twist>("/classb_car/cmd_vel", 20);
    // set a walked way variate  must be nav_msgs::Path type
    path_pub = n.advertise<nav_msgs::Path>("/localpath", 100, true);
    // set nav_msgs::Path info  such as stamp frame_id
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    // set a pose info
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";

    splinePath = n.subscribe("/splinepoints", 20, process_path_point);
    car_vel = n.subscribe("/classb_car/velocity", 20, velocity_process);
    car_pose = n.subscribe("/classb_car/front_pose", 20, stanley_calculate);

    ros::spin();
    return 0;
}