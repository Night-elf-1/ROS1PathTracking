#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cpprobotics_types_double.h"
#include "frenet_path_double.h"
#include "quintic_polynomial_double.h"

using namespace std;
using namespace cpprobotics;

// param
double T = 50;  // sim Time
#define DT 0.1

cpprobotics::FrenetPath fp;
nav_msgs::Path path;

ros::Publisher path_pub_;

void calc_frenet_paths() {
  cpprobotics::QuinticPolynomial lon_qp(x_start[0], x_start[1], x_start[2], x_end[0],
                           x_end[1], x_end[2], T);
  cpprobotics::QuinticPolynomial lat_qp(y_start[0], y_start[1], y_start[2], y_end[0],
                           y_end[1], y_end[2], T, xend);
  for (double t = 0; t < T; t += DT) {
    double x = lon_qp.calc_point_x(t);
    double xd = lon_qp.calc_point_xd(t);
    double xdd = lon_qp.calc_point_xdd(t);

    fp.t.push_back(t);
    fp.x.push_back(x);
    fp.x_d.push_back(xd);
    fp.x_dd.push_back(xdd);

    double y_x_t = lat_qp.calc_point_y_x(x);
    double y_x_d = lat_qp.calc_point_y_x_d(x);
    double y_x_t_d = lat_qp.calc_point_y_t_d(y_x_d, xd);

    double y_x_dd = lat_qp.calc_point_y_x_dd(x);
    double y_x_t_dd = lat_qp.calc_point_y_t_dd(y_x_dd, xd, y_x_d, xdd);

    fp.y.push_back(y_x_t);
    fp.y_d.push_back(y_x_t_d);
    fp.y_dd.push_back(y_x_t_dd);

    // fp.threat.push_back(lat_qp.calc_point_thetar(y_x_t_d, xd));

    fp.k.push_back(lat_qp.calc_point_k(y_x_dd, y_x_d));
    // fp.k.push_back(lat_qp.calc_point_k(y_x_t_dd, y_x_t_d, xdd, xd));
  }
  int num = fp.x.size();
  for (int i = 0; i < num; i++) {
    double dy = fp.y[i + 1] - fp.y[i];
    double dx = fp.x[i + 1] - fp.x[i];
    fp.threat.push_back(lat_qp.calc_point_thetar(dy, dx));
  }
  // fp.threat.push_back(fp.threat.back());
  process_path_point();
}

void process_path_point(const nav_msgs::Path &datas){
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";

    int sNum = fp.x.size();
    for (int i = 0; i < sNum; i++) {
        pose.pose.position.x = fp.x[i];
        pose.pose.position.y = fp.y[i];
        pose.pose.position.z = 0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;
        path.poses.push_back(pose);
    }
    path_pub_.publish(path);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "path_node");
    ros::NodeHandle n;

    path_pub_ = n.advertise<nav_msgs::Path>("rviz_path", 20, true);

    ros::spin();
    return 0;
}
