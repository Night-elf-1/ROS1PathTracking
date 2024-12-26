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
#define DT 0.1


// define ros publisher and subscriber===================
ros::Publisher frenet_lqr_;
ros::Publisher path_pub_;
ros::Publisher localpath_pub_;

nav_msgs::Path path;
nav_msgs::Path localpath;

//=======================================================

// define params=========================================
cpprobotics::FrenetPath fp;
int index_ = 0;
double T = 50;  // sim Time

double xend = 80.0;
double yend = 30.0;

std::array<double, 3> x_start{0.0, 0.0, 0.0};
std::array<double, 3> x_end{xend, 0.0, 0.0};
std::array<double, 3> y_start{0.0, 0.0, 0.0};
std::array<double, 3> y_end{yend, 0.0, 0.0};

double vx = 0.01;
double vy = 0;  
double cf = -65494.663, cr = -115494.663;
double mass_fl = 500, mass_fr = 500, mass_rl = 520, mass_rr = 520;
double mass_front = mass_fl + mass_fr;
double mass_rear = mass_rl + mass_rr;
double m = mass_front + mass_rear;
double max_lateral_acceleration = 5.0;
double standstill_acceleration = -3.0;
double wheel_base = 3.8;
double a = wheel_base * (1.0 - mass_front / m);
double b = wheel_base * (1.0 - mass_rear / m);
double Iz = std::pow(a, 2) * mass_front + std::pow(b, 2) * mass_rear;
double wheel_max_degree = 0.6;
//=======================================================

// calculate siyuanshu
array<double, 3> calQuaternionToEuler(const double x, const double y,
                                           const double z, const double w) {
  std::array<double, 3> calRPY = {(0, 0, 0)};
  // roll = atan2(2(wx+yz),1-2(x*x+y*y))
  calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // pitch = arcsin(2(wy-zx))
  calRPY[1] = asin(2 * (w * y - z * x));
  // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
  calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return calRPY;
}

// path planning
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
}

// find min ref point
int findTrajref(double current_post_x, double current_post_y) {
  int numPoints = fp.x.size();  // total ref point
  // double dis_min = std::pow(fp.x[0] - current_post_x, 2) +
  //                  std::pow(fp.y[0] - current_post_y, 2);
  double dis_min = std::numeric_limits<double>::max();  // return double type max value

  int index = 0;
  for (int i = index; i < numPoints; i++) {
    double temp_dis = std::pow(fp.x[i] - current_post_x, 2) +
                      std::pow(fp.y[i] - current_post_y, 2);
    if (temp_dis < dis_min) {
      dis_min = temp_dis;
      index = i;
    }
  }
  index_ = index;
  return index;
}

// calculate err and match point k
std::array<double, 5> cal_err_k(double current_post_x, double current_post_y,
                                std::array<double, 3> calRPY) {
  std::array<double, 5> err_k;
  int index = findTrajref(current_post_x, current_post_y);      // find min ref point index
  // Eigen::Vector2f tor;
  Eigen::Matrix<double, 2, 1> tor;
  tor << cos(fp.threat[index]), sin(fp.threat[index]);
  // Eigen::Vector2f nor;
  Eigen::Matrix<double, 2, 1> nor;
  nor << -sin(fp.threat[index]), cos(fp.threat[index]);

  // Eigen::Vector2f d_err;
  Eigen::Matrix<double, 2, 1> d_err;
  d_err << current_post_x - fp.x[index], current_post_y - fp.y[index];  // input x_err and y_err

  double phi = calRPY[2];   // car yaw

  // nor.transpose()
  double ed = nor.transpose() * d_err;
  // double ed = -vx*sin();

  double es = tor.transpose() * d_err;

  double projection_point_threat = fp.threat[index] + fp.k[index] * es;     

  // double phi = fp.threat[index];
  double ed_d = vy * cos(phi - projection_point_threat) +
                vx * sin(phi - projection_point_threat);
  // double ephi = sin(phi - projection_point_threat);
  double ephi = phi - projection_point_threat;

  double s_d = (vx * cos(phi - projection_point_threat) -
                vy * sin(phi - projection_point_threat)) /
               (1 - fp.k[index] * ed);
  double phi_d = vx * fp.k[index];
  double ephi_d = phi_d - fp.k[index] * s_d;

  double projection_point_curvature = fp.k[index];

  err_k[0] = ed;
  err_k[1] = ed_d;
  err_k[2] = ephi;
  err_k[3] = ephi_d;
  err_k[4] = projection_point_curvature;

  return err_k;
}

Eigen::Matrix<double, 1, 4> cal_dlqr(Eigen::Matrix4d A,
                                     Eigen::Matrix<double, 4, 1> B,
                                     Eigen::Matrix4d Q,
                                     Eigen::Matrix<double, 1, 1> R) {

  int numLoop = 200;    // xun huan ci shu

  double minValue = 10e-10;     // yu zhi 
  Eigen::Matrix4d p_old;
  p_old = Q;

  double ts = 0.001;
  Eigen::Matrix4d eye;
  eye.setIdentity(4, 4);

  Eigen::Matrix4d Ad;
  Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);   // zhong dian oula fa
  Eigen::Matrix<double, 4, 1> Bd;
  Bd = B * ts;  // xiang qian oula

  /*************************************/
  for (int i = 0; i < numLoop; i++) {
    // B.inverse()求逆
    Eigen::Matrix4d p_new = Ad.transpose() * p_old * Ad -
                            Ad.transpose() * p_old * Bd *
                                (R + Bd.transpose() * p_old * Bd).inverse() *
                                Bd.transpose() * p_old * Ad +
                            Q;

    if (fabs((p_new - p_old).maxCoeff()) < minValue) {
      p_old = p_new;
      break;
    }
    p_old = p_new;
  }
  Eigen::Matrix<double, 1, 4> k;
  // Eigen::RowVector4f;
  k = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;

  return k;
}

Eigen::Matrix<double, 1, 4> cal_k(std::array<double, 5> err_k) {
  Eigen::Matrix4d A;    // enter A
  A << 0, 1, 0, 0, 0, (cf + cr) / (m * vx), -(cf + cr) / m,
      (a * cf - b * cr) / (m * vx), 0, 0, 0, 1, 0,
      (a * cf - b * cr) / (Iz * vx), -(a * cf - b * cr) / Iz,
      (a * a * cf + b * b * cr) / (Iz * vx);

  Eigen::Matrix<double, 4, 1> B;
  B << 0, -cf / m, 0, -a * cf / Iz;

  Eigen::Matrix4d Q;    // quan zhong Q
  // Q.setIdentity(4, 4);
  Q(0, 0) = 60;
  Q(1, 1) = 1;
  Q(2, 2) = 1;
  Q(3, 3) = 1;

  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = 35.0;

  Eigen::Matrix<double, 1, 4> k = cal_dlqr(A, B, Q, R);

  return k;
}

double cal_forword_angle(Eigen::Matrix<double, 1, 4> k,
                         std::array<double, 5> err_k) {
  double k3 = k[2];

  double kv = b * m / (cf * wheel_base) - a * m / (cr * wheel_base);

  double point_curvature = err_k[4];
  double forword_angle =
      wheel_base * point_curvature + kv * vx * vx * point_curvature -
      k3 * (b * point_curvature - a * m * vx * vx * point_curvature / cr / b);

  return forword_angle;
}

double cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle,
                 std::array<double, 5> err_k) {
  Eigen::Matrix<double, 4, 1> err;
  err << err_k[0], err_k[1], err_k[2], err_k[3];
  double angle = -k * err + forword_angle;

  return angle;
}

double limitSterringAngle(double value, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

double theta_angle(double currentPositionX, double currentPositionY,
                   std::array<double, 3> cal_RPY) {
  std::array<double, 5> err_k =
      cal_err_k(currentPositionX, currentPositionY, cal_RPY);
  Eigen::Matrix<double, 1, 4> k = cal_k(err_k);

  double forword_angle = cal_forword_angle(k, err_k);
  double tempangle = cal_angle(k, forword_angle, err_k);
  double angle =
      limitSterringAngle(tempangle, -wheel_max_degree, wheel_max_degree);
  printf("angle,forword_angle=%.3f,%.3f\n", angle, forword_angle);

  return angle;
}

// main enter 
void poseCallback(const geometry_msgs::PoseStamped &currentWaypoint) {
    double currentPositionX = currentWaypoint.pose.position.x;
    double currentPositionY = currentWaypoint.pose.position.y;
    double currentPositionZ = 0.0;

    double currentQuaternionX = currentWaypoint.pose.orientation.x;
    double currentQuaternionY = currentWaypoint.pose.orientation.y;
    double currentQuaternionZ = currentWaypoint.pose.orientation.z;
    double currentQuaternionW = currentWaypoint.pose.orientation.w;

    // calculate car yaw
    std::array<double, 3> cal_RPY =
        calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                            currentQuaternionZ, currentQuaternionW);
    // calculate car front tyre theta
    double theta = theta_angle(currentPositionX, currentPositionY, cal_RPY);

    int numpoints = fp.x.size();
    if (index_ < numpoints - 2) {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 8;
        vel_msg.angular.z = theta;
        frenet_lqr_.publish(vel_msg);

        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = currentPositionX;
        this_pose_stamped.pose.position.y = currentPositionY;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
        this_pose_stamped.pose.orientation.x = currentQuaternionX;
        this_pose_stamped.pose.orientation.y = currentQuaternionY;
        this_pose_stamped.pose.orientation.z = currentQuaternionZ;
        this_pose_stamped.pose.orientation.w = currentQuaternionW;

        this_pose_stamped.header.stamp = ros::Time::now();

        this_pose_stamped.header.frame_id = "world";
        localpath.poses.push_back(this_pose_stamped);

        localpath_pub_.publish(localpath);
    } else {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        frenet_lqr_.publish(vel_msg);
    }
}

void velocityCall(const geometry_msgs::TwistStamped &carWaypoint) {
  vx = carWaypoint.twist.linear.x;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "lqr_node");
    ros::NodeHandle n;
    frenet_lqr_ = n.advertise<geometry_msgs::Twist>("/classb_car/cmd_vel", 20);

    calc_frenet_paths();
    // this no matter
    int Num = fp.x.size();
    for (int i = 0; i < Num; i++) {
        printf("x,y,th,k,i=%.3f,%.3f,%.3f,%f,%d \n", fp.x[i], fp.y[i], fp.threat[i],
            fp.k[i], i);
    }

    // pub path
    path_pub_ = n.advertise<nav_msgs::Path>("rviz_path", 20, true);

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

    // publish car motion path
    localpath_pub_ = n.advertise<nav_msgs::Path>("local_path", 20, true);
    localpath.header.stamp = ros::Time::now();
    localpath.header.frame_id = "world";

    ros::Subscriber car_velocity = n.subscribe("/classb_car/velocity", 20, velocityCall);
    ros::Subscriber car_rear_pose = n.subscribe("/classb_car/rear_pose", 20, poseCallback);

    ros::spin();
    return 0;
}