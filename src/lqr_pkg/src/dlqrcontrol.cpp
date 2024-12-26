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

std::array<double, 3> x_start{0.0, 0.0, 0.0};
std::array<double, 3> x_end{xend, 0.0, 0.0};
std::array<double, 3> y_start{0.0, 0.0, 0.0};
std::array<double, 3> y_end{yend, 0.0, 0.0};

ros::Subscriber QuinticPolynomial_path;
cpprobotics::FrenetPath fp;
double vx = 0.01;
double vy = 0;
int total_point_num;
int lqr_loop = 200;
double ts = 0.001;
double minValue = 10e-10;
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

std::array<double, 3> cal_rpy;

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

    fp.k.push_back(lat_qp.calc_point_k(y_x_dd, y_x_d));
  }
  int num = fp.x.size();
  for (int i = 0; i < num; i++) {
    double dy = fp.y[i + 1] - fp.y[i];
    double dx = fp.x[i + 1] - fp.x[i];
    fp.threat.push_back(lat_qp.calc_point_thetar(dy, dx));
  }
  // fp.threat.push_back(fp.threat.back());
}

void velocityCall(const geometry_msgs::TwistStamped &datas){
    vx = datas.twist.linear.x;
}

int find_matchpoint(double x, double y){
    vector<double> min_index;
    for(int i = 0; i < fp.x.size(); i++){
        double temp_dis = sqrt(pow(x - fp.x[i], 2) + pow(y - fp.y[i], 2));
        min_index.push_back(temp_dis);
    }
    auto samllest = min_element(min_index.begin(), min_index.end());
    int index = distance(min_index.begin(), samllest);
    return index;
}

std::array<double, 5> cal_err_x(double x, double y, std::array<double, 3> car_rpy){
    std::array<double, 5> err_x;
    int index = find_matchpoint(x, y);
    // calculate lat err model
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(fp.threat[index]), sin(fp.threat[index]);
    Eigen::Matrix<double, 2, 1> nor;
    nor << -sin(fp.threat[index]), cos(fp.threat[index]);

    Eigen::Matrix<double, 2, 1> err_ed;
    err_ed << x - fp.x[index], y - fp.y[index];
    double ed = nor.transpose() * err_ed;
    double es = tor.transpose() * err_ed;

    double car_phi = car_rpy[2];
    double projection_point_threat = fp.threat[index] + fp.k[index] * es;

    double ed_d = vy * cos(car_phi - projection_point_threat) + vx * sin(car_phi - projection_point_threat);

    double err_phi = car_phi - projection_point_threat;

    double phi_d = vx * fp.k[index];
    double projection_point_threat_d = fp.k[index] * s_d;
    double err_phi_d = phi_d - projection_point_threat_d;

    double projection_point_k = fp.k[index];

    err_x[0] = ed;
    err_x[1] = ed_d;
    err_x[2] = err_phi;
    err_x[3] = err_phi_d;
    err_x[4] = projection_point_k;

    return err_x;
}

Eigen::Matrix<double, 1, 4> cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double, 4, 1> B,
                                    Eigen::Matrix4d Q, Eigen::Matrix<double, 1, 1> R){
    Eigen::Matrix4d Ad;
    Eigen::Matrix4d p_old;
    Eigen::Matrix<double, 4, 1> Bd;
    Eigen::Matrix4d eye;
    Eigen::Matrix4d p_new;
    Eigen::Matrix<double, 1, 4> k;
    eye.setIdentity(4, 4);

    p_old = Q;
    Ad = (eye - 0.5*A*ts).inverse()*(eye + 0.5*A*ts);
    Bd = B*ts;

    // calculate dlqr
    for(int i = 0; i < lqr_loop; i++){
        p_new = Ad.transpose()*p_old*Ad - Ad.transpose()*p_old*Bd*(R + Bd.transpose()*p_old*Bd).inverse()*Bd.transpose()*p_old*Ad + Q;
        if(fabs(p_new - p_old) < minValue){
            p_old = p_new;
            break;
        }
        p_old = p_new;
    }
    k = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;
    return k;
}

Eigen::Matrix<double, 1, 4> err_model_initial(std::array<double, 5> err_x){
    Eigen::Matrix4d A;
    A << 0, 1, 0, 0, 0, (cf + cr)/(m * vx), -(cf + cr)/m, (a * cf - b * cr)/(m * vx),
        0, 0, 0, 1, 0, (a * cf - b * cr)/(Iz * vx), -(a * cf - b * cr)/(Iz), (a*a*cf + b*b*cr)/(Iz * vx);
    
    Eigen::Matrix<double, 4, 1> B;
    B << 0, -cf/m, 0, -(a * cf)/Iz;

    Eigen::Matrix4d Q;
    Q(0,0) = 60;
    Q(1,1) = 1;
    Q(2,2) = 1;
    Q(3,3) = 1;
    Eigen::Matrix<double, 1, 1> R;
    R(0,0) = 35;

    Eigen::Matrix<double, 1, 4> k;
    k  = cal_dlqr(A, B, Q, R);

    return k;
}

double Feedforward_Angle(std::array<double, 5> err_x, Eigen::Matrix<double, 1, 4> k){
    k3 = k[2];

    double kv = b * m / (cf * wheel_base) - a * m / (cr * wheel_base);

    double point_curvature = err_k[4];
    double forword_angle =
        wheel_base * point_curvature + kv * vx * vx * point_curvature -
        k3 * (b * point_curvature - a * m * vx * vx * point_curvature / cr / b);

    return forword_angle;
}

double cal_u(Eigen::Matrix<double, 1, 4> k, std::array<double, 5> err_x, double feedforward_theta){
    Eigen::Matrix<double, 4, 1> err;
    err << err_x[0], err_x[1], err_x[2], err_x[3];
    double u = -k * err + feedforward_theta;
    return u;
}

double limitSterringAngle(double u, double wheel_max_degree1, double wheel_max_degree2){
    if (wheel_max_degree1 > wheel_max_degree2) {
        std::swap(wheel_max_degree1, wheel_max_degree2);
    }
    if (u < wheel_max_degree1) {
        return wheel_max_degree1;
    } else if (u > wheel_max_degree2) {
        return wheel_max_degree2;
    }
    return u;
}

void theta_cal(double x, double y, std::array<double, 3> car_rpy){
    std::array<double, 5> err_x = cal_err_x(x, y, car_rpy);
    Eigen::Matrix<double, 1, 4> k = err_model_initial(err_x);

    double feedforward_theta = Feedforward_Angle(err_x, k);
    double u = cal_u(k, err_x, feedforward_theta);
    double u = limitSterringAngle(u, -wheel_max_degree, wheel_max_degree);
    return u;
}

void poseCallback(const geometry_msgs::PoseStamped &datas){
    double current_position_x = datas.pose.position.x;
    double current_position_y = datas.pose.position.y;
    double current_position_z = 0.0;

    double current_Quaternion_x = datas.pose.orientation.x;
    double current_Quaternion_y = datas.pose.orientation.y;
    double current_Quaternion_z = datas.pose.orientation.z;
    double current_Quaternion_w = datas.pose.orientation.w;

    // calculate car rpy
    cal_rpy = calQuaternionToEuler(current_Quaternion_x, current_Quaternion_y, 
                                    current_Quaternion_z, current_Quaternion_w);
    // calculate front tyre theta
    double front_tyre_theta = theta_cal(current_position_x, current_position_y, cal_rpy);


}

int main(int argc, char **argv){
    ros::init(argc, argv, "dlqr_node");
    ros::NodeHandle n;

    // calculate path
    calc_frenet_paths();

    QuinticPolynomial_path = n.advertise("rviz_path", 20, process_path_point);

    // publish car motion path
    localpath_pub_ = n.advertise<nav_msgs::Path>("local_path", 20, true);
    localpath.header.stamp = ros::Time::now();
    localpath.header.frame_id = "world";

    ros::Subscriber car_velocity = n.subscribe("/classb_car/velocity", 20, velocityCall);
    ros::Subscriber car_rear_pose = n.subscribe("/classb_car/rear_pose", 20, poseCallback);

    ros::spin();
    return 0;
}