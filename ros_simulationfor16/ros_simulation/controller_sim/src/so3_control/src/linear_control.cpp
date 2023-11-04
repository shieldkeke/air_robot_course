#include <linear_control.h>
#include <iostream>
#include <ros/ros.h>

LinearControl::LinearControl()
  : mass_(0.49)
  , g_(9.81)
{
}

void
LinearControl::setMass(const double mass)
{
  mass_ = mass;
}

void
LinearControl::setGravity(const double g)
{
  g_ = g;
}

// void
// LinearControl::calculateControl(const Desired_State_t &des,
//                         const Odom_Data_t &odom, 
//                         const Imu_Data_t &imu,
//                         Controller_Output_t &u,
//                         Gain gain)
// {
//   // use linear controller
//   double p1 = des.a[0] + gain.Kv0 * (des.v[0] - odom.v[0]) + gain.Kp0 * (des.p[0] - odom.p[0]);
//   double p2 = des.a[1] + gain.Kv1 * (des.v[1] - odom.v[1]) + gain.Kp1 * (des.p[1] - odom.p[1]);
//   double p3 = des.a[2] + gain.Kv2 * (des.v[2] - odom.v[2]) + gain.Kp2 * (des.p[2] - odom.p[2]);
//   double u1 = mass_ * (g_ + p3);
//   Eigen::Vector3d ea = odom.q.matrix().eulerAngles(2,1,0);
//   double yaw = ea[0];
//   double phi_c = 1 / g_ * (p1 * sin(yaw) - p2 * cos(yaw));
//   double theta_c = 1 / g_ * (p1 * cos(yaw) + p2 * sin(yaw));
//   Eigen::Vector3d e{yaw, theta_c, phi_c};
//   Eigen::Quaterniond desire_w = Eigen::AngleAxisd(e[0], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(e[1], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(e[2], ::Eigen::Vector3d::UnitX());
//   u.q = Eigen::Quaterniond((imu.q.normalized().toRotationMatrix())*(odom.q.normalized().toRotationMatrix().inverse()) * (desire_w.normalized().toRotationMatrix())).normalized();
//   u.thrust = u1;
// }

void
LinearControl::calculateControl(const Desired_State_t &des,
                        const Odom_Data_t &odom, 
                        const Imu_Data_t &imu,
                        Controller_Output_t &u,
                        Gain gain)
{
  // use se3 controller
    Eigen::Vector3d Kv(gain.Kv0, gain.Kv1, gain.Kv2);
    Eigen::Vector3d Kp(gain.Kp0, gain.Kp1, gain.Kp2);
    Eigen::Vector3d euler_angle = odom.q.normalized().toRotationMatrix().eulerAngles(2,1,0);
    double yaw = euler_angle[0];
    Eigen::Vector3d ep = des.p - odom.p;
    Eigen::Vector3d ev = des.v - odom.v;
    Eigen::Vector3d F_des = 2.5 * Kp.cwiseProduct(ep) + 2 * Kv.cwiseProduct(ev) + mass_ * des.a + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d z_b = odom.q.normalized().toRotationMatrix() * Eigen::Vector3d(0, 0, 1);
    u.thrust = F_des.dot(z_b);

    Eigen::Vector3d Z_b_des = F_des / F_des.norm();
    Eigen::Vector3d X_c_des = Eigen::Vector3d(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d Y_b_des = Z_b_des.cross(X_c_des) / (Z_b_des.cross(X_c_des)).norm();
    Eigen::Vector3d X_b_des = Y_b_des.cross(Z_b_des);
    Eigen::Matrix3d R_b_des;
    R_b_des << X_b_des, Y_b_des, Z_b_des;
    Eigen::Quaterniond q_b_des(R_b_des);
    // u.q = q_b_des;
    u.q = Eigen::Quaterniond((imu.q.normalized().toRotationMatrix())*(odom.q.normalized().toRotationMatrix().inverse()) * (q_b_des.normalized().toRotationMatrix())).normalized();
}