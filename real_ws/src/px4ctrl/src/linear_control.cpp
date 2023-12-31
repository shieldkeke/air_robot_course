#include "linear_control.h"
#include <iostream>
#include <ros/ros.h>

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
// LinearControl::calculateControl(const Desired_State_t &des,
//     const Odom_Data_t &odom,
//     const Imu_Data_t &imu, 
//     Controller_Output_t &u)
// {     
//       //linear controller
//       //compute disired acceleration

//       double p1 = des.a(0) + param_.gain.Kv0 * (des.v(0) - odom.v(0)) + param_.gain.Kp0 * (des.p(0) - odom.p(0));
//       double p2 = des.a(1) + param_.gain.Kv1 * (des.v(1) - odom.v(1)) + param_.gain.Kp1 * (des.p(1) - odom.p(1));
//       double p3 = des.a(2) + param_.gain.Kv2 * (des.v(2) - odom.v(2)) + param_.gain.Kp2 * (des.p(2) - odom.p(2)) + param_.gra ;
      
//       //supposed to be readonly, compute thrust by acc 
//       Eigen::Vector3d des_acc(p1, p2, p3);
//       u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
//       Eigen::Vector3d ea = odom.q.normalized().toRotationMatrix().eulerAngles(2,1,0);
//       double yaw = ea[0];
//       double phi_c = 1 / param_.gra * (p1 * sin(yaw) - p2 * cos(yaw));
//       double theta_c = 1 / param_.gra * (p1 * cos(yaw) + p2 * sin(yaw));
//       Eigen::Vector3d e(yaw, theta_c, phi_c);
//       auto desire_q_world = Eigen::AngleAxisd(e[0], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(e[1], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(e[2], ::Eigen::Vector3d::UnitX());
//       u.q = Eigen::Quaterniond((imu.q.normalized().toRotationMatrix())*(odom.q.normalized().toRotationMatrix().inverse()) * (desire_q_world.normalized().toRotationMatrix())).normalized();

//   //used for debug
//   debug_msg_.des_p_x = des.p(0);
//   debug_msg_.des_p_y = des.p(1);
//   debug_msg_.des_p_z = des.p(2);
  
//   debug_msg_.des_v_x = des.v(0);
//   debug_msg_.des_v_y = des.v(1);
//   debug_msg_.des_v_z = des.v(2);
  
//   debug_msg_.des_a_x = des_acc(0);
//   debug_msg_.des_a_y = des_acc(1);
//   debug_msg_.des_a_z = des_acc(2);
  
//   debug_msg_.des_q_x = u.q.x();
//   debug_msg_.des_q_y = u.q.y();
//   debug_msg_.des_q_z = u.q.z();
//   debug_msg_.des_q_w = u.q.w();
  
//   debug_msg_.des_thr = u.thrust;
  
//   // Used for thrust-accel mapping estimation
//   timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
//   while (timed_thrust_.size() > 100)
//   {
//     timed_thrust_.pop();
//   }
//   return debug_msg_;
// }

// calculate control with se3 controller version
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
    //se3 controller
    Eigen::Vector3d Kv(param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2);
    Eigen::Vector3d Kp(param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2);
    Eigen::Vector3d ea = odom.q.normalized().toRotationMatrix().eulerAngles(2,1,0);
    double yaw = ea[0];
    Eigen::Vector3d ep = des.p - odom.p;
    Eigen::Vector3d ev = des.v - odom.v;
    Eigen::Vector3d F_des = Kp.cwiseProduct(ep) + Kv.cwiseProduct(ev) + param_.mass * des.a + param_.mass * param_.gra * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d z_b = odom.q.normalized().toRotationMatrix().col(2);
    u.thrust = F_des.dot(z_b);

    Eigen::Vector3d Z_b_des = F_des / F_des.norm();
    Eigen::Vector3d X_c_des = Eigen::Vector3d(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d Y_b_des = Z_b_des.cross(X_c_des) / (Z_b_des.cross(X_c_des)).norm();
    Eigen::Vector3d X_b_des = Y_b_des.cross(Z_b_des);
    Eigen::Matrix3d R_b_des;
    R_b_des << X_b_des, Y_b_des, Z_b_des;
    Eigen::Quaterniond q_b_des(R_b_des);
    u.q = Eigen::Quaterniond((imu.q.normalized().toRotationMatrix())*(odom.q.normalized().toRotationMatrix().inverse()) * (q_b_des.normalized().toRotationMatrix())).normalized();

    //used for debug
    debug_msg_.des_p_x = des.p(0);
    debug_msg_.des_p_y = des.p(1);
    debug_msg_.des_p_z = des.p(2);
    
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);
    
    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);
    
    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();
    
    debug_msg_.des_thr = u.thrust;
    
    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100)
    {
      timed_thrust_.pop();
    }
    return debug_msg_;

}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

