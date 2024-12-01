#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include "unicon/in_loop_cmd_generator.h"
#include "unicon/geometry_math_type.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <unicon/ParamConfig.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Sparse>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
// #include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
// #include "ooqp_eigen_interface/ooqpei_gtest_eigen.hpp"
#include "unicon/Stop.h"

typedef Eigen::Triplet<double> Trip;
ros::Publisher rpyt_command_pub;
ros::Publisher mpc_sim_odom;
ros::Publisher rpyh_command_pub;
fullstate_t cmd_, current_;
double offset_heading_=0.0, offset_altitude_=0.0;

void CtrloopCallback(const ros::TimerEvent&);
void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void Paramcallback(unicon::ParamConfig &config, uint32_t level);
void offsets_cb(const geometry_msgs::Vector3::ConstPtr& msg);

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3);
Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current);
Eigen::Vector3f mpccontrol();

Eigen::Vector3f CtrlOmega(1.0, 1.0, 1.8); //norminal natural frequency
Eigen::Vector3f CtrlEpsilon(1, 1, 1); //tuning parameter
Eigen::Vector3f CtrlZita(1, 1, 1.1); //damping ratio
Eigen::Vector3f PosErrorAccumulated_(0, 0, 0);
Eigen::Vector3f k_I_(0.02, 0.02, 0.15);
mppi_control::InLoopCmdGen InLoopCmdGen_(0.19); //for st, hover throttle is roughly 40%
//for mini drone with dji n3, hover throttle is roughly 19%
float posErrAccLimit_ = 10.0, acc_xy_limit_=10.0;
float k_p_yaw_=0.2, k_I_yaw_=0.2;
float yawErrorAccum_ = 0, yawErrorAccumLim_ = 3.1415927;
float max_roll_pitch_angle_ = 15/180*3.1415927, max_yaw_rate_ = 40/180*3.1415927;
std::string sim_type_;
float thrust_;
//for mpc
double cost_g_ = 1.0, cost_h_ = 0.50, cost_v_ = 0.50;
double T_s = 0.1;
int horizon_N = 20;
Eigen::MatrixXd z_reference_(3*200, 1);
Eigen::VectorXd sim_state_ = Eigen::MatrixXd::Constant(6, 1, 0);
bool mpc_sim_;
float thrust_offset_=1.5, thrust_coefficient_=70, maximum_thrust_=90, minimum_thrust_=10;
bool thrust_control_=true, got_first_cmd_ = false;;
enum moveState { holdPos, yawOnly, moving, idle };
moveState droneState = idle;
bool stop_control = false;
bool hold_pos_ = true;
bool stop_srv_cb(unicon::Stop::Request &req, unicon::Stop::Response &res)
{
  stop_control = true;
  res.result = 1;
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "unicon");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); //node handle just for getting param

  dynamic_reconfigure::Server<unicon::ParamConfig> server;
  dynamic_reconfigure::Server<unicon::ParamConfig>::CallbackType externalfunction;
  externalfunction = boost::bind(&Paramcallback, _1, _2);
  server.setCallback(externalfunction);
  if (!pnh.getParam("sim_type", sim_type_)){
    ROS_WARN("Don't have sim type parameter. Exiting");
    exit(-1);
  }
  pnh.getParam("mpc_sim", mpc_sim_);
  pnh.getParam("enable_thrust_control", thrust_control_);
  double _CtrlOmega_xy, _CtrlOmega_z;
  pnh.getParam("CtrlOmega_xy", _CtrlOmega_xy);
  pnh.getParam("CtrlOmega_z", _CtrlOmega_z);
  CtrlOmega(0) = _CtrlOmega_xy;
  CtrlOmega(1) = _CtrlOmega_xy;
  CtrlOmega(2) = _CtrlOmega_z;

  double _CtrlZita_xy, _CtrlZita_z;
  pnh.getParam("CtrlZita_xy", _CtrlZita_xy);
  pnh.getParam("CtrlZita_z", _CtrlZita_z);
  CtrlZita(0) = _CtrlZita_xy;
  CtrlZita(1) = _CtrlZita_xy;
  CtrlZita(2) = _CtrlZita_z;

  pnh.getParam("PosErrorAccumulatedLimit_xyz", posErrAccLimit_);

  double _Pos_ki_xy, _Pos_ki_z;
  pnh.getParam("Pos_ki_xy", _Pos_ki_xy);
  pnh.getParam("Pos_ki_z", _Pos_ki_z);
  k_I_(0) = _Pos_ki_xy;
  k_I_(1) = _Pos_ki_xy;
  k_I_(2) = _Pos_ki_z;

  pnh.getParam("k_p_yaw_", k_p_yaw_);
  pnh.getParam("k_I_yaw_", k_I_yaw_);
  pnh.getParam("yawErrorAccumLim_", yawErrorAccumLim_);
  pnh.getParam("thrust_", thrust_);
  double tmp_angle, tmp_yaw_rate;
  pnh.getParam("max_angle_", tmp_angle);
  pnh.getParam("max_yaw_rate_", tmp_yaw_rate);
  max_yaw_rate_ = tmp_yaw_rate/180.0*3.1415927;
  max_roll_pitch_angle_ = tmp_angle/180.0*3.1415927;

  if(sim_type_!="rotors" && sim_type_!="dji" && sim_type_!="vins_dji" && sim_type_!="vins_st" && 
    sim_type_!="sim_st" && sim_type_!="vinsfusion_dji_mini" && sim_type_!="vicon_dji_mini"
    && sim_type_!="unity"){
    ROS_WARN("not good, don't know in simulation or real flight");
    exit(-1);
  }


  rpyh_command_pub = nh.advertise<sensor_msgs::Joy>("st_sdk/flight_control_setpoint_generic", 50);
  if (sim_type_!="rotors"&&sim_type_!="vins_dji"&&sim_type_!="vinsfusion_dji_mini" && 
      sim_type_!="vicon_dji_mini" && sim_type_!="unity"){
    rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
                                      "command/roll_pitch_yawrate_thrust1", 50);    
  } else {
    rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
                                      "command/roll_pitch_yawrate_thrust", 50);        
  }

  mpc_sim_odom = nh.advertise<nav_msgs::Odometry>("mpc_sim_odom", 50);
  ros::Subscriber trajectory_sub;
  if (sim_type_=="vins_dji"||sim_type_=="vinsfusion_dji_mini" || sim_type_=="vicon_dji_mini"){
    trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                                 "command/trajectory_true", 10, trajectory_cb);    
  } else {
    trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                                 "command/trajectory", 10, trajectory_cb);     
  }

  ros::ServiceServer stop_srv = nh.advertiseService("stop", stop_srv_cb);

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("vins_estimator/odometry", 10, odom_cb);

  ros::Subscriber offset_sub = nh.subscribe<geometry_msgs::Vector3>("NTU_internal/offsets", 10, offsets_cb);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/20), CtrloopCallback);

  //mpccontrol();
  ros::spin();
  //ros::spinOnce();
  return 0;
}

void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  cmd_.timestamp = ros::Time::now();
  geoVec3toEigenVec3(msg->points[0].transforms[0].translation, cmd_.pos);
  geoVec3toEigenVec3(msg->points[0].velocities[0].linear, cmd_.vel);
  geoVec3toEigenVec3(msg->points[0].accelerations[0].linear, cmd_.acc);
  Eigen::Quaternion<float> cmd_Quat(msg->points[0].transforms[0].rotation.w, 
                                    msg->points[0].transforms[0].rotation.x, 
                                    msg->points[0].transforms[0].rotation.y, 
                                    msg->points[0].transforms[0].rotation.z);
  get_dcm_from_q(cmd_.R, cmd_Quat);
  //std::cout<<"target pos x: "<<cmd_.pos(0)<<"y: "<<cmd_.pos(1)<<"z: "<<cmd_.pos(2)<<"\n";
  int _n = msg->points.size();
  for (int i=0; i<_n; i++){
    z_reference_(i*3+0, 0) = msg->points[i].transforms[0].translation.x;
    z_reference_(i*3+1, 0) = msg->points[i].transforms[0].translation.y;
    z_reference_(i*3+2, 0) = msg->points[i].transforms[0].translation.z;
  }
  //std::cout<<"z reference is"<<z_reference_<<"\n";
  ROS_INFO_ONCE("Got first trajectory message!");
  if (droneState == idle) droneState = moving;
}

void offsets_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
  offset_heading_=msg->x;
  offset_altitude_=msg->y;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_.timestamp = ros::Time::now();
  // no need to transform position
  geoPt3toEigenVec3(msg->pose.pose.position, current_.pos);

  // transforming velocity
  if (sim_type_!="rotors" && sim_type_!="vicon_dji_mini"){ //for real flight or using DJI simulator, velocity is already in global ENU frame
    geoVec3toEigenVec3(msg->twist.twist.linear, current_.vel);
  } else { //for simulation with ROTORS, convert body frame velocity to global frame
    Eigen::Quaternionf orientation_W_B(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3f velocity_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3f velocity_world = orientation_W_B *velocity_body;    
    current_.vel = velocity_world;
  }

  // transforming attitude
  if (sim_type_=="vins_dji"||sim_type_=="vins_st"){ //for real flight using dji or st drone, using vins, attitude transform
    tf::Quaternion q0(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion q1 = tf::createQuaternionFromRPY(3.1415927, 0, 0); //transform from vins imu frame to FLU frame
    tf::Quaternion qf = q0*q1;
    Eigen::Quaternion<float> current_Quat(qf.w(), qf.x(), qf.y(), qf.z());
    get_dcm_from_q(current_.R, current_Quat);
  } else if (sim_type_=="sim_st"){ //for st simulation flight, transform attitude from NED to body frame ENU/FLU
    fullstate_t current_odom_;
    Eigen::Quaternion<float> _current_Quat_ned(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    get_dcm_from_q(current_odom_.R, _current_Quat_ned);    
    Eigen::Vector3f _rpy;
    get_euler_from_R(_rpy, current_odom_.R);
    tf::Quaternion qf = tf::createQuaternionFromRPY(_rpy(0), -_rpy(1), -wrapPi(_rpy(2)-3.1415927/2)); 
    Eigen::Quaternion<float> _current_Quat_enu(qf.w(), qf.x(), qf.y(), qf.z());
    get_dcm_from_q(current_.R, _current_Quat_enu);
    //get_euler_from_R(_rpy, current_.R);
    // std::cout<< "CURRENT ENU roll"<< _rpy(0)/3.1415*180<<"  pitch"<<
    //             _rpy(1)/3.1415*180<<"  yaw"<<_rpy(2)/3.1415*180<<"\n";

    //temporarily use mpc_sim_odom message for viewing st odom
    nav_msgs::Odometry odom_sim;
    odom_sim.header.frame_id = "world";
    odom_sim.header.stamp = ros::Time::now();
    odom_sim.pose.pose.position.x = current_.pos(0);
    odom_sim.pose.pose.position.y = current_.pos(1);
    odom_sim.pose.pose.position.z = current_.pos(2);
    odom_sim.pose.pose.orientation.x = _current_Quat_enu.x();
    odom_sim.pose.pose.orientation.y = _current_Quat_enu.y();
    odom_sim.pose.pose.orientation.z = _current_Quat_enu.z();
    odom_sim.pose.pose.orientation.w = _current_Quat_enu.w();
    mpc_sim_odom.publish(odom_sim);
            
  }else { //for simulation with ROTORS or DJI simulator, attitude already in body frame ENU/FLU
    Eigen::Quaternion<float> current_Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);    
    get_dcm_from_q(current_.R, current_Quat);
  }
  ROS_INFO_ONCE("[Trajectory Tracking Control] Got first odom message!");

}

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3)
{
  eigenVec3(0) = geoVector3.x;
  eigenVec3(1) = geoVector3.y;
  eigenVec3(2) = geoVector3.z;
}

void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3)
{
  eigenVec3(0) = geoPt3.x;
  eigenVec3(1) = geoPt3.y;
  eigenVec3(2) = geoPt3.z;
}
// void geoQuatoEigenQua (geometry_msgs::Quaternion geoQua, Eigen::Quaternion& eigenQua)
// {
//   eigenQua = Quaternion(geoQua.w, geoQua.x, geoQua.y, geoQua.z);
// }

Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current)
{ 
  PosErrorAccumulated_ += (cmd.pos - current.pos);
  for (int i =0; i<=2; i++){
    if (PosErrorAccumulated_(i)>posErrAccLimit_) PosErrorAccumulated_(i) = posErrAccLimit_;
    else if (PosErrorAccumulated_(i)<-posErrAccLimit_) PosErrorAccumulated_(i) = -posErrAccLimit_;
  }
  double position_control = 1.0;
  double velocity_control = 1.0;
  if (cmd.pos.norm()<0.01) {
    position_control = 0.0;
    if (cmd.vel.norm()<0.01) {
      velocity_control = 0.0;
    }
    
  }
  Eigen::Vector3f tarAcc(0,0,0);

  tarAcc(0) = pow(CtrlOmega(0)/CtrlEpsilon(0), 2)*(cmd.pos(0)-current.pos(0))*position_control +
             2*CtrlZita(0)*CtrlOmega(0)/CtrlEpsilon(0)*(cmd.vel(0)-current.vel(0))*velocity_control + 
             cmd.acc(0) + k_I_(0)*PosErrorAccumulated_(0);
  tarAcc(1) = pow(CtrlOmega(1)/CtrlEpsilon(1), 2)*(cmd.pos(1)-current.pos(1))*position_control +
             2*CtrlZita(1)*CtrlOmega(1)/CtrlEpsilon(1)*(cmd.vel(1)-current.vel(1))*velocity_control + 
             cmd.acc(1) + k_I_(1)*PosErrorAccumulated_(1);
  tarAcc(2) = pow(CtrlOmega(2)/CtrlEpsilon(2), 2)*(cmd.pos(2)-current.pos(2))*position_control +
             2*CtrlZita(2)*CtrlOmega(2)/CtrlEpsilon(2)*(cmd.vel(2)-current.vel(2))*velocity_control + 
             cmd.acc(2) +  + k_I_(2)*PosErrorAccumulated_(2) + ONE_G;

  // std::cout<<"velocity command"<<cmd.vel<<"\n"; 
  // std::cout<<"current velocity"<<current.vel<<"\n"; 
  if (tarAcc(0) >= acc_xy_limit_) tarAcc(0) = acc_xy_limit_;
  else if (tarAcc(0) < -acc_xy_limit_) tarAcc(0) = -acc_xy_limit_;
  if (tarAcc(1) >= acc_xy_limit_) tarAcc(1) = acc_xy_limit_;
  else if (tarAcc(1) < -acc_xy_limit_) tarAcc(1) = -acc_xy_limit_;
  if (tarAcc(2) >= 2.0*ONE_G) tarAcc(2) = 2.0*ONE_G;
  else if (tarAcc(2) < 0.3*ONE_G) tarAcc(2) = 0.3*ONE_G;
  // std::cout <<"------------------cmd-----------------\n ";
  // std::cout<<"target pos x: "<<cmd.pos(0)<<"y: "<<cmd.pos(1)<<"z: "<<cmd.pos(2)<<"\n";
  // std::cout<<"target vel x: "<<cmd.vel(0)<<"y: "<<cmd.vel(1)<<"z: "<<cmd.vel(2)<<"\n";
  // std::cout<<"target acc x: "<<cmd.acc(0)<<"y: "<<cmd.acc(1)<<"z: "<<cmd.acc(2)<<"\n";
  // std::cout<<"\n";
  // std::cout <<"------------------current-----------------\n ";
  // std::cout<<"current pos x: "<<current.pos(0)<<"y: "<<current.pos(1)<<"z: "<<current.pos(2)<<"\n";
  // std::cout<<"current vel x: "<<current.vel(0)<<"y: "<<current.vel(1)<<"z: "<<current.vel(2)<<"\n";
  // std::cout<<"\n";

  // if (std::isnan(tarAcc(0))||std::isnan(tarAcc(1))||std::isnan(tarAcc(2))){
  //   tarAcc.setZero();
  //   std::cout<<"target acc becomes NAN!!\n";
  // }
  if (sim_type_=="sim_st"){
    std::cout <<"------------------target acc final-----------------\n ";
    std::cout<<"trarget acc x: "<<tarAcc(0)<<"y: "<<tarAcc(1)<<"z: "<<tarAcc(2)<<"\n";
    std::cout<<"\n";
  }
  return tarAcc;
}

void CtrloopCallback(const ros::TimerEvent&)
{
  if (droneState == idle) return;  
  if (fabs((ros::Time::now() - cmd_.timestamp).toSec()) > 0.1 && droneState!=holdPos)
  {
    droneState = holdPos;
    cmd_.pos = current_.pos;
    cmd_.R = current_.R;
    cmd_.vel.setZero();
    cmd_.acc.setZero();
  } else if (fabs((ros::Time::now() - cmd_.timestamp).toSec()) < 0.1 && droneState != yawOnly
    && cmd_.pos.norm()<0.01 && cmd_.vel.norm()<0.01 && cmd_.acc.norm()<0.01){ //only attitude change requested
    droneState = yawOnly;
    cmd_.pos = current_.pos;
  } else if (fabs((ros::Time::now() - cmd_.timestamp).toSec()) < 0.1 
    && ((cmd_.pos - current_.pos).norm()>0.05 || cmd_.vel.norm()>0.01 || cmd_.acc.norm()>0.01)) {
    droneState = moving;
  }

  if ((ros::Time::now()-current_.timestamp).toSec()<=0.10 && 
    !std::isnan(cmd_.pos(0)) && !std::isnan(current_.pos(0))){
    //do control
    ROS_INFO_ONCE("started control!");
    Eigen::Vector3f tarAcc_ = prtcontrol(cmd_, current_);
    //Eigen::Vector3f tarAcc_ = mpccontrol();
    /*enu to ned */
    Eigen::Vector3f RefAtti_ned, tarAtti_ned, currentAtti_ned;
    Eigen::Vector3f tarAcc_ned_;
    tarAcc_ned_(0) = tarAcc_(0);
    tarAcc_ned_(1) = -tarAcc_(1);
    tarAcc_ned_(2) = -tarAcc_(2);
    Eigen::Matrix3f cmd_R_ned, current_R_ned;
    cmd_R_ned = cmd_.R;
    cmd_R_ned(0,1) = -cmd_.R(0,1);
    cmd_R_ned(0,2) = -cmd_.R(0,2);
    cmd_R_ned(1,0) = -cmd_.R(1,0);
    cmd_R_ned(2,0) = -cmd_.R(2,0);    
    current_R_ned = current_.R;
    current_R_ned(0,1) = -current_.R(0,1);
    current_R_ned(0,2) = -current_.R(0,2);
    current_R_ned(1,0) = -current_.R(1,0);
    current_R_ned(2,0) = -current_.R(2,0);

    //testing code
    Eigen::Vector3f _rpy_c, _rpy_d;
    get_euler_from_R(_rpy_c, current_.R);
    get_euler_from_R(_rpy_d, cmd_.R);
    // std::cout<< "CURRENT  roll"<< _rpy_c(0)/3.1415*180<<"  pitch"<<_rpy_c(1)/3.1415*180<<"  yaw"<<_rpy_c(2)/3.1415*180<<"\n";
    //std::cout<< "DESIRED  roll"<< _rpy_d(0)/3.1415*180<<"  pitch"<<_rpy_d(1)/3.1415*180<<"  yaw"<<_rpy_d(2)/3.1415*180<<"\n";

    get_euler_from_R<float>(RefAtti_ned, cmd_R_ned);
    get_euler_from_R(currentAtti_ned, current_R_ned);
    //std::cout<<"current attitide roll: "<<currentAtti_ned(0)/3.14159*180<<"pitch: "<<currentAtti_ned(1)/3.14159*180<<"yaw: "<<currentAtti_ned(2)/3.14159*180<<"\n";
    mppi_control::InLoopCmdGen::drone_cmd_t in_loop_cmd = InLoopCmdGen_.cal_R_T(tarAcc_ned_, current_R_ned, currentAtti_ned(2));
    get_euler_from_R(tarAtti_ned, in_loop_cmd.R);
    //std::cout<<"command attitide roll: "<<tarAtti_ned(0)/3.14159*180<<"pitch: "<<tarAtti_ned(1)/3.14159*180<<"yaw: "<<tarAtti_ned(2)/3.14159*180<<"\n";
    // std::cout<<"command thrust: "<<in_loop_cmd.T<<"\n";
    mav_msgs::RollPitchYawrateThrust rpyrt_msg;
    float roll_tmp=tarAtti_ned(0);
    float pitch_tmp=tarAtti_ned(1);
    if (roll_tmp>max_roll_pitch_angle_) roll_tmp = max_roll_pitch_angle_;
    else if (roll_tmp<-max_roll_pitch_angle_) roll_tmp = -max_roll_pitch_angle_;
    if (pitch_tmp>max_roll_pitch_angle_) pitch_tmp = max_roll_pitch_angle_;
    else if (pitch_tmp<-max_roll_pitch_angle_) pitch_tmp = -max_roll_pitch_angle_;

    rpyrt_msg.roll = roll_tmp;
    rpyrt_msg.pitch = -pitch_tmp;
    yawErrorAccum_ += (-RefAtti_ned(2) + currentAtti_ned(2));
    if (yawErrorAccum_>yawErrorAccumLim_) yawErrorAccum_ = yawErrorAccumLim_;
    else if (yawErrorAccum_<-yawErrorAccumLim_) yawErrorAccum_ = -yawErrorAccumLim_;

    float yaw_rate_tmp=wrapPi(-RefAtti_ned(2) + currentAtti_ned(2)) * k_p_yaw_ + yawErrorAccum_*k_I_yaw_;
    if (yaw_rate_tmp>max_yaw_rate_) yaw_rate_tmp = max_yaw_rate_;
    else if (yaw_rate_tmp<-max_yaw_rate_) yaw_rate_tmp = -max_yaw_rate_;

    rpyrt_msg.yaw_rate = yaw_rate_tmp;
    rpyrt_msg.thrust.x = 0;
    rpyrt_msg.thrust.y = 0;
    if (sim_type_=="rotors"){  //for simulation with ROTORS only
      rpyrt_msg.thrust.z = in_loop_cmd.T*thrust_; //70 for unity, 43.75 for rotors      
    }else if (sim_type_=="unity") {
      rpyrt_msg.thrust.z = in_loop_cmd.T*70; //70 for unity, 43.75 for rotors 
    }else if (sim_type_=="vins_dji"||(sim_type_=="vinsfusion_dji_mini"&&!thrust_control_)||
      (sim_type_=="vicon_dji_mini"&&!thrust_control_)){
      //rpyrt_msg.thrust.z= cmd_.pos(2); //for dji m600
      rpyrt_msg.thrust.z= 1.5*(cmd_.pos(2)-current_.pos(2))+cmd_.vel(2); //for dji m600 velocity control instead
    }else { //for mini drone dji
      float _thrust_cmd=thrust_offset_ + in_loop_cmd.T*thrust_coefficient_;
      if(_thrust_cmd < minimum_thrust_){
        ROS_WARN("Throttle command is below minimum.. set to minimum");
        _thrust_cmd = minimum_thrust_;
      }
      if(_thrust_cmd > maximum_thrust_){
        ROS_WARN("Throttle command is too high.. set to max");
        _thrust_cmd = maximum_thrust_;
      }      
      rpyrt_msg.thrust.z = _thrust_cmd;
    }

    if (stop_control)
    {
      // printf("STOPPING CONTROL\n");
      rpyrt_msg.roll = 0.0;
      rpyrt_msg.pitch = 0.0;      
      rpyrt_msg.yaw_rate = 0.0;
      rpyrt_msg.thrust.z = 0.0;
    }

    if (sim_type_!="vins_st"&&sim_type_!="sim_st"){
      rpyt_command_pub.publish(rpyrt_msg);
    } else if (sim_type_=="sim_st"){ //set up the control interface for st
      sensor_msgs::Joy rpyh_msg;
      rpyh_msg.axes.push_back(tarAtti_ned(0));
      rpyh_msg.axes.push_back(tarAtti_ned(1));
      rpyh_msg.axes.push_back(wrapPi(RefAtti_ned(2)+3.1415927/2));
      rpyh_msg.axes.push_back(cmd_.pos(2));
      float _flag=19;
      rpyh_msg.axes.push_back(_flag);
      rpyh_command_pub.publish(rpyh_msg);
    } else if (sim_type_=="vins_st") {
      sensor_msgs::Joy rpyh_msg;
      rpyh_msg.axes.push_back(tarAtti_ned(0));
      rpyh_msg.axes.push_back(tarAtti_ned(1));
      rpyh_msg.axes.push_back(wrapPi(RefAtti_ned(2)+offset_heading_));
      rpyh_msg.axes.push_back(cmd_.pos(2)+offset_altitude_);
      float _flag=19;
      rpyh_msg.axes.push_back(_flag);
      rpyh_command_pub.publish(rpyh_msg);      
    }
  } else if ((ros::Time::now()-current_.timestamp).toSec()<=0.11 && sim_type_=="vins_st"){
      Eigen::Vector3f currentAtti_ned; 
      Eigen::Matrix3f current_R_ned; 
      current_R_ned = current_.R;
      current_R_ned(0,1) = -current_.R(0,1);
      current_R_ned(0,2) = -current_.R(0,2);
      current_R_ned(1,0) = -current_.R(1,0);
      current_R_ned(2,0) = -current_.R(2,0);
      get_euler_from_R(currentAtti_ned, current_R_ned);

      sensor_msgs::Joy rpyh_msg;
      rpyh_msg.axes.push_back(00);
      rpyh_msg.axes.push_back(00);
      rpyh_msg.axes.push_back(wrapPi(currentAtti_ned(2)+offset_heading_));
      rpyh_msg.axes.push_back(current_.pos(2)+offset_altitude_);
      float _flag=19;
      rpyh_msg.axes.push_back(_flag);
      rpyh_command_pub.publish(rpyh_msg);

  } else if ((ros::Time::now()-current_.timestamp).toSec()<=0.11 && sim_type_=="sim_st"){
      Eigen::Vector3f currentAtti_ned; 
      Eigen::Matrix3f current_R_ned; 
      current_R_ned = current_.R;
      current_R_ned(0,1) = -current_.R(0,1);
      current_R_ned(0,2) = -current_.R(0,2);
      current_R_ned(1,0) = -current_.R(1,0);
      current_R_ned(2,0) = -current_.R(2,0);
      get_euler_from_R(currentAtti_ned, current_R_ned);

      sensor_msgs::Joy rpyh_msg;
      rpyh_msg.axes.push_back(00);
      rpyh_msg.axes.push_back(00);
      rpyh_msg.axes.push_back(wrapPi(currentAtti_ned(2)+3.1415927/2));
      rpyh_msg.axes.push_back(current_.pos(2));
      float _flag=19;
      rpyh_msg.axes.push_back(_flag);
      rpyh_command_pub.publish(rpyh_msg);

  } else if(mpc_sim_){
    mpccontrol();
  } else ROS_INFO_ONCE("ref trajectory lagging behind odom time!");
}


void Paramcallback(unicon::ParamConfig &config, uint32_t level)
{

    CtrlOmega(0) = config.CtrlOmega_xy;
    CtrlOmega(1) = config.CtrlOmega_xy;
    CtrlOmega(2) = config.CtrlOmega_z;
    CtrlZita(0) = config.CtrlZita_xy;
    CtrlZita(1) = config.CtrlZita_xy;
    CtrlZita(2) = config.CtrlZita_z;

    posErrAccLimit_ = config.PosErrorAccumulatedLimit_xyz;
    k_I_(0) = config.Pos_ki_xy;
    k_I_(1) = config.Pos_ki_xy;
    k_I_(2) = config.Pos_ki_z;
    k_p_yaw_ = config.k_p_yaw_;
    k_I_yaw_ = config.k_I_yaw_;
    cost_h_ = config.mpc_accel_weight_;
    thrust_ = config.thrust_;
    cost_v_ =  config.mpc_vel_weight_;
    yawErrorAccumLim_ = config.yawErrorAccumLim_;
    max_roll_pitch_angle_ = config.max_angle_/180.0*3.1415927;
    max_yaw_rate_ = config.max_yaw_rate_/180.0*3.1415927;

    thrust_offset_ = config.dji_thrust_offset;  //mq
    thrust_coefficient_ = config.dji_thrust_coefficient;
    maximum_thrust_ = config.dji_maximum_thrust;
    minimum_thrust_ = config.dji_minimum_thrust;
}

Eigen::Vector3f mpccontrol()
{ 
  Eigen::SparseMatrix<double, Eigen::RowMajor> A, B, C, G, H;
  A.resize(6*horizon_N, 6);
  B.resize(6*horizon_N, 3*(horizon_N));
  C.resize(6*horizon_N, 6*horizon_N);
  G.resize(6*horizon_N, 6*horizon_N);
  H.resize(3*horizon_N, 3*horizon_N);

  std::vector<Trip> tripletListA, tripletListB, tripletListC, tripletListG, tripletListH;
  tripletListA.reserve(10*horizon_N);
  tripletListB.reserve((horizon_N*horizon_N+horizon_N)*6);
  tripletListC.reserve(6*horizon_N);
  tripletListG.reserve(6*horizon_N);
  tripletListH.reserve(3*horizon_N);

  //set up matrix A
  for (int i=0; i<horizon_N; i++) {
    for (int j=0; j<6; j++){
      tripletListA.push_back(Trip(i*6+j, j, 1));
    }
    for (int j=0; j<3; j++){
      tripletListA.push_back(Trip(i*6+j, 3+j, T_s*(i+1)));
    }
  }
  A.setFromTriplets(tripletListA.begin(), tripletListA.end());

  //set up matrix B
  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<=i; j++){
      for (int k=0; k<3; k++){
        tripletListB.push_back(Trip(i*6+k, j*3+k, T_s*T_s*(i-j)+T_s*T_s/2));
      }
      for (int k=3; k<6; k++){
        tripletListB.push_back(Trip(i*6+k, j*3+k-3, T_s));
      }
    }
  }
  B.setFromTriplets(tripletListB.begin(), tripletListB.end());

  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<6; j++){
      tripletListC.push_back(Trip(i*6+j, i*6+j, 1));
    }
  }
  C.setFromTriplets(tripletListC.begin(), tripletListC.end());

  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<3; j++){
      tripletListG.push_back(Trip(i*6+j, i*6+j, cost_g_));
      tripletListG.push_back(Trip(i*6+j+3, i*6+j+3, cost_v_));
      tripletListH.push_back(Trip(i*3+j, i*3+j, cost_h_));
    }
  }
  G.setFromTriplets(tripletListG.begin(), tripletListG.end());
  H.setFromTriplets(tripletListH.begin(), tripletListH.end());

  Eigen::SparseMatrix<double, Eigen::RowMajor> _z_r;
  _z_r.resize(6*horizon_N, 1);

  if (!mpc_sim_){
    for (int i=0; i<horizon_N; i++){
      for (int j=0; j<3; j++){
        _z_r.insert(i*6+j, 0) = z_reference_(i*3+j, 0);
        _z_r.insert(i*6+j+3, 0) = 0.0;
      }
        // _z_r.insert(i*6+0, 0) = 4.0;     //for fixed point simulation
        // _z_r.insert(i*6+1, 0) = 0.0;
        // _z_r.insert(i*6+2, 0) = 4.0;   
    }
  }else {
    for (int i=0; i<horizon_N; i++){
      _z_r.insert(i*6+1, 0) = 100.0;
    }
  }

  std::cout<<_z_r<<"\n";
  Eigen::SparseMatrix<double, Eigen::RowMajor> _x_0;
  _x_0.resize(6, 1);
  if (!mpc_sim_){
    _x_0.insert(0, 0) = current_.pos(0);
    _x_0.insert(1, 0) = current_.pos(1);
    _x_0.insert(2, 0) = current_.pos(2);
    _x_0.insert(3, 0) = current_.vel(0);
    _x_0.insert(4, 0) = current_.vel(1);
    _x_0.insert(5, 0) = current_.vel(2);   
  } else {
    _x_0.insert(0, 0) = sim_state_(0);
    _x_0.insert(1, 0) = sim_state_(1);
    _x_0.insert(2, 0) = sim_state_(2);
    _x_0.insert(3, 0) = sim_state_(3);
    _x_0.insert(4, 0) = sim_state_(4);
    _x_0.insert(5, 0) = sim_state_(5);  
  }


   // std::cout<<"big A matrix is"<<A<<"\n";
   // std::cout<<"big B matrix is"<<B<<"\n";
   // std::cout<<"big C matrix is"<<C<<"\n";
   // std::cout<<"big G matrix is"<<G<<"\n";
   // std::cout<<"big H matrix is"<<H<<"\n";

  Eigen::SparseMatrix<double, Eigen::RowMajor> Q = B.transpose()*C.transpose()*G*C*B + H;
  Eigen::MatrixXd bigB1 = _x_0.transpose()*A.transpose()*C.transpose()*G*C*B;
  Eigen::MatrixXd bigB2 = -_z_r.transpose()*G*C*B;
  //Eigen::MatrixXd c = bigB1 + bigB2;
  //Eigen::VectorXd c = Eigen::VectorXd::Zero(3*horizon_N);
  Eigen::VectorXd c = 2* (bigB1 + bigB2).transpose();
  // std::cout<<"big A matrix is"<<Q<<"\n";
  //std::cout<<"big B matrix is"<<c<<"\n";

  Eigen::SparseMatrix<double, Eigen::RowMajor> E;
  E.resize(3*horizon_N, 3*horizon_N);
  for (int i=0; i<3*horizon_N; i++){
    E.insert(i, i) = 1;
    //E.insert(i+3*horizon_N, i) = -1;
  }
  Eigen::VectorXd f(3*horizon_N), g(3*horizon_N);
  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<2; j++){
      f(i*3+j) = -acc_xy_limit_;
      g(i*3+j) = acc_xy_limit_;
    }
      f(i*3+2) = -0.7*ONE_G;
      g(i*3+2) = 1.0*ONE_G;
  }
  //set up ooqp solver
  Eigen::SparseMatrix<double, Eigen::RowMajor> a;
  //a.resize(3*horizon_N, 3*horizon_N);
  Eigen::VectorXd b;//(3*horizon_N);
  Eigen::VectorXd l = Eigen::VectorXd::Constant(3*horizon_N, -10000000);
  Eigen::VectorXd u = Eigen::VectorXd::Constant(3*horizon_N, std::numeric_limits<double>::max());
  Eigen::VectorXd x;
  //bool result = ooqpei::OoqpEigenInterface::solve(Q, c, a, b, E, f, g, l, u, x);
  //Eigen::MatrixXd U_k = -bigB*bigA.inverse();
  //std::cout<<"solution is "<<x<<"\n";

  Eigen::Vector3f tarAcc;
  tarAcc(0) = x(0);
  tarAcc(1) = x(1);
  tarAcc(2) = x(2)+ONE_G;

  if (mpc_sim_){
    //tarAcc(2) = tarAcc(2)-ONE_G;
    Eigen::VectorXd tarAccd(3);
    tarAccd(0) =(double)tarAcc(0);
    tarAccd(1) =(double)tarAcc(1);
    tarAccd(2) =(double)(tarAcc(2)-ONE_G);
    Eigen::MatrixXd smallA = Eigen::MatrixXd::Identity(6, 6);
    smallA.topRightCorner(3,3) = Eigen::MatrixXd::Identity(3, 3)*T_s;
    Eigen::MatrixXd smallB(6,3);
    smallB<< T_s*T_s/2, 0, 0, 0, T_s*T_s/2, 0, 0, 0, T_s*T_s/2,
             T_s, 0, 0, 0, T_s, 0, 0, 0, T_s;
    sim_state_ = smallA*sim_state_ + smallB*tarAccd;
    nav_msgs::Odometry odom_sim;
    odom_sim.header.stamp = ros::Time::now();
    odom_sim.pose.pose.position.x = sim_state_(0);
    odom_sim.pose.pose.position.y = sim_state_(1);
    odom_sim.pose.pose.position.z = sim_state_(2);
    mpc_sim_odom.publish(odom_sim);
  }
  if (sim_type_!="sim_st"){
    std::cout<<"target acc x: "<<tarAcc(0)<<" y: "<<tarAcc(1)<<" z: "<<tarAcc(2)<<"\n";
  }
  return tarAcc;
}
