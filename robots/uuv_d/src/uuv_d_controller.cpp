#include <uuv_d/uuv_d_controller.h>

namespace aerial_robot_control
{
UUVDController::UUVDController()
  : PoseLinearController(),
    torque_allocation_matrix_inv_pub_stamp_(0.0),
    torque_allocation_matrix_inv_pub_interval_(0.05),
    target_roll_(0.0),
    target_pitch_(0.0)
{
}

void UUVDController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  q_mat_.resize(6, motor_num_);
  q_mat_inv_.resize(motor_num_, 6);
  target_base_thrust_.resize(motor_num_);

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  debug_wrench_pubs_.resize(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    // トピック名を "motor_0/wrench", "motor_1/wrench" のように設定
    std::string topic_name = "motor_" + std::to_string(i) + "/wrench";
    debug_wrench_pubs_.at(i) = nh_.advertise<geometry_msgs::WrenchStamped>(topic_name, 1);
  }
}
void UUVDController::publishDebugWrench()
{
  for (int i = 0; i < motor_num_; i++)
  {
    geometry_msgs::WrenchStamped wrench_msg;
    
    // ヘッダー情報の設定
    wrench_msg.header.stamp = ros::Time::now();
    // ※注意：ここのフレーム名はURDF/TFツリーで定義されているモーターのリンク名と完全に一致させる必要があります。
    wrench_msg.header.frame_id = "uuv_d/thrust" + std::to_string(i+1); 

    // 力の設定（Z軸方向に推力が発生すると仮定）
    wrench_msg.wrench.force.x = 0.0;
    wrench_msg.wrench.force.y = 0.0;
    wrench_msg.wrench.force.z = target_base_thrust_.at(i);

    // トルクの設定（反トルクも可視化したい場合はZ軸に値を入れますが、今回は推力のみとします）
    wrench_msg.wrench.torque.x = 0.0;
    wrench_msg.wrench.torque.y = 0.0;
    wrench_msg.wrench.torque.z = 0.0;

    // 配信
    debug_wrench_pubs_.at(i).publish(wrench_msg);
  }
}
void UUVDController::controlCore()
{
  PoseLinearController::controlCore();
  target_roll_ = target_rpy_.x();
  target_pitch_ = target_rpy_.y();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv = 1.0 / robot_model_->getMass();
  Eigen::MatrixXd q_mat = robot_model_->calcWrenchMatrixOnCoG();
  q_mat_.topRows(3) = mass_inv * q_mat.topRows(3);
  q_mat_.bottomRows(3) = inertia_inv * q_mat.bottomRows(3);
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

  Eigen::Matrix<double, 6, 1> target_wrench_acc;
  target_wrench_acc << target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z(),
      pid_controllers_.at(ROLL).result(), pid_controllers_.at(PITCH).result(), pid_controllers_.at(YAW).result();
  Eigen::VectorXd lambda = q_mat_inv_ * target_wrench_acc;

  for (int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = lambda(i);
    }

  pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();

  pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();
}

void UUVDController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
}

void UUVDController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();
  sendTorqueAllocationMatrixInv();
  publishDebugWrench();
  
}

void UUVDController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = 0.0;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void UUVDController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void UUVDController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = 0.0;
  rpy_gain_msg.motors.at(0).roll_i = 0.0;
  rpy_gain_msg.motors.at(0).roll_d = 0.0;
  rpy_gain_msg.motors.at(0).pitch_p = 0.0;
  rpy_gain_msg.motors.at(0).pitch_i = 0.0;
  rpy_gain_msg.motors.at(0).pitch_d = 0.0;
  rpy_gain_msg.motors.at(0).yaw_d = 0.0;
  rpy_gain_pub_.publish(rpy_gain_msg);
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UUVDController, aerial_robot_control::ControlBase);
