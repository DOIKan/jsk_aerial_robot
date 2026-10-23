#include <uuv_d/uuv_d_controller.h>
#include <XmlRpcValue.h>
#include <algorithm>
#include <cmath>

namespace aerial_robot_control
{
namespace
{
std::string cogFrameId(const ros::NodeHandle& nh)
{
  std::string ns = nh.getNamespace();
  if (!ns.empty() && ns.front() == '/') ns.erase(0, 1);
  return ns.empty() ? std::string("cog") : ns + "/cog";
}

bool xmlRpcNumber(const XmlRpc::XmlRpcValue& value, double& result)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    result = static_cast<double>(value);
  else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    result = static_cast<int>(value);
  else
    return false;
  return std::isfinite(result);
}

bool xmlRpcVector3(const XmlRpc::XmlRpcValue& value, Eigen::Vector3d& result)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
    return false;
  for (int i = 0; i < 3; ++i)
    if (!xmlRpcNumber(value[i], result[i])) return false;
  return true;
}

bool xmlRpcMatrix6(const XmlRpc::XmlRpcValue& value, Eigen::Matrix<double, 6, 6>& result)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 6)
    return false;
  for (int row = 0; row < 6; ++row)
  {
    if (value[row].getType() != XmlRpc::XmlRpcValue::TypeArray || value[row].size() != 6)
      return false;
    for (int col = 0; col < 6; ++col)
      if (!xmlRpcNumber(value[row][col], result(row, col))) return false;
  }
  return true;
}

Eigen::Matrix3d eigenRotation(const tf::Matrix3x3& rotation)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = rotation[row][col];
  return result;
}

Eigen::Matrix3d eigenRotation(const KDL::Rotation& rotation)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = rotation(row, col);
  return result;
}
}
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
  loadExternalWrenchFeedforward();

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gravity_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("debug/gravity_wrench", 1);
  feedforward_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("debug/external_wrench_feedforward", 1);
  debug_wrench_pubs_.resize(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    // トピック名を "motor_0/wrench", "motor_1/wrench" のように設定
    std::string topic_name = "motor_" + std::to_string(i) + "/wrench";
    debug_wrench_pubs_.at(i) = nh_.advertise<geometry_msgs::WrenchStamped>(topic_name, 1);
  }
}

void UUVDController::loadExternalWrenchFeedforward()
{
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.param("external_wrench_feedforward_enabled", external_wrench_feedforward_enabled_, false);
  if (!external_wrench_feedforward_enabled_) return;

  ros::NodeHandle buoy_nh(nh_, "environment/buoyancy");
  buoy_nh.param("per_link_enabled", buoyancy_feedforward_enabled_, false);
  buoy_nh.param("rho_water", rho_water_, 1000.0);
  buoy_nh.param("submerged_ratio", submerged_ratio_, 0.0);
  if (buoyancy_feedforward_enabled_)
  {
    XmlRpc::XmlRpcValue links;
    Eigen::Vector3d cob;
    if (!buoy_nh.getParam("links", links) ||
        links.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !links.hasMember("base_link") ||
        links["base_link"].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !links["base_link"].hasMember("volume") ||
        !links["base_link"].hasMember("cob") ||
        !xmlRpcNumber(links["base_link"]["volume"], buoyancy_volume_) ||
        buoyancy_volume_ <= 0.0 ||
        !xmlRpcVector3(links["base_link"]["cob"], cob))
    {
      ROS_ERROR("[UUVDController] Invalid base_link buoyancy configuration; buoyancy feedforward disabled");
      buoyancy_feedforward_enabled_ = false;
    }
    else
      buoyancy_cob_ = KDL::Vector(cob.x(), cob.y(), cob.z());
  }

  ros::NodeHandle hydro_nh(nh_, "environment/hydrodynamics");
  bool hydrodynamics_enabled = false;
  hydro_nh.param("enabled", hydrodynamics_enabled, false);
  hydro_nh.param("controller_compensation_enabled", hydrodynamic_feedforward_enabled_, false);
  hydrodynamic_feedforward_enabled_ &= hydrodynamics_enabled;
  if (hydrodynamic_feedforward_enabled_)
  {
    XmlRpc::XmlRpcValue current, links;
    if (!hydro_nh.getParam("current_velocity_world", current) ||
        !xmlRpcVector3(current, current_velocity_world_) ||
        !hydro_nh.getParam("links", links) ||
        links.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !links.hasMember("base_link") ||
        links["base_link"].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !links["base_link"].hasMember("added_mass") ||
        !links["base_link"].hasMember("quadratic_drag") ||
        !xmlRpcMatrix6(links["base_link"]["added_mass"], added_mass_) ||
        !xmlRpcMatrix6(links["base_link"]["quadratic_drag"], quadratic_drag_) ||
        (quadratic_drag_.diagonal().array() < 0.0).any())
    {
      ROS_ERROR("[UUVDController] Invalid base_link hydrodynamics configuration; fluid feedforward disabled");
      hydrodynamic_feedforward_enabled_ = false;
    }
  }
}

UUVDController::Vector6 UUVDController::computeExternalWrenchFeedforward(
  const tf::Matrix3x3& cog_rotation, const tf::Vector3& target_acceleration_cog)
{
  Vector6 compensation = Vector6::Zero();
  if (!external_wrench_feedforward_enabled_) return compensation;

  const Eigen::Matrix3d world_from_cog = eigenRotation(cog_rotation);
  compensation.head<3>() = world_from_cog.transpose() *
    Eigen::Vector3d(0.0, 0.0, robot_model_->getMass() * gravity_magnitude_);

  if (!buoyancy_feedforward_enabled_ && !hydrodynamic_feedforward_enabled_)
    return compensation;

  const auto frames = robot_model_->getSegmentsTf();
  const auto base = frames.find("base_link");
  if (base == frames.end())
  {
    ROS_ERROR_THROTTLE(1.0, "[UUVDController] Missing base_link frame for external wrench feedforward");
    return compensation;
  }
  const KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  const KDL::Frame cog_to_base = cog.Inverse() * base->second;
  const Eigen::Matrix3d cog_from_base = eigenRotation(cog_to_base.M);
  const Eigen::Vector3d base_origin_cog(cog_to_base.p.x(), cog_to_base.p.y(), cog_to_base.p.z());
  const double ratio = std::clamp(submerged_ratio_, 0.0, 1.0);

  if (buoyancy_feedforward_enabled_)
  {
    const Eigen::Vector3d buoyancy_force_cog = world_from_cog.transpose() *
      Eigen::Vector3d(0.0, 0.0, ratio * rho_water_ * 9.80665 * buoyancy_volume_);
    const KDL::Vector cob_cog_kdl = cog_to_base * buoyancy_cob_;
    const Eigen::Vector3d cob_cog(cob_cog_kdl.x(), cob_cog_kdl.y(), cob_cog_kdl.z());
    compensation.head<3>() -= buoyancy_force_cog;
    compensation.tail<3>() -= cob_cog.cross(buoyancy_force_cog);
  }

  if (hydrodynamic_feedforward_enabled_)
  {
    const Eigen::Matrix3d base_from_cog = cog_from_base.transpose();
    const Eigen::Vector3d omega_cog(omega_.x(), omega_.y(), omega_.z());
    const Eigen::Vector3d velocity_world(vel_.x(), vel_.y(), vel_.z());
    const Eigen::Vector3d velocity_cog = world_from_cog.transpose() *
      (velocity_world - current_velocity_world_);
    const Eigen::Vector3d linear_base = base_from_cog *
      (velocity_cog + omega_cog.cross(base_origin_cog));
    const Eigen::Vector3d omega_base = base_from_cog * omega_cog;
    Vector6 velocity;
    velocity << linear_base, omega_base;

    tf::Matrix3x3 target_rotation;
    target_rotation.setRPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
    const tf::Vector3 target_angular_acceleration_tf =
      cog_rotation.inverse() * target_rotation * target_ang_acc_;
    const Eigen::Vector3d angular_acceleration_cog(
      target_angular_acceleration_tf.x(), target_angular_acceleration_tf.y(),
      target_angular_acceleration_tf.z());
    const Eigen::Vector3d target_acceleration(
      target_acceleration_cog.x(), target_acceleration_cog.y(), target_acceleration_cog.z());
    const Eigen::Vector3d linear_acceleration_base = base_from_cog *
      (target_acceleration + angular_acceleration_cog.cross(base_origin_cog) +
       omega_cog.cross(omega_cog.cross(base_origin_cog))) - omega_base.cross(linear_base);
    Vector6 desired_acceleration;
    desired_acceleration << linear_acceleration_base, base_from_cog * angular_acceleration_cog;

    const Vector6 momentum = added_mass_ * velocity;
    Vector6 added_coriolis;
    added_coriolis.head<3>() = -momentum.head<3>().cross(omega_base);
    added_coriolis.tail<3>() = -momentum.head<3>().cross(linear_base) -
                               momentum.tail<3>().cross(omega_base);
    const Vector6 fluid_base = ratio *
      (added_mass_ * desired_acceleration + added_coriolis +
       quadratic_drag_ * velocity.cwiseProduct(velocity.cwiseAbs()));
    if (fluid_base.allFinite())
    {
      const Eigen::Vector3d force_cog = cog_from_base * fluid_base.head<3>();
      compensation.head<3>() += force_cog;
      compensation.tail<3>() += cog_from_base * fluid_base.tail<3>() +
                                base_origin_cog.cross(force_cog);
    }
    else
      ROS_ERROR_THROTTLE(1.0, "[UUVDController] Invalid fluid feedforward wrench");
  }
  return compensation;
}

void UUVDController::publishExternalWrenchFeedforward()
{
  if (!external_wrench_feedforward_enabled_) return;
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cogFrameId(nh_);
  msg.wrench.force.x = feedforward_wrench_[0];
  msg.wrench.force.y = feedforward_wrench_[1];
  msg.wrench.force.z = feedforward_wrench_[2];
  msg.wrench.torque.x = feedforward_wrench_[3];
  msg.wrench.torque.y = feedforward_wrench_[4];
  msg.wrench.torque.z = feedforward_wrench_[5];
  feedforward_wrench_pub_.publish(msg);
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
void UUVDController::publishGravityWrench()
{
  if (!gravity_wrench_pub_) return;

  tf::Vector3 gravity_world(0.0, 0.0, -robot_model_->getMass() * gravity_magnitude_);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 gravity_cog = cog_rot.inverse() * gravity_world;

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.header.frame_id = cogFrameId(nh_);
  wrench_msg.wrench.force.x = gravity_cog.x();
  wrench_msg.wrench.force.y = gravity_cog.y();
  wrench_msg.wrench.force.z = gravity_cog.z();
  wrench_msg.wrench.torque.x = 0.0;
  wrench_msg.wrench.torque.y = 0.0;
  wrench_msg.wrench.torque.z = 0.0;
  gravity_wrench_pub_.publish(wrench_msg);
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
  feedforward_wrench_ = computeExternalWrenchFeedforward(uav_rot, target_acc_cog);
  target_wrench_acc.head<3>() += feedforward_wrench_.head<3>() * mass_inv;
  target_wrench_acc.tail<3>() += inertia_inv * feedforward_wrench_.tail<3>();
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
  feedforward_wrench_.setZero();

  setAttitudeGains();
}

void UUVDController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();
  sendTorqueAllocationMatrixInv();
  publishDebugWrench();
  publishGravityWrench();
  publishExternalWrenchFeedforward();
  
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
