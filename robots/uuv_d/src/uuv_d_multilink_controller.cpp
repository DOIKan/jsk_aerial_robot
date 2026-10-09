#include <uuv_d/uuv_d_multilink_controller.h>
#include <XmlRpcValue.h>
#include <cmath>
#include <limits>

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
}
UUVDMultilinkController::UUVDMultilinkController()
  : PoseLinearController(),
    torque_allocation_matrix_inv_pub_stamp_(0.0),
    torque_allocation_matrix_inv_pub_interval_(0.05),
    target_roll_(0.0),
    target_pitch_(0.0),
    candidate_yaw_term_(0.0),
    per_link_buoyancy_enabled_(false),
    rho_water_(1000.0),
    submerged_ratio_(0.0),
    gimbal_motor_num_(4),
    fixed_motor_num_(2),
    output_rate_limit_initialized_(false),
    max_gimbal_angle_step_(0.05),
    max_thrust_step_(0.5),
    gimbal_branch_tolerance_(0.2),
    thrust_torque_weight_(10.0),
    thrust_anchor_weight_(0.1),
    gimbal_selection_initialized_(false)
{
}

void UUVDMultilinkController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  uuv_d_robot_model_ = boost::dynamic_pointer_cast<uuv_d_model::UUVDMultilinkRobotModel>(robot_model_);
  if (!uuv_d_robot_model_)
  {
    ROS_ERROR("[UUVDMultilinkController] Failed to cast robot_model_ to UUVDMultilinkRobotModel!");
  }

  ros::NodeHandle buoy_nh(nh_, "environment/buoyancy");
  buoy_nh.param("per_link_enabled", per_link_buoyancy_enabled_, false);
  buoy_nh.param("rho_water", rho_water_, 1000.0);
  buoy_nh.param("submerged_ratio", submerged_ratio_, 0.0);
  if (per_link_buoyancy_enabled_)
  {
    XmlRpc::XmlRpcValue links;
    if (!buoy_nh.getParam("links", links) || links.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("[UUVDMultilinkController] environment/buoyancy/links must be a map");
      per_link_buoyancy_enabled_ = false;
    }
    else
    {
      for (auto it = links.begin(); it != links.end(); ++it)
      {
        const XmlRpc::XmlRpcValue& config = it->second;
        if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !config.hasMember("volume") || !config.hasMember("cob") ||
            config["cob"].getType() != XmlRpc::XmlRpcValue::TypeArray || config["cob"].size() != 3)
        {
          ROS_ERROR("[UUVDMultilinkController] Invalid buoyancy settings for %s", it->first.c_str());
          per_link_buoyancy_enabled_ = false;
          break;
        }

        BuoyancyLink link;
        link.name = it->first;
        double xyz[3];
        if (!xmlRpcNumber(config["volume"], link.volume) || link.volume <= 0.0 ||
            !xmlRpcNumber(config["cob"][0], xyz[0]) ||
            !xmlRpcNumber(config["cob"][1], xyz[1]) ||
            !xmlRpcNumber(config["cob"][2], xyz[2]))
        {
          ROS_ERROR("[UUVDMultilinkController] Invalid buoyancy volume or COB for %s", link.name.c_str());
          per_link_buoyancy_enabled_ = false;
          break;
        }
        link.cob = KDL::Vector(xyz[0], xyz[1], xyz[2]);
        buoyancy_links_.push_back(link);
      }
      if (buoyancy_links_.empty()) per_link_buoyancy_enabled_ = false;
      if (!per_link_buoyancy_enabled_) buoyancy_links_.clear();
    }
  }
  robot_model_for_control_ = boost::make_shared<uuv_d_model::UUVDMultilinkRobotModel>();

  q_mat_.resize(4, motor_num_);
  q_mat_inv_.resize(motor_num_, 4);
  gimbal_rotor_indices_ = {0, 1, 4, 5};  // rotor1, rotor2, rotor5, rotor6
  fixed_rotor_indices_  = {2, 3};//rotor3, rotor4

  target_base_thrust_.resize(motor_num_, 0.0);
  target_gimbal_angles_.resize(gimbal_motor_num_, 0.0);
  prev_base_thrust_.resize(motor_num_, 0.0);
  prev_gimbal_angles_.resize(gimbal_motor_num_, 0.0);

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  getParam<double>(control_nh, "max_gimbal_angle_step", max_gimbal_angle_step_, 0.05);
  getParam<double>(control_nh, "max_thrust_step", max_thrust_step_, 0.5);
  getParam<double>(control_nh, "gimbal_branch_tolerance", gimbal_branch_tolerance_, 0.2);
  getParam<double>(control_nh, "thrust_torque_weight", thrust_torque_weight_, 10.0);
  getParam<double>(control_nh, "thrust_anchor_weight", thrust_anchor_weight_, 0.1);

  // internal z force per gimbal rotor; enlarges small rotor forces so that gimbal angles become less sensitive
  if (!control_nh.getParam("gimbal_internal_force", gimbal_internal_force_))
    gimbal_internal_force_ = {-3.0, 0.0, 0.0, -3.0};
  if (gimbal_internal_force_.size() != static_cast<size_t>(gimbal_motor_num_))
  {
    ROS_ERROR("[UUVDMultilinkController] gimbal_internal_force needs %d elements; disabled", gimbal_motor_num_);
    gimbal_internal_force_.assign(gimbal_motor_num_, 0.0);
  }

  target_wrench_cog_ = Eigen::VectorXd::Zero(6);

  selected_gimbal_angles_.assign(gimbal_motor_num_, 0.0);

  // gimbal joint limits from URDF, used for branch selection and clamping
  gimbal_lower_limits_.assign(gimbal_motor_num_, -M_PI_2);
  gimbal_upper_limits_.assign(gimbal_motor_num_, M_PI_2);
  for (int i = 0; i < gimbal_motor_num_; i++)
  {
    const std::string name = "gimbal" + std::to_string(i + 1);
    const auto joint = robot_model_->getUrdfModel().getJoint(name);
    if (joint && joint->limits && joint->limits->lower < joint->limits->upper)
    {
      gimbal_lower_limits_.at(i) = joint->limits->lower;
      gimbal_upper_limits_.at(i) = joint->limits->upper;
    }
    else
    {
      ROS_WARN("[UUVDMultilinkController] No joint limits for %s; use [-pi/2, pi/2]", name.c_str());
    }
  }

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gravity_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("debug/gravity_wrench", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  debug_wrench_pubs_.resize(motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    // トピック名を "motor_0/wrench", "motor_1/wrench" のように設定
    std::string topic_name = "motor_" + std::to_string(i) + "/wrench";
    debug_wrench_pubs_.at(i) = nh_.advertise<geometry_msgs::WrenchStamped>(topic_name, 1);
  }
}
void UUVDMultilinkController::publishDebugWrench()
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
void UUVDMultilinkController::publishGravityWrench()
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
void UUVDMultilinkController::controlCore()
{
  PoseLinearController::controlCore();
  target_roll_ = target_rpy_.x();
  target_pitch_ = target_rpy_.y();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

  // Gravity acts at the CoG. Match Gazebo's per-link buoyancy forces and
  // transform each link's center of buoyancy into the current CoG frame.
  const tf::Vector3 gravity_world(0.0, 0.0, -robot_model_->getMass() * gravity_magnitude_);
  tf::Vector3 external_force_cog = uav_rot.inverse() * gravity_world;
  tf::Vector3 buoyancy_torque_cog(0.0, 0.0, 0.0);
  if (per_link_buoyancy_enabled_ && uuv_d_robot_model_)
  {
    const double force_per_volume = std::clamp(submerged_ratio_, 0.0, 1.0) * rho_water_ * 9.80665;
    const KDL::Frame cog = uuv_d_robot_model_->getCog<KDL::Frame>();
    const auto segment_frames = uuv_d_robot_model_->getSegmentsTf();
    for (const auto& link : buoyancy_links_)
    {
      const auto segment = segment_frames.find(link.name);
      if (segment == segment_frames.end())
      {
        ROS_ERROR_THROTTLE(1.0, "[UUVDMultilinkController] Missing buoyancy link %s", link.name.c_str());
        continue;
      }
      const tf::Vector3 force_cog = uav_rot.inverse() *
        tf::Vector3(0.0, 0.0, force_per_volume * link.volume);
      const KDL::Vector cob_cog = cog.Inverse() * (segment->second * link.cob);
      const tf::Vector3 lever_arm(cob_cog.x(), cob_cog.y(), cob_cog.z());
      external_force_cog += force_cog;
      buoyancy_torque_cog += lever_arm.cross(force_cog);
    }
  }

  Eigen::Matrix<double, 6, 1> target_wrench_cog;
  // Spinal applies attitude feedback; compensate only the buoyancy torque here.
  target_wrench_cog << robot_model_->getMass() * target_acc_cog.x() - external_force_cog.x(),
                       robot_model_->getMass() * target_acc_cog.y() - external_force_cog.y(),
                       robot_model_->getMass() * target_acc_cog.z() - external_force_cog.z(),
                       -buoyancy_torque_cog.x(), -buoyancy_torque_cog.y(), -buoyancy_torque_cog.z();
  target_wrench_cog_ = target_wrench_cog;

  wrenchAllocation(target_wrench_cog);
  applyOutputRateLimit();
  processGimbalAngles();

  // Spinal reconstructs the PC yaw command from the reference motor's thrust
  // and adds measured-rate yaw D feedback with the same inverse matrix.
  const double max_yaw_scale = q_mat_inv_.col(3).maxCoeff();
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * std::max(0.0, max_yaw_scale);

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
  ROS_INFO_STREAM_THROTTLE(0.5, "[UUVDMultilinkController] controlCore");
}
void UUVDMultilinkController::wrenchAllocation(const Eigen::VectorXd& target_wrench)
{
  Eigen::MatrixXd full_q_mat;
  std::vector<int> gimbal_rotor_indices;

  if (uuv_d_robot_model_)
  {
    full_q_mat = uuv_d_robot_model_->getFullWrenchAllocationMatrixfromCoG();
    gimbal_rotor_indices = uuv_d_robot_model_->getGimbalRotorIndices();
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "[UUVDMultilinkController] UUVDMultilinkRobotModel is not available");
    return;
  }

  // lambda: [2D force of each gimbal rotor (gimbal force basis), thrust of each fixed rotor]
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  Eigen::VectorXd lambda = full_q_mat_inv * target_wrench;

  // add the internal force only in the null space so that the net wrench is unchanged
  Eigen::VectorXd internal_force = Eigen::VectorXd::Zero(lambda.size());
  for (size_t i = 0; i < gimbal_rotor_indices.size(); ++i)
    internal_force(2 * i + 1) = gimbal_internal_force_.at(i);  // 2nd column of the basis is z
  const Eigen::MatrixXd null_projector =
    Eigen::MatrixXd::Identity(lambda.size(), lambda.size()) - full_q_mat_inv * full_q_mat;
  lambda += null_projector * internal_force;
  allocation_lambda_ = lambda;  // reused as the anchor in updateRotorThrusts()

  // start the branch selection from the measured gimbal angles
  if (!gimbal_selection_initialized_)
  {
    const auto& current_gimbal_angles = uuv_d_robot_model_->getCurrentGimbalAngles();
    for (size_t i = 0; i < selected_gimbal_angles_.size(); ++i)
      selected_gimbal_angles_.at(i) = current_gimbal_angles.size() == selected_gimbal_angles_.size() ? current_gimbal_angles.at(i) : 0.0;
    gimbal_selection_initialized_ = true;
  }

  // decide only the gimbal angles here; thrusts are decided in updateRotorThrusts()
  for (size_t i = 0; i < gimbal_rotor_indices.size(); ++i)
  {
    // keep the previous angle when the force is too small to define a direction
    if (lambda.segment(2 * i, 2).norm() > 1e-3)
    {
      const double raw_angle = std::atan2(-lambda(2 * i + 0), lambda(2 * i + 1));  // rotor axis is (-sin q, cos q)
      // reference is the previous selection, not the rate-limited angle, to keep the branch while turning
      selected_gimbal_angles_.at(i) = selectGimbalAngle(i, raw_angle, selected_gimbal_angles_.at(i));
    }
    target_gimbal_angles_.at(i) = selected_gimbal_angles_.at(i);
  }

}

double UUVDMultilinkController::selectGimbalAngle(int gimbal_index, double raw_angle, double reference_angle) const
{
  // choose among q + k*pi (thrust sign flips with odd k) the reachable angle closest to the reference
  const double lower = gimbal_lower_limits_.at(gimbal_index);
  const double upper = gimbal_upper_limits_.at(gimbal_index);
  const double base = angles::normalize_angle(raw_angle);

  double best_angle = std::clamp(reference_angle, lower, upper);
  double best_direction_error = std::numeric_limits<double>::max();
  double best_distance = std::numeric_limits<double>::max();
  bool best_feasible = false;
  for (int k = -2; k <= 2; ++k)
  {
    const double candidate = base + k * M_PI;
    const double clamped = std::clamp(candidate, lower, upper);
    const double direction_error = std::fabs(candidate - clamped);  // force direction lost by the joint limit
    const double distance = std::fabs(clamped - reference_angle);
    const bool feasible = direction_error <= gimbal_branch_tolerance_;

    // prefer feasible candidates, then the closest one; if none is feasible, the smallest direction error
    bool better;
    if (feasible != best_feasible) better = feasible;
    else if (feasible) better = distance < best_distance;
    else better = direction_error < best_direction_error ||
                  (direction_error == best_direction_error && distance < best_distance);

    if (better)
    {
      best_angle = clamped;
      best_direction_error = direction_error;
      best_distance = distance;
      best_feasible = feasible;
    }
  }
  return best_angle;
}

void UUVDMultilinkController::applyOutputRateLimit()
{
  if (!output_rate_limit_initialized_)
  {
    std::fill(prev_base_thrust_.begin(), prev_base_thrust_.end(), 0.0);

    if (uuv_d_robot_model_ && uuv_d_robot_model_->getCurrentGimbalAngles().size() == prev_gimbal_angles_.size())
    {
      const auto& current_gimbal_angles = uuv_d_robot_model_->getCurrentGimbalAngles();
      for (size_t i = 0; i < prev_gimbal_angles_.size(); ++i)
        prev_gimbal_angles_.at(i) = static_cast<float>(current_gimbal_angles.at(i));
    }
    else
    {
      std::fill(prev_gimbal_angles_.begin(), prev_gimbal_angles_.end(), 0.0);
    }

    output_rate_limit_initialized_ = true;
  }

  // plain difference, since shortest_angular_distance may turn the gimbal through the joint limit
  const double angle_step = std::max(0.0, max_gimbal_angle_step_);
  for (size_t i = 0; i < target_gimbal_angles_.size(); ++i)
  {
    const double diff = target_gimbal_angles_.at(i) - prev_gimbal_angles_.at(i);
    const double limited = prev_gimbal_angles_.at(i) + std::clamp(diff, -angle_step, angle_step);
    target_gimbal_angles_.at(i) = static_cast<float>(std::clamp(limited, gimbal_lower_limits_.at(i), gimbal_upper_limits_.at(i)));
    prev_gimbal_angles_.at(i) = target_gimbal_angles_.at(i);
  }

  // thrusts must be decided after the gimbal angles are rate-limited
  updateRotorThrusts();

  const float thrust_step = static_cast<float>(std::max(0.0, max_thrust_step_));
  for (size_t i = 0; i < target_base_thrust_.size(); ++i)
  {
    const float diff = target_base_thrust_.at(i) - prev_base_thrust_.at(i);
    const float limited_diff = std::clamp(diff, -thrust_step, thrust_step);
    target_base_thrust_.at(i) = prev_base_thrust_.at(i) + limited_diff;
    prev_base_thrust_.at(i) = target_base_thrust_.at(i);
  }
}

void UUVDMultilinkController::updateRotorThrusts()
{
  if (!uuv_d_robot_model_) return;

  const auto& gimbal_rotor_indices = uuv_d_robot_model_->getGimbalRotorIndices();
  const auto& fixed_rotor_indices = uuv_d_robot_model_->getFixedRotorIndices();
  const int rotor_num = target_base_thrust_.size();

  if (allocation_lambda_.size() != static_cast<int>(2 * gimbal_rotor_indices.size() + fixed_rotor_indices.size()))
    return;

  // anchor: thrusts of wrenchAllocation() at the commanded gimbal angles
  Eigen::VectorXd anchor = Eigen::VectorXd::Zero(rotor_num);
  for (size_t i = 0; i < gimbal_rotor_indices.size(); ++i)
  {
    // project the 2D force onto the rotor axis (-sin q, cos q)
    const double q = target_gimbal_angles_.at(i);
    anchor(gimbal_rotor_indices.at(i)) = -allocation_lambda_(2 * i) * std::sin(q) + allocation_lambda_(2 * i + 1) * std::cos(q);
  }
  for (size_t i = 0; i < fixed_rotor_indices.size(); ++i)
    anchor(fixed_rotor_indices.at(i)) = allocation_lambda_(2 * gimbal_rotor_indices.size() + i);

  // re-solve the target wrench with the measured gimbal angles to remove torque errors while the gimbals turn
  const Eigen::MatrixXd q_mat = uuv_d_robot_model_->calcWrenchMatrixOnCoG();

  // weight torque more, since an attitude error is not recovered by the position loop
  Eigen::VectorXd weight = Eigen::VectorXd::Ones(6);
  weight.tail(3).setConstant(thrust_torque_weight_);
  const Eigen::MatrixXd wq = weight.asDiagonal() * q_mat;

  // min |W(Q f - w)|^2 + a |f - anchor|^2; the anchor term keeps ill-conditioned directions from amplifying noise
  // derivative: 2 Q^T W^T W (Q f - w) + 2 a (f - anchor) = 0
  // solve: (Q^T W^T W Q + a I) f = Q^T W^T W w + a anchor
  const Eigen::MatrixXd h = wq.transpose() * wq + thrust_anchor_weight_ * Eigen::MatrixXd::Identity(rotor_num, rotor_num);
  const Eigen::VectorXd thrust = h.ldlt().solve(wq.transpose() * (weight.asDiagonal() * target_wrench_cog_) +
                                                thrust_anchor_weight_ * anchor);

  if (!thrust.allFinite())
  {
    ROS_ERROR_THROTTLE(1.0, "[UUVDMultilinkController] invalid thrust re-allocation; keep previous thrust");
    target_base_thrust_ = prev_base_thrust_;
    return;
  }

  const float lower_limit = static_cast<float>(robot_model_->getThrustLowerLimit());
  const float upper_limit = static_cast<float>(robot_model_->getThrustUpperLimit());
  for (int i = 0; i < rotor_num; ++i)
    target_base_thrust_.at(i) = std::clamp(static_cast<float>(thrust(i)), lower_limit, upper_limit);
}

void UUVDMultilinkController::processGimbalAngles()
{
  KDL::JntArray current_joint_positions = robot_model_->getJointPositions();

  robot_model_for_control_->setCogDesireOrientation(
    robot_model_->getCogDesireOrientation<KDL::Rotation>());

  robot_model_for_control_->updateRobotModel(current_joint_positions);

  const Eigen::MatrixXd wrench_mat = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_.row(0) = wrench_mat.row(2) / robot_model_for_control_->getMass();
  q_mat_.bottomRows(3) = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse() * wrench_mat.bottomRows(3);
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

  // The rosserial message stores each coefficient as int16 after x1000.
  // A singular pose must not send wrapped coefficients to Spinal.
  const Eigen::MatrixXd attitude_inv = q_mat_inv_.rightCols(3);
  if (!attitude_inv.allFinite() || attitude_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001)
  {
    ROS_ERROR_THROTTLE(1.0, "Torque Allocation Matrix overflow; disabling Spinal attitude correction for this pose");
    q_mat_inv_.setZero();
  }
}
void UUVDMultilinkController::reset()
{
  PoseLinearController::reset();

  std::fill(prev_base_thrust_.begin(), prev_base_thrust_.end(), 0.0);
  std::fill(prev_gimbal_angles_.begin(), prev_gimbal_angles_.end(), 0.0);
  output_rate_limit_initialized_ = false;
  gimbal_selection_initialized_ = false;
  target_wrench_cog_.setZero();
  allocation_lambda_.resize(0);
  candidate_yaw_term_ = 0.0;

  setAttitudeGains();
}

void UUVDMultilinkController::sendCmd()
{
  PoseLinearController::sendCmd();
  ROS_INFO_STREAM_THROTTLE(0.5, "[UUVDMultilinkController] sendCmd");                    

  sendTorqueAllocationMatrixInv();
  sendFourAxisCommand();
  setGimbalAngles();
  publishDebugWrench();
  publishGravityWrench();
  
}
void UUVDMultilinkController::setGimbalAngles()
{
  sensor_msgs::JointState gimbal_cmd_msg;
  gimbal_cmd_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < gimbal_motor_num_; i++)
  {
    gimbal_cmd_msg.name.push_back("gimbal" + std::to_string(i + 1));
    gimbal_cmd_msg.position.push_back(target_gimbal_angles_.at(i));
  }

  gimbal_control_pub_.publish(gimbal_cmd_msg);
}

void UUVDMultilinkController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void UUVDMultilinkController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void UUVDMultilinkController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UUVDMultilinkController, aerial_robot_control::ControlBase);
