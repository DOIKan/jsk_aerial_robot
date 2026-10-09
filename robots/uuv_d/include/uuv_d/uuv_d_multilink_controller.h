#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <uuv_d/model/uuv_d_multilink_robot_model.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <sensor_msgs/JointState.h>// add gimbal joint state
#include <angles/angles.h>
#include <algorithm>

namespace aerial_robot_control
{
class UUVDMultilinkController : public aerial_robot_control::PoseLinearController
{
public:
  UUVDMultilinkController();
  virtual ~UUVDMultilinkController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

  void reset() override;
  void controlCore() override;
  void sendCmd() override;
  const std::vector<int>& getGimbalRotorIndices() const { return gimbal_rotor_indices_; }
  const std::vector<int>& getFixedRotorIndices() const { return fixed_rotor_indices_; }

private:
  ros::Publisher flight_cmd_pub_;
  ros::Publisher rpy_gain_pub_;
  ros::Publisher torque_allocation_matrix_inv_pub_;
  ros::Publisher gimbal_control_pub_;// add joint state publisher for gimbal control

  double torque_allocation_matrix_inv_pub_stamp_;
  double torque_allocation_matrix_inv_pub_interval_;
  Eigen::MatrixXd q_mat_;
  Eigen::MatrixXd q_mat_inv_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_gimbal_angles_;//add target gimbal angles
  std::vector<float> prev_base_thrust_;
  std::vector<float> prev_gimbal_angles_;
  bool output_rate_limit_initialized_;
  double max_gimbal_angle_step_;
  double max_thrust_step_;
  std::vector<double> gimbal_lower_limits_;
  std::vector<double> gimbal_upper_limits_;
  double gimbal_branch_tolerance_;
  double thrust_torque_weight_;
  double thrust_anchor_weight_;
  std::vector<double> gimbal_internal_force_;  // internal z force per gimbal rotor (null space only)
  Eigen::VectorXd allocation_lambda_;  // [2D force per gimbal rotor, fixed rotor thrusts]
  Eigen::VectorXd target_wrench_cog_;
  std::vector<double> selected_gimbal_angles_;  // selected branch before the rate limit
  bool gimbal_selection_initialized_;
  double target_roll_;
  double target_pitch_;
  double candidate_yaw_term_;

  struct BuoyancyLink
  {
    std::string name;
    double volume;
    KDL::Vector cob;
  };
  std::vector<BuoyancyLink> buoyancy_links_;
  bool per_link_buoyancy_enabled_;
  double rho_water_;
  double submerged_ratio_;

  int gimbal_motor_num_;
  int fixed_motor_num_;
  boost::shared_ptr<uuv_d_model::UUVDMultilinkRobotModel> uuv_d_robot_model_;
  boost::shared_ptr<uuv_d_model::UUVDMultilinkRobotModel> robot_model_for_control_;
  void wrenchAllocation(const Eigen::VectorXd& target_wrench);
  double selectGimbalAngle(int gimbal_index, double raw_angle, double reference_angle) const;
  void applyOutputRateLimit();
  void updateRotorThrusts();
  void processGimbalAngles();

  void sendFourAxisCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
  void setGimbalAngles();//add gimbal angle control function

  std::vector<ros::Publisher> debug_wrench_pubs_;
  ros::Publisher gravity_wrench_pub_;
  void publishDebugWrench();
  void publishGravityWrench();
  std::vector<int> gimbal_rotor_indices_;
  std::vector<int> fixed_rotor_indices_;
};
}  // namespace aerial_robot_control
