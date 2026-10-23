#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <Eigen/Geometry>

namespace aerial_robot_control
{
class UUVDController : public aerial_robot_control::PoseLinearController
{
public:
  UUVDController();
  virtual ~UUVDController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

  void reset() override;
  void controlCore() override;
  void sendCmd() override;

private:
  ros::Publisher flight_cmd_pub_;
  ros::Publisher rpy_gain_pub_;
  ros::Publisher torque_allocation_matrix_inv_pub_;
  double torque_allocation_matrix_inv_pub_stamp_;
  double torque_allocation_matrix_inv_pub_interval_;
  Eigen::MatrixXd q_mat_;
  Eigen::MatrixXd q_mat_inv_;
  std::vector<float> target_base_thrust_;
  double target_roll_;
  double target_pitch_;
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  bool external_wrench_feedforward_enabled_ = false;
  bool buoyancy_feedforward_enabled_ = false;
  bool hydrodynamic_feedforward_enabled_ = false;
  double rho_water_ = 1000.0;
  double submerged_ratio_ = 0.0;
  double buoyancy_volume_ = 0.0;
  KDL::Vector buoyancy_cob_;
  Eigen::Vector3d current_velocity_world_ = Eigen::Vector3d::Zero();
  Matrix6 added_mass_ = Matrix6::Zero();
  Matrix6 quadratic_drag_ = Matrix6::Zero();
  Vector6 feedforward_wrench_ = Vector6::Zero();
  ros::Publisher feedforward_wrench_pub_;
  void loadExternalWrenchFeedforward();
  Vector6 computeExternalWrenchFeedforward(const tf::Matrix3x3& cog_rotation,
                                           const tf::Vector3& target_acceleration_cog);
  void publishExternalWrenchFeedforward();
  void sendFourAxisCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();

  std::vector<ros::Publisher> debug_wrench_pubs_;
  ros::Publisher gravity_wrench_pub_;
  void publishDebugWrench();
  void publishGravityWrench();
};
}  // namespace aerial_robot_control
