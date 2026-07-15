#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>

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
  void sendFourAxisCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();

  std::vector<ros::Publisher> debug_wrench_pubs_;
  void publishDebugWrench();
};
}  // namespace aerial_robot_control
