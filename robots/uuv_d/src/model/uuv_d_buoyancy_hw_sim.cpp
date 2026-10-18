#include <aerial_robot_simulation/aerial_robot_hw_sim.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <geometry_msgs/WrenchStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <XmlRpcValue.h>

namespace uuv_d_simulation
{

class BuoyancyHWSim : public gazebo_ros_control::AerialRobotHWSim
{
public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent_model, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override
  {
    if (!AerialRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions))
      return false;

    ros::NodeHandle buoy_nh(model_nh, "environment/buoyancy");
    buoy_nh.param("per_link_enabled", enabled_, false);

    ros::NodeHandle hydro_nh(model_nh, "environment/hydrodynamics");
    hydro_nh.param("enabled", hydrodynamics_enabled_, false);
    hydro_nh.param("acceleration_filter_alpha", acceleration_filter_alpha_, 0.3);
    hydro_nh.param("debug_rate", hydrodynamics_debug_rate_, 20.0);
    if (hydrodynamics_enabled_)
    {
      XmlRpc::XmlRpcValue current;
      if (!hydro_nh.getParam("current_velocity_world", current) ||
          current.getType() != XmlRpc::XmlRpcValue::TypeArray || current.size() != 3 ||
          !number(current[0], current_velocity_world_[0]) ||
          !number(current[1], current_velocity_world_[1]) ||
          !number(current[2], current_velocity_world_[2]))
      {
        ROS_ERROR("environment/hydrodynamics/current_velocity_world must have 3 finite values");
        return false;
      }
    }
    if (hydrodynamics_enabled_ &&
        (!std::isfinite(acceleration_filter_alpha_) || acceleration_filter_alpha_ <= 0.0 ||
         acceleration_filter_alpha_ > 1.0 || !std::isfinite(hydrodynamics_debug_rate_) ||
         hydrodynamics_debug_rate_ <= 0.0))
    {
      ROS_ERROR("Invalid hydrodynamics acceleration_filter_alpha or debug_rate");
      return false;
    }

    frame_prefix_ = model_nh.getNamespace();
    if (!frame_prefix_.empty() && frame_prefix_.front() == '/')
      frame_prefix_.erase(0, 1);

    if (hydrodynamics_enabled_ && !loadHydrodynamics(model_nh, parent_model))
      return false;
    if (!enabled_)
      return true;

    if (use_buoyancy_)
    {
      ROS_ERROR("Disable environment/buoyancy/enabled when per_link_enabled is true");
      return false;
    }

    XmlRpc::XmlRpcValue params;
    if (!buoy_nh.getParam("links", params) || params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("environment/buoyancy/links must be a map of link volumes and COB offsets");
      return false;
    }

    for (auto it = params.begin(); it != params.end(); ++it)
    {
      const std::string name = it->first;
      const XmlRpc::XmlRpcValue& config = it->second;
      if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !config.hasMember("volume") || !config.hasMember("cob") ||
          config["cob"].getType() != XmlRpc::XmlRpcValue::TypeArray || config["cob"].size() != 3)
      {
        ROS_ERROR("Invalid buoyancy settings for link %s", name.c_str());
        return false;
      }

      LinkBuoyancy entry;
      entry.name = name;
      entry.link = parent_model_->GetLink(name);
      if (!entry.link || !number(config["volume"], entry.volume) || entry.volume <= 0.0)
      {
        ROS_ERROR("Invalid link or buoyancy volume for %s", name.c_str());
        return false;
      }

      double xyz[3];
      for (int axis = 0; axis < 3; ++axis)
        if (!number(config["cob"][axis], xyz[axis]))
        {
          ROS_ERROR("Invalid COB offset for %s", name.c_str());
          return false;
        }
      entry.cob.Set(xyz[0], xyz[1], xyz[2]);
      entry.publisher = model_nh.advertise<geometry_msgs::WrenchStamped>("debug/buoyancy_wrench/" + name, 1);
      links_.push_back(entry);
    }

    buoy_nh.param("debug_rate", debug_rate_, 20.0);
    if (links_.empty() || !std::isfinite(debug_rate_) || debug_rate_ <= 0.0)
    {
      ROS_ERROR("Buoyancy requires at least one link and a positive debug_rate");
      return false;
    }
    ROS_INFO("Per-link buoyancy enabled for %zu links", links_.size());
    return true;
  }

  void writeSim(ros::Time time, ros::Duration period) override
  {
    // a non-finite force never recovers through RotorHandle's filter and is skipped silently; restore the last finite one
    last_finite_forces_.resize(rotor_n_dof_, 0.0);
    for (unsigned int j = 0; j < rotor_n_dof_; ++j)
    {
      hardware_interface::RotorHandle rotor = spinal_interface_.getHandle(sim_rotors_.at(j)->GetName());
      if (std::isfinite(rotor.getForce()))
      {
        last_finite_forces_.at(j) = rotor.getForce();
        continue;
      }
      ROS_ERROR_THROTTLE(1.0, "[BuoyancyHWSim] non-finite force on %s at %.3f; use last finite force %.3f",
                         rotor.getName().c_str(), time.toSec(), last_finite_forces_.at(j));
      rotor.setForce(last_finite_forces_.at(j), true);
    }

    AerialRobotHWSim::writeSim(time, period);
    if (hydrodynamics_enabled_)
      applyHydrodynamics(time, period);
    if (!enabled_ || control_mode_ != FORCE_CONTROL_MODE)
      return;

    const double force_scale = rho_water_ * std::max(0.0, std::min(submerged_ratio_, 1.0)) * 9.80665;
    const bool publish = last_debug_time_.isZero() ||
                         (time - last_debug_time_).toSec() >= 1.0 / debug_rate_;

    for (const auto& entry : links_)
    {
      const double force = force_scale * entry.volume;
      const ignition::math::Vector3d world_force(0.0, 0.0, force);
      const auto pose = entry.link->WorldPose();
      entry.link->AddForceAtWorldPosition(world_force, pose.Pos() + pose.Rot().RotateVector(entry.cob));

      if (!publish)
        continue;

      const std::string link_frame = frame_prefix_ + "/" + entry.name;
      const std::string cob_frame = frame_prefix_ + "/cob_" + entry.name;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(entry.cob.X(), entry.cob.Y(), entry.cob.Z()));
      transform.setRotation(tf::Quaternion(0, 0, 0, 1));
      broadcaster_.sendTransform(tf::StampedTransform(transform, time, link_frame, cob_frame));

      const auto local_force = pose.Rot().RotateVectorReverse(world_force);
      geometry_msgs::WrenchStamped wrench;
      wrench.header.stamp = time;
      wrench.header.frame_id = cob_frame;
      wrench.wrench.force.x = local_force.X();
      wrench.wrench.force.y = local_force.Y();
      wrench.wrench.force.z = local_force.Z();
      entry.publisher.publish(wrench);
    }
    if (publish)
      last_debug_time_ = time;
  }

private:
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;

  struct LinkHydrodynamics
  {
    std::string name;
    gazebo::physics::LinkPtr link;
    Matrix6 added_mass;
    Matrix6 quadratic_drag;
    Matrix6 rigid_mass;
    double acceleration_alpha = 0.0;
    Vector6 previous_velocity = Vector6::Zero();
    Vector6 filtered_acceleration = Vector6::Zero();
    ros::Time previous_time;
    ros::Publisher drag_publisher;
  };

  static bool matrix6(const XmlRpc::XmlRpcValue& value, Matrix6& matrix)
  {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 6)
      return false;
    for (int row = 0; row < 6; ++row)
    {
      if (value[row].getType() != XmlRpc::XmlRpcValue::TypeArray || value[row].size() != 6)
        return false;
      for (int col = 0; col < 6; ++col)
        if (!number(value[row][col], matrix(row, col)))
          return false;
    }
    return true;
  }

  static Eigen::Matrix3d skew(const Eigen::Vector3d& v)
  {
    Eigen::Matrix3d s;
    s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return s;
  }

  static Matrix6 rigidMassAtLinkOrigin(const gazebo::physics::LinkPtr& link)
  {
    const auto inertial = link->GetInertial();
    const double mass = inertial->Mass();
    const auto cog = inertial->CoG();
    const Eigen::Vector3d r(cog.X(), cog.Y(), cog.Z());
    const Eigen::Matrix3d sr = skew(r);
    Eigen::Matrix3d inertia;
    inertia << inertial->IXX(), inertial->IXY(), inertial->IXZ(),
               inertial->IXY(), inertial->IYY(), inertial->IYZ(),
               inertial->IXZ(), inertial->IYZ(), inertial->IZZ();
    Matrix6 result = Matrix6::Zero();
    result.topLeftCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
    result.topRightCorner<3, 3>() = -mass * sr;
    result.bottomLeftCorner<3, 3>() = mass * sr;
    result.bottomRightCorner<3, 3>() = inertia - mass * sr * sr;
    return result;
  }

  bool loadHydrodynamics(ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model)
  {
    XmlRpc::XmlRpcValue params;
    ros::NodeHandle hydro_nh(model_nh, "environment/hydrodynamics");
    if (!hydro_nh.getParam("links", params) || params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("environment/hydrodynamics/links must be a map");
      return false;
    }
    for (auto it = params.begin(); it != params.end(); ++it)
    {
      const std::string name = it->first;
      const XmlRpc::XmlRpcValue& config = it->second;
      if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !config.hasMember("added_mass") || !config.hasMember("quadratic_drag"))
      {
        ROS_ERROR("Missing added_mass or quadratic_drag for %s", name.c_str());
        return false;
      }
      LinkHydrodynamics entry;
      entry.name = name;
      entry.link = parent_model->GetLink(name);
      if (!entry.link || !matrix6(config["added_mass"], entry.added_mass) ||
          !matrix6(config["quadratic_drag"], entry.quadratic_drag))
      {
        ROS_ERROR("Invalid hydrodynamics link or 6x6 matrix for %s", name.c_str());
        return false;
      }
      for (int axis = 0; axis < 6; ++axis)
      {
        if (entry.quadratic_drag(axis, axis) < 0.0)
        {
          ROS_ERROR("%s: quadratic_drag[%d][%d] is negative; enter a positive resistance magnitude",
                    name.c_str(), axis, axis);
          return false;
        }
      }
      entry.rigid_mass = rigidMassAtLinkOrigin(entry.link);
      const Matrix6 symmetric_added_mass =
        0.5 * (entry.added_mass + entry.added_mass.transpose());
      Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6> modes(
        symmetric_added_mass, entry.rigid_mass, Eigen::EigenvaluesOnly);
      if (modes.info() != Eigen::Success || !modes.eigenvalues().allFinite() ||
          modes.eigenvalues().minCoeff() < -0.99)
      {
        ROS_ERROR("Invalid rigid or added mass matrix for %s", name.c_str());
        return false;
      }
      // Explicit acceleration feedback has a pole 1-alpha*(1+M_A/M_RB).
      // Keep that pole inside the unit circle, including for large angular added mass.
      const double largest_ratio = std::max(0.0, modes.eigenvalues().maxCoeff());
      entry.acceleration_alpha = std::min(acceleration_filter_alpha_, 0.8 / (1.0 + largest_ratio));
      if (entry.acceleration_alpha < acceleration_filter_alpha_)
        ROS_WARN("%s: added mass / rigid inertia ratio %.1f limits acceleration alpha to %.6g",
                 name.c_str(), largest_ratio, entry.acceleration_alpha);
      entry.drag_publisher = model_nh.advertise<geometry_msgs::WrenchStamped>(
        "debug/drag_wrench/" + name, 1);
      hydrodynamics_links_.push_back(entry);
    }
    if (hydrodynamics_links_.empty())
    {
      ROS_ERROR("Hydrodynamics requires at least one link");
      return false;
    }
    ROS_INFO("Per-link hydrodynamics enabled for %zu links", hydrodynamics_links_.size());
    return true;
  }

  void applyHydrodynamics(const ros::Time& time, const ros::Duration& period)
  {
    // The matrices, velocities and published wrenches are all expressed at the link origin.
    const double ratio = std::clamp(submerged_ratio_, 0.0, 1.0);
    const bool publish = last_hydrodynamics_debug_time_.isZero() ||
      (time - last_hydrodynamics_debug_time_).toSec() >= 1.0 / hydrodynamics_debug_rate_;
    for (auto& entry : hydrodynamics_links_)
    {
      const auto pose = entry.link->WorldPose();
      const auto linear = pose.Rot().RotateVectorReverse(
        entry.link->WorldLinearVel() - ignition::math::Vector3d(
          current_velocity_world_[0], current_velocity_world_[1], current_velocity_world_[2]));
      const auto angular = pose.Rot().RotateVectorReverse(entry.link->WorldAngularVel());
      Vector6 velocity;
      velocity << linear.X(), linear.Y(), linear.Z(), angular.X(), angular.Y(), angular.Z();
      if (!velocity.allFinite() || velocity.cwiseAbs().maxCoeff() > 1e6)
      {
        entry.previous_time = ros::Time();
        entry.filtered_acceleration.setZero();
        ROS_ERROR_THROTTLE(1.0, "Invalid link-origin velocity for %s: [%g %g %g %g %g %g]",
                           entry.name.c_str(), velocity[0], velocity[1], velocity[2],
                           velocity[3], velocity[4], velocity[5]);
        continue;
      }

      double dt = period.toSec();
      if (!entry.previous_time.isZero() && time > entry.previous_time)
      {
        dt = (time - entry.previous_time).toSec();
        const Vector6 acceleration = (velocity - entry.previous_velocity) / dt;
        entry.filtered_acceleration = (1.0 - entry.acceleration_alpha) *
          entry.filtered_acceleration + entry.acceleration_alpha * acceleration;
      }
      else
        entry.filtered_acceleration.setZero();
      entry.previous_velocity = velocity;
      entry.previous_time = time;
      if (!std::isfinite(dt) || dt <= 0.0)
        continue;

      const Vector6 momentum = entry.added_mass * velocity;
      const Eigen::Vector3d linear_velocity = velocity.head<3>();
      const Eigen::Vector3d angular_velocity = velocity.tail<3>();
      Vector6 added_coriolis;
      added_coriolis.head<3>() = -momentum.head<3>().cross(angular_velocity);
      added_coriolis.tail<3>() = -momentum.head<3>().cross(linear_velocity) -
                                momentum.tail<3>().cross(angular_velocity);

      // A one-step implicit damping factor prevents explicit quadratic drag
      // from reversing a low-inertia link's velocity in one physics step.
      const Matrix6 drag_jacobian = 2.0 * ratio * entry.quadratic_drag *
        velocity.cwiseAbs().asDiagonal();
      const Matrix6 normalized_jacobian = entry.rigid_mass.ldlt().solve(drag_jacobian);
      const double stiffness = normalized_jacobian.cwiseAbs().rowwise().sum().maxCoeff();
      const double drag_scale = 1.0 / (1.0 + dt * stiffness);
      const Vector6 drag = -ratio * drag_scale * entry.quadratic_drag *
        velocity.cwiseProduct(velocity.cwiseAbs());
      const Vector6 fluid_wrench = drag - ratio *
        (entry.added_mass * entry.filtered_acceleration + added_coriolis);
      if (!fluid_wrench.allFinite())
      {
        ROS_ERROR_THROTTLE(1.0, "Non-finite hydrodynamic wrench for %s (speed=%.4g, accel=%.4g, drag stiffness=%.4g)",
                           entry.name.c_str(), velocity.norm(), entry.filtered_acceleration.norm(), stiffness);
        continue;
      }
      // AddLinkForce applies force at the link origin, matching the matrix reference point.
      entry.link->AddLinkForce(ignition::math::Vector3d(
        fluid_wrench[0], fluid_wrench[1], fluid_wrench[2]));
      entry.link->AddRelativeTorque(ignition::math::Vector3d(
        fluid_wrench[3], fluid_wrench[4], fluid_wrench[5]));

      if (publish)
      {
        geometry_msgs::WrenchStamped wrench;
        wrench.header.stamp = time;
        wrench.header.frame_id = frame_prefix_ + "/" + entry.name;
        wrench.wrench.force.x = drag[0];
        wrench.wrench.force.y = drag[1];
        wrench.wrench.force.z = drag[2];
        wrench.wrench.torque.x = drag[3];
        wrench.wrench.torque.y = drag[4];
        wrench.wrench.torque.z = drag[5];
        entry.drag_publisher.publish(wrench);
      }
    }
    if (publish)
      last_hydrodynamics_debug_time_ = time;
  }

  struct LinkBuoyancy
  {
    std::string name;
    gazebo::physics::LinkPtr link;
    ignition::math::Vector3d cob;
    double volume;
    ros::Publisher publisher;
  };

  static bool number(const XmlRpc::XmlRpcValue& value, double& result)
  {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      result = static_cast<double>(value);
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      result = static_cast<int>(value);
    else
      return false;
    return std::isfinite(result);
  }

  bool enabled_ = false;
  bool hydrodynamics_enabled_ = false;
  double acceleration_filter_alpha_ = 0.3;
  double hydrodynamics_debug_rate_ = 20.0;
  Eigen::Vector3d current_velocity_world_ = Eigen::Vector3d::Zero();
  ros::Time last_hydrodynamics_debug_time_;
  std::vector<LinkHydrodynamics> hydrodynamics_links_;
  double debug_rate_ = 20.0;
  ros::Time last_debug_time_;
  std::string frame_prefix_;
  std::vector<LinkBuoyancy> links_;
  std::vector<double> last_finite_forces_;
  tf::TransformBroadcaster broadcaster_;
};

}  // namespace uuv_d_simulation

PLUGINLIB_EXPORT_CLASS(uuv_d_simulation::BuoyancyHWSim, gazebo_ros_control::RobotHWSim)
