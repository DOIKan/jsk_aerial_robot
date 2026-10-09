#include <aerial_robot_simulation/aerial_robot_hw_sim.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

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
    frame_prefix_ = model_nh.getNamespace();
    if (!frame_prefix_.empty() && frame_prefix_.front() == '/')
      frame_prefix_.erase(0, 1);
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
  double debug_rate_ = 20.0;
  ros::Time last_debug_time_;
  std::string frame_prefix_;
  std::vector<LinkBuoyancy> links_;
  std::vector<double> last_finite_forces_;
  tf::TransformBroadcaster broadcaster_;
};

}  // namespace uuv_d_simulation

PLUGINLIB_EXPORT_CLASS(uuv_d_simulation::BuoyancyHWSim, gazebo_ros_control::RobotHWSim)
