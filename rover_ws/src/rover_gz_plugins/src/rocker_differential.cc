// Rocker-bogie differential for Gazebo Sim (Harmonic / gz-sim 8).
//
// WHY THIS EXISTS
// On the real rover the two rockers are tied together by a differential:
// each rocker's upward extension is linked by a tie rod to a lever that
// pivots on the chassis, so when one rocker rotates up the other rotates
// down by the same angle (theta_left + theta_right = 0). The chassis then
// rides at the AVERAGE of the two sides' terrain instead of flopping.
//
// That linkage is a closed kinematic loop, and gz-physics refuses to build
// one: a loop-closing joint is rejected with "child link already has a
// parent joint", so the tie rods would simply dangle. SDF <mimic> would
// express the same constraint, but the DART engine (the only one that can
// do controllable skid-steer here) ignores mimic constraints too.
//
// Without the constraint the suspension has one free internal degree of
// freedom -- both rockers rotate together, the chassis pitches over onto
// the joint limits, and the lidar tilts ~20 deg, which wrecks SLAM.
//
// So this system applies the constraint force the real tie rods would
// apply: a stiff PD that drives (theta_left + theta_right) to zero. It runs
// inside the physics loop, so it works under any engine.
//
// Sign note: the constraint is e = theta_left + theta_right, and
// de/dtheta_left == de/dtheta_right == 1, so the SAME corrective torque
// goes to both joints. The differential mode (theta_left = -theta_right)
// leaves e unchanged and is therefore completely unresisted -- the
// suspension still articulates freely, exactly like the real mechanism.

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace rover_gz_plugins
{
class RockerDifferential
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    if (!model.Valid(_ecm))
    {
      gzerr << "RockerDifferential must be attached to a model entity.\n";
      return;
    }

    const std::string leftName =
        _sdf->Get<std::string>("left_joint", "left_rocker_joint").first;
    const std::string rightName =
        _sdf->Get<std::string>("right_joint", "right_rocker_joint").first;
    this->kp = _sdf->Get<double>("p_gain", this->kp).first;
    this->kd = _sdf->Get<double>("d_gain", this->kd).first;
    this->maxTorque = _sdf->Get<double>("max_torque", this->maxTorque).first;

    this->leftJoint = model.JointByName(_ecm, leftName);
    this->rightJoint = model.JointByName(_ecm, rightName);
    if (this->leftJoint == gz::sim::kNullEntity ||
        this->rightJoint == gz::sim::kNullEntity)
    {
      gzerr << "RockerDifferential: could not find joints [" << leftName
            << "] and/or [" << rightName << "]. Differential disabled.\n";
      return;
    }

    // physics only populates these components when something asks for them
    for (const auto joint : {this->leftJoint, this->rightJoint})
    {
      if (!_ecm.Component<gz::sim::components::JointPosition>(joint))
        _ecm.CreateComponent(joint, gz::sim::components::JointPosition());
      if (!_ecm.Component<gz::sim::components::JointVelocity>(joint))
        _ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
    }

    this->ready = true;
    gzmsg << "RockerDifferential: tying [" << leftName << "] to [" << rightName
          << "] (kp=" << this->kp << " kd=" << this->kd << ")\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->ready || _info.paused)
      return;

    const auto *leftPos =
        _ecm.Component<gz::sim::components::JointPosition>(this->leftJoint);
    const auto *rightPos =
        _ecm.Component<gz::sim::components::JointPosition>(this->rightJoint);
    const auto *leftVel =
        _ecm.Component<gz::sim::components::JointVelocity>(this->leftJoint);
    const auto *rightVel =
        _ecm.Component<gz::sim::components::JointVelocity>(this->rightJoint);

    if (!leftPos || !rightPos || !leftVel || !rightVel ||
        leftPos->Data().empty() || rightPos->Data().empty() ||
        leftVel->Data().empty() || rightVel->Data().empty())
    {
      return;  // components not populated yet (first steps)
    }

    const double error = leftPos->Data()[0] + rightPos->Data()[0];
    const double errorRate = leftVel->Data()[0] + rightVel->Data()[0];
    const double torque = std::clamp(-(this->kp * error + this->kd * errorRate),
                                     -this->maxTorque, this->maxTorque);

    for (const auto joint : {this->leftJoint, this->rightJoint})
    {
      auto *cmd = _ecm.Component<gz::sim::components::JointForceCmd>(joint);
      if (!cmd)
      {
        _ecm.CreateComponent(
            joint, gz::sim::components::JointForceCmd({torque}));
      }
      else if (cmd->Data().empty())
      {
        cmd->Data() = {torque};
      }
      else
      {
        cmd->Data()[0] = torque;
      }
    }
  }

private:
  gz::sim::Entity leftJoint{gz::sim::kNullEntity};
  gz::sim::Entity rightJoint{gz::sim::kNullEntity};
  double kp{250.0};
  double kd{15.0};
  double maxTorque{80.0};
  bool ready{false};
};
}  // namespace rover_gz_plugins

GZ_ADD_PLUGIN(rover_gz_plugins::RockerDifferential, gz::sim::System,
              rover_gz_plugins::RockerDifferential::ISystemConfigure,
              rover_gz_plugins::RockerDifferential::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(rover_gz_plugins::RockerDifferential,
                    "rover_gz_plugins::RockerDifferential")
