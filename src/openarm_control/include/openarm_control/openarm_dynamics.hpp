// OpenArm rigid-body model (official URDF, Pinocchio) as a control-toolbox ControlledSystem.
//
// State x = [q; dq] (2 NJ), control u = joint torque (NJ). Any ct::core controller or solver
// (CTC now, NLOC/MPC from ct_optcon later) can use the same model: forward dynamics through
// computeControlledDynamics, inverse dynamics and the terms M, C dq, G through the helpers.
#pragma once

#include <ct/core/core.h>

#include <memory>
#include <string>
#include <vector>

namespace openarm_control
{

template<size_t NJ>
using JointVector = Eigen::Matrix<double, NJ, 1>;
template<size_t NJ>
using State = ct::core::StateVector<2 * NJ>;
template<size_t NJ>
using Torque = ct::core::ControlVector<NJ>;

template<size_t NJ>
class OpenArmDynamics : public ct::core::ControlledSystem<2 * NJ, NJ>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // joint_names: the NJ controlled joints, in the order of every vector of this class.
  // Other URDF joints (other arm, fingers) are held at setPassiveJointPosition() values (0 by default).
  // Throws std::invalid_argument for an unreadable URDF or unknown/duplicate names.
  OpenArmDynamics(const std::string & urdf_path, const std::vector<std::string> & joint_names);
  OpenArmDynamics(const OpenArmDynamics & other);
  ~OpenArmDynamics() override;
  OpenArmDynamics * clone() const override;

  // dx = [dq; M(q)^-1 (u - C(q, dq) dq - G(q))]
  void computeControlledDynamics(
    const State<NJ> & state, const double & t, const Torque<NJ> & control,
    ct::core::StateVector<2 * NJ> & derivative) override;

  // M(q) ddq + C(q, dq) dq + G(q)
  JointVector<NJ> inverseDynamics(
    const JointVector<NJ> & q, const JointVector<NJ> & dq, const JointVector<NJ> & ddq) const;
  JointVector<NJ> gravity(const JointVector<NJ> & q) const;
  Eigen::Matrix<double, NJ, NJ> massMatrix(const JointVector<NJ> & q) const;

  const std::vector<std::string> & jointNames() const;
  // Uncontrolled scalar joints (other arm, fingers). Unknown names return false.
  bool setPassiveJointPosition(const std::string & name, double position);
  const std::vector<std::string> & passiveJointNames() const;
  JointVector<NJ> lowerLimits() const;
  JointVector<NJ> upperLimits() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

extern template class OpenArmDynamics<7>;
extern template class OpenArmDynamics<14>;

}  // namespace openarm_control
