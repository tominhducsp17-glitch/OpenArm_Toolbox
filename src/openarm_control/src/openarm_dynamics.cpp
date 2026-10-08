#include "openarm_control/openarm_dynamics.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <set>
#include <stdexcept>

namespace openarm_control
{

template<size_t NJ>
struct OpenArmDynamics<NJ>::Impl
{
  std::string urdf_path;
  pinocchio::Model model;
  mutable pinocchio::Data data;
  std::vector<std::string> joint_names;
  std::vector<int> iq, iv;
  std::vector<std::string> passive_names;
  std::vector<int> passive_iq;
  mutable Eigen::VectorXd q_full, v_full, a_full;

  Impl(const std::string & urdf, const std::vector<std::string> & names)
  : urdf_path(urdf), joint_names(names)
  {
    if (names.size() != NJ) {
      throw std::invalid_argument("OpenArmDynamics: expected " + std::to_string(NJ) + " joint names");
    }
    try {
      pinocchio::urdf::buildModel(urdf, model);
    } catch (const std::exception & e) {
      throw std::invalid_argument("OpenArmDynamics: cannot read URDF " + urdf + ": " + e.what());
    }
    data = pinocchio::Data(model);
    if (std::set<std::string>(names.begin(), names.end()).size() != names.size()) {
      throw std::invalid_argument("OpenArmDynamics: duplicate joint names");
    }
    for (const auto & name : names) {
      if (!model.existJointName(name)) {throw std::invalid_argument("OpenArmDynamics: no joint " + name);}
      const auto & joint = model.joints[model.getJointId(name)];
      if (joint.nq() != 1 || joint.nv() != 1) {throw std::invalid_argument("OpenArmDynamics: not scalar " + name);}
      iq.push_back(joint.idx_q());
      iv.push_back(joint.idx_v());
    }
    for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(model.njoints); ++id) {
      const auto & name = model.names[id];
      if (std::find(names.begin(), names.end(), name) != names.end()) {continue;}
      if (model.joints[id].nq() == 1) {
        passive_names.push_back(name);
        passive_iq.push_back(model.joints[id].idx_q());
      }
    }
    q_full = pinocchio::neutral(model);
    v_full = Eigen::VectorXd::Zero(model.nv);
    a_full = Eigen::VectorXd::Zero(model.nv);
  }

  void load(const JointVector<NJ> & q, const JointVector<NJ> * dq, const JointVector<NJ> * ddq) const
  {
    if (!q.allFinite() || (dq && !dq->allFinite()) || (ddq && !ddq->allFinite())) {
      throw std::invalid_argument("OpenArmDynamics: non-finite input");
    }
    v_full.setZero();
    a_full.setZero();
    for (size_t j = 0; j < NJ; ++j) {
      q_full[iq[j]] = q[j];
      if (dq) {v_full[iv[j]] = (*dq)[j];}
      if (ddq) {a_full[iv[j]] = (*ddq)[j];}
    }
  }

  JointVector<NJ> pick(const Eigen::VectorXd & full) const
  {
    JointVector<NJ> out;
    for (size_t j = 0; j < NJ; ++j) {out[j] = full[iv[j]];}
    return out;
  }
};

template<size_t NJ>
OpenArmDynamics<NJ>::OpenArmDynamics(const std::string & urdf_path, const std::vector<std::string> & joint_names)
: impl_(std::make_unique<Impl>(urdf_path, joint_names)) {}

template<size_t NJ>
OpenArmDynamics<NJ>::OpenArmDynamics(const OpenArmDynamics & other)
: ct::core::ControlledSystem<2 * NJ, NJ>(other), impl_(std::make_unique<Impl>(*other.impl_)) {}

template<size_t NJ>
OpenArmDynamics<NJ>::~OpenArmDynamics() = default;

template<size_t NJ>
OpenArmDynamics<NJ> * OpenArmDynamics<NJ>::clone() const {return new OpenArmDynamics(*this);}

template<size_t NJ>
void OpenArmDynamics<NJ>::computeControlledDynamics(
  const State<NJ> & state, const double & /*t*/, const Torque<NJ> & control,
  ct::core::StateVector<2 * NJ> & derivative)
{
  const JointVector<NJ> q = state.template head<NJ>();
  const JointVector<NJ> dq = state.template tail<NJ>();
  // Forward dynamics of the controlled joints with all other joints fixed:
  // M_cc ddq = u - (C dq + G)_c
  const Eigen::Matrix<double, NJ, NJ> m = massMatrix(q);
  const JointVector<NJ> bias = inverseDynamics(q, dq, JointVector<NJ>::Zero());
  derivative.template head<NJ>() = dq;
  derivative.template tail<NJ>() = m.ldlt().solve(JointVector<NJ>(control) - bias);
}

template<size_t NJ>
JointVector<NJ> OpenArmDynamics<NJ>::inverseDynamics(
  const JointVector<NJ> & q, const JointVector<NJ> & dq, const JointVector<NJ> & ddq) const
{
  impl_->load(q, &dq, &ddq);
  return impl_->pick(pinocchio::rnea(impl_->model, impl_->data, impl_->q_full, impl_->v_full, impl_->a_full));
}

template<size_t NJ>
JointVector<NJ> OpenArmDynamics<NJ>::gravity(const JointVector<NJ> & q) const
{
  impl_->load(q, nullptr, nullptr);
  return impl_->pick(pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, impl_->q_full));
}

template<size_t NJ>
Eigen::Matrix<double, NJ, NJ> OpenArmDynamics<NJ>::massMatrix(const JointVector<NJ> & q) const
{
  impl_->load(q, nullptr, nullptr);
  pinocchio::crba(impl_->model, impl_->data, impl_->q_full);
  Eigen::Matrix<double, NJ, NJ> m;
  for (size_t r = 0; r < NJ; ++r) {
    for (size_t c = 0; c < NJ; ++c) {
      const int a = std::min(impl_->iv[r], impl_->iv[c]), b = std::max(impl_->iv[r], impl_->iv[c]);
      m(r, c) = impl_->data.M(a, b);  // crba fills the upper triangle
    }
  }
  return m;
}

template<size_t NJ>
const std::vector<std::string> & OpenArmDynamics<NJ>::jointNames() const {return impl_->joint_names;}

template<size_t NJ>
bool OpenArmDynamics<NJ>::setPassiveJointPosition(const std::string & name, double position)
{
  for (size_t k = 0; k < impl_->passive_names.size(); ++k) {
    if (impl_->passive_names[k] == name) {
      impl_->q_full[impl_->passive_iq[k]] = position;
      return true;
    }
  }
  return false;
}

template<size_t NJ>
const std::vector<std::string> & OpenArmDynamics<NJ>::passiveJointNames() const {return impl_->passive_names;}

template<size_t NJ>
JointVector<NJ> OpenArmDynamics<NJ>::lowerLimits() const
{
  JointVector<NJ> out;
  for (size_t j = 0; j < NJ; ++j) {out[j] = impl_->model.lowerPositionLimit[impl_->iq[j]];}
  return out;
}

template<size_t NJ>
JointVector<NJ> OpenArmDynamics<NJ>::upperLimits() const
{
  JointVector<NJ> out;
  for (size_t j = 0; j < NJ; ++j) {out[j] = impl_->model.upperPositionLimit[impl_->iq[j]];}
  return out;
}

template class OpenArmDynamics<7>;
template class OpenArmDynamics<14>;

}  // namespace openarm_control
