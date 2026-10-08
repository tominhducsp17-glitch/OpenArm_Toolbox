#include "openarm_control/tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openarm_control
{

template<size_t NJ>
JointTrackingController<NJ>::JointTrackingController(const TrackingControllerConfig & config)
: dynamics_(std::make_shared<OpenArmDynamics<NJ>>(config.urdf_path, config.joint_names))
{
  if (!config.torque_limit.empty()) {
    if (config.torque_limit.size() != NJ) {
      throw std::invalid_argument("torque_limit must have one value per joint");
    }
    for (size_t j = 0; j < NJ; ++j) {
      if (!(config.torque_limit[j] > 0.0)) {throw std::invalid_argument("torque_limit must be > 0");}
      limit_[j] = config.torque_limit[j];
    }
    has_limit_ = true;
  }
}

template<size_t NJ>
JointTrackingController<NJ>::JointTrackingController(const JointTrackingController & other)
: ct::core::Controller<2 * NJ, NJ>(other),
  dynamics_(std::shared_ptr<OpenArmDynamics<NJ>>(other.dynamics_->clone())),
  reference_(other.reference_), limit_(other.limit_), has_limit_(other.has_limit_) {}

template<size_t NJ>
void JointTrackingController<NJ>::setReference(const JointReference<NJ> & reference)
{
  if (!reference.q.allFinite() || !reference.dq.allFinite() || !reference.ddq.allFinite()) {
    throw std::invalid_argument("reference must be finite");
  }
  reference_ = reference;
}

template<size_t NJ>
bool JointTrackingController<NJ>::setPassiveJointPosition(const std::string & name, double position)
{
  return dynamics_->setPassiveJointPosition(name, position);
}

template<size_t NJ>
void JointTrackingController<NJ>::computeControl(const State<NJ> & state, const double & t, Torque<NJ> & control)
{
  if (!state.allFinite()) {throw std::invalid_argument("state must be finite");}
  const JointVector<NJ> raw = torque(state, t);
  JointVector<NJ> out = raw;
  if (has_limit_) {out = raw.cwiseMax(-limit_).cwiseMin(limit_);}
  clamped_ = (out - raw).cwiseAbs().maxCoeff() > 0.0;
  control = out;
}

template<size_t NJ>
JointVector<NJ> CtcFeedforwardController<NJ>::torque(const State<NJ> & /*state*/, double /*t*/)
{
  const auto & r = this->reference_;
  return this->dynamics_->inverseDynamics(r.q, r.dq, r.ddq);
}

template<size_t NJ>
JointVector<NJ> GravityCompensationController<NJ>::torque(const State<NJ> & /*state*/, double /*t*/)
{
  return this->dynamics_->gravity(this->reference_.q);
}

template<size_t NJ>
std::unique_ptr<JointTrackingController<NJ>> makeTrackingController(
  const std::string & type, const TrackingControllerConfig & config)
{
  if (type == "ctc_feedforward") {return std::make_unique<CtcFeedforwardController<NJ>>(config);}
  if (type == "gravity_compensation") {return std::make_unique<GravityCompensationController<NJ>>(config);}
  throw std::invalid_argument("unknown controller type '" + type + "'");
}

std::vector<std::string> availableTrackingControllers()
{
  return {"ctc_feedforward", "gravity_compensation"};
}

template<size_t NJ>
AccelerationEstimator<NJ>::AccelerationEstimator(double cutoff_hz, double max_abs_acceleration)
: cutoff_hz_(cutoff_hz), max_abs_(max_abs_acceleration)
{
  if (!(cutoff_hz > 0.0) || !(max_abs_acceleration > 0.0)) {
    throw std::invalid_argument("AccelerationEstimator: cutoff and limit must be > 0");
  }
}

template<size_t NJ>
JointVector<NJ> AccelerationEstimator<NJ>::update(double t, const JointVector<NJ> & dq)
{
  if (has_previous_ && t > t_prev_) {
    const double dt = t - t_prev_;
    const JointVector<NJ> raw = (dq - dq_prev_) / dt;
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff_hz_ * dt);
    estimate_ += alpha * (raw - estimate_);
    estimate_ = estimate_.cwiseMax(-max_abs_).cwiseMin(max_abs_);
  }
  if (!has_previous_ || t > t_prev_) {
    has_previous_ = true;
    t_prev_ = t;
    dq_prev_ = dq;
  }
  return estimate_;
}

template<size_t NJ>
void AccelerationEstimator<NJ>::reset()
{
  has_previous_ = false;
  estimate_.setZero();
}

template class JointTrackingController<7>;
template class JointTrackingController<14>;
template class CtcFeedforwardController<7>;
template class CtcFeedforwardController<14>;
template class GravityCompensationController<7>;
template class GravityCompensationController<14>;
template class AccelerationEstimator<7>;
template class AccelerationEstimator<14>;
template std::unique_ptr<JointTrackingController<7>> makeTrackingController<7>(
  const std::string &, const TrackingControllerConfig &);
template std::unique_ptr<JointTrackingController<14>> makeTrackingController<14>(
  const std::string &, const TrackingControllerConfig &);

}  // namespace openarm_control
