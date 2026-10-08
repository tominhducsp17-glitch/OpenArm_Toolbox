// Joint-trajectory tracking controllers on the control-toolbox interface.
//
// Every controller is a ct::core::Controller<2 NJ, NJ>: computeControl(x = [q; dq], t, u) returns
// the joint torque u. The caller sets the reference (q_d, dq_d, ddq_d) before each call. The ROS 2
// node only talks to this interface, so a new controller (for example an MPC from ct_optcon) is
// added by deriving from JointTrackingController and registering it in makeTrackingController().
#pragma once

#include "openarm_control/openarm_dynamics.hpp"

#include <memory>
#include <string>
#include <vector>

namespace openarm_control
{

template<size_t NJ>
struct JointReference
{
  JointVector<NJ> q = JointVector<NJ>::Zero();
  JointVector<NJ> dq = JointVector<NJ>::Zero();
  JointVector<NJ> ddq = JointVector<NJ>::Zero();
};

struct TrackingControllerConfig
{
  std::string urdf_path;
  std::vector<std::string> joint_names;
  std::vector<double> torque_limit;  // |u_i| <= torque_limit_i [N m]; empty: no clamp
};

template<size_t NJ>
class JointTrackingController : public ct::core::Controller<2 * NJ, NJ>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit JointTrackingController(const TrackingControllerConfig & config);
  JointTrackingController(const JointTrackingController & other);

  void setReference(const JointReference<NJ> & reference);
  const JointReference<NJ> & reference() const {return reference_;}
  // Fingers or the other arm, used inside the model only.
  bool setPassiveJointPosition(const std::string & name, double position);
  const std::vector<std::string> & jointNames() const {return dynamics_->jointNames();}
  const OpenArmDynamics<NJ> & dynamics() const {return *dynamics_;}
  // True when the last computeControl() result was clamped by torque_limit.
  bool lastClamped() const {return clamped_;}
  virtual std::string name() const = 0;

  // ct::core::Controller: torque for the measured state x = [q; dq] at time t.
  void computeControl(
    const State<NJ> & state, const double & t, Torque<NJ> & control) final;

protected:
  // Unclamped torque of the concrete controller.
  virtual JointVector<NJ> torque(const State<NJ> & state, double t) = 0;
  std::shared_ptr<OpenArmDynamics<NJ>> dynamics_;
  JointReference<NJ> reference_;

private:
  JointVector<NJ> limit_;
  bool has_limit_ = false;
  bool clamped_ = false;
};

// CTC feedforward (model part of computed-torque control) for a driver that closes the PD loop:
// u = M(q_d) ddq_d + C(q_d, dq_d) dq_d + G(q_d). The measured state is not used.
template<size_t NJ>
class CtcFeedforwardController : public JointTrackingController<NJ>
{
public:
  using JointTrackingController<NJ>::JointTrackingController;
  CtcFeedforwardController * clone() const override {return new CtcFeedforwardController(*this);}
  std::string name() const override {return "ctc_feedforward";}

protected:
  JointVector<NJ> torque(const State<NJ> & state, double t) override;
};

// Gravity compensation at the reference: u = G(q_d).
template<size_t NJ>
class GravityCompensationController : public JointTrackingController<NJ>
{
public:
  using JointTrackingController<NJ>::JointTrackingController;
  GravityCompensationController * clone() const override {return new GravityCompensationController(*this);}
  std::string name() const override {return "gravity_compensation";}

protected:
  JointVector<NJ> torque(const State<NJ> & state, double t) override;
};

// "ctc_feedforward" | "gravity_compensation". Throws std::invalid_argument for other names.
template<size_t NJ>
std::unique_ptr<JointTrackingController<NJ>> makeTrackingController(
  const std::string & type, const TrackingControllerConfig & config);

std::vector<std::string> availableTrackingControllers();

// ddq_d from a stream of dq_d samples: finite difference, first-order low-pass, magnitude clamp.
// The first sample and non-increasing stamps return the previous estimate (zeros at start).
template<size_t NJ>
class AccelerationEstimator
{
public:
  AccelerationEstimator(double cutoff_hz, double max_abs_acceleration);
  JointVector<NJ> update(double t, const JointVector<NJ> & dq);
  void reset();

private:
  double cutoff_hz_, max_abs_;
  bool has_previous_ = false;
  double t_prev_ = 0.0;
  JointVector<NJ> dq_prev_ = JointVector<NJ>::Zero();
  JointVector<NJ> estimate_ = JointVector<NJ>::Zero();
};

extern template class JointTrackingController<7>;
extern template class JointTrackingController<14>;
extern template class CtcFeedforwardController<7>;
extern template class CtcFeedforwardController<14>;
extern template class GravityCompensationController<7>;
extern template class GravityCompensationController<14>;
extern template class AccelerationEstimator<7>;
extern template class AccelerationEstimator<14>;

}  // namespace openarm_control
