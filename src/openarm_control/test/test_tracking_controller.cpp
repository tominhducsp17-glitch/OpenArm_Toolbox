// The control-toolbox library against the validated golden vectors and inside a ct::core simulation.
#include "openarm_control/tracking_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

using namespace openarm_control;

namespace
{
std::string urdf()
{
  return ament_index_cpp::get_package_share_directory("openarm_control") + "/urdf/openarm_v1_bimanual.urdf";
}

std::vector<std::string> bimanual()
{
  std::vector<std::string> names;
  for (const char * side : {"left", "right"}) {
    for (int j = 1; j <= 7; ++j) {names.push_back(std::string("openarm_") + side + "_joint" + std::to_string(j));}
  }
  return names;
}

const std::vector<std::string> kFingers{"openarm_left_finger_joint1", "openarm_left_finger_joint2",
  "openarm_right_finger_joint1", "openarm_right_finger_joint2"};

// Rows: q_d dq_d ddq_d q_m (14 each), fingers (4), tau_id, tau_g_target, tau_g_measured, mass_diag (14 each).
std::vector<std::vector<double>> golden()
{
  std::ifstream file(GOLDEN_CSV);
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::stringstream in(line);
    std::vector<double> row;
    double v;
    while (in >> v) {row.push_back(v);}
    rows.push_back(row);
  }
  return rows;
}

JointVector<14> slice(const std::vector<double> & row, int block)
{
  JointVector<14> v;
  const int offset = block < 4 ? 14 * block : 4 + 14 * block;
  for (int j = 0; j < 14; ++j) {v[j] = row[offset + j];}
  return v;
}

TrackingControllerConfig config(const std::vector<std::string> & names)
{
  TrackingControllerConfig c;
  c.urdf_path = urdf();
  c.joint_names = names;
  return c;
}
}  // namespace

TEST(OpenArmControl, CtcFeedforwardMatchesGoldenVectors)
{
  const auto rows = golden();
  ASSERT_EQ(rows.size(), 40u);
  auto ctc = makeTrackingController<14>("ctc_feedforward", config(bimanual()));
  auto grav = makeTrackingController<14>("gravity_compensation", config(bimanual()));
  for (const auto & row : rows) {
    for (int f = 0; f < 4; ++f) {
      ctc->setPassiveJointPosition(kFingers[f], row[56 + f]);
      grav->setPassiveJointPosition(kFingers[f], row[56 + f]);
    }
    JointReference<14> ref{slice(row, 0), slice(row, 1), slice(row, 2)};
    ctc->setReference(ref);
    grav->setReference(ref);
    State<14> x;
    x << slice(row, 3), JointVector<14>::Zero();
    Torque<14> u;
    ctc->computeControl(x, 0.0, u);
    EXPECT_LT((JointVector<14>(u) - slice(row, 4)).cwiseAbs().maxCoeff(), 1e-9);
    grav->computeControl(x, 0.0, u);
    EXPECT_LT((JointVector<14>(u) - slice(row, 5)).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((ctc->dynamics().massMatrix(ref.q).diagonal() - slice(row, 7)).cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(OpenArmControl, PlugsIntoControlToolboxSimulation)
{
  // ct::core: ControlledSystem + Controller + Integrator. With u = G(q0) the arm held at q0 stays there.
  std::vector<std::string> right;
  for (int j = 1; j <= 7; ++j) {right.push_back("openarm_right_joint" + std::to_string(j));}
  auto system = std::make_shared<OpenArmDynamics<7>>(urdf(), right);
  auto controller = std::make_shared<GravityCompensationController<7>>(config(right));
  JointReference<7> ref;
  ref.q << 0.4, 0.1, 0.0, 0.9, 0.0, -0.2, 0.0;
  controller->setReference(ref);
  system->setController(controller);
  ct::core::Integrator<14> integrator(system, ct::core::IntegrationType::RK4);
  State<7> x;
  x << ref.q, JointVector<7>::Zero();
  integrator.integrate_n_steps(x, 0.0, 500, 0.001);
  EXPECT_LT((x.head<7>() - ref.q).cwiseAbs().maxCoeff(), 1e-6);

  // Without the controller torque (u = 0) the same arm falls: the model and the integrator are live.
  State<7> fall;
  fall << ref.q, JointVector<7>::Zero();
  auto zero = std::make_shared<ct::core::ConstantController<14, 7>>();
  system->setController(zero);
  integrator.integrate_n_steps(fall, 0.0, 200, 0.001);
  EXPECT_GT((fall.head<7>() - ref.q).cwiseAbs().maxCoeff(), 1e-3);
}

TEST(OpenArmControl, TorqueLimitAndRejection)
{
  auto c = config(bimanual());
  c.torque_limit.assign(14, 0.5);
  auto ctc = makeTrackingController<14>("ctc_feedforward", c);
  JointReference<14> ref;
  ref.q.setConstant(0.3);
  ref.q[3] = ref.q[10] = 1.0;
  ctc->setReference(ref);
  State<14> x = State<14>::Zero();
  Torque<14> u;
  ctc->computeControl(x, 0.0, u);
  EXPECT_LE(u.cwiseAbs().maxCoeff(), 0.5);
  EXPECT_TRUE(ctc->lastClamped());
  ref.q[0] = std::nan("");
  EXPECT_THROW(ctc->setReference(ref), std::invalid_argument);
  EXPECT_THROW(makeTrackingController<14>("mpc", config(bimanual())), std::invalid_argument);
  auto bad = config(bimanual());
  bad.joint_names[0] = "nope";
  EXPECT_THROW(makeTrackingController<14>("ctc_feedforward", bad), std::invalid_argument);
}

TEST(OpenArmControl, AccelerationEstimator)
{
  AccelerationEstimator<7> est(5.0, 20.0);
  JointVector<7> dq = JointVector<7>::Zero();
  EXPECT_EQ(est.update(0.0, dq).norm(), 0.0);
  for (int k = 1; k <= 400; ++k) {
    dq.setConstant(0.5 * k / 400.0);  // ddq = 0.5 rad/s^2
    est.update(k / 400.0, dq);
  }
  EXPECT_NEAR(est.update(1.0, dq)[0], 0.5, 0.01);
}
