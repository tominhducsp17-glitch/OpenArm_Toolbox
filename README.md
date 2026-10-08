# OpenArm_Toolbox

ROS 2 (Humble, colcon) packages for model-based control of the OpenArm v1 bimanual robot, built on the
[control-toolbox](https://github.com/ethz-adrl/control-toolbox) interface so that the CTC controller used
today can later be replaced (for example by an MPC) without changing the nodes.

```
                     joint_states (measured q, dq)
                 ┌──────────────────────────────────────────────┐
                 ▼                                              ▼
  ┌──────────────────────┐  joint_commands (q, dq)  ┌──────────────────────┐  controller/tau_ff
  │ node "trajectory"    │ ───────────────────────▶ │ node "controller"    │ ─────────────────▶ robot driver
  │ openarm_trajectory   │        400 Hz            │ openarm_controller   │  (effort, N·m)     (MIT: kp, kd,
  └──────────────────────┘                          └──────────┬───────────┘                    q, dq, tau_ff)
                                                               │ calls (no node inside)
                                                    ┌──────────▼───────────┐
                                                    │ library               │
                                                    │ openarm_control       │ ct::core::Controller
                                                    │  + ct_core (CT)       │ ct::core::ControlledSystem
                                                    └───────────────────────┘
```

| Package | Kind | Content |
| --- | --- | --- |
| `control_toolbox/ct_core` | library (header-only) | control-toolbox core, ETH ADRL commit `7d36e42`, BSD-2. See `src/control_toolbox/VENDORED.md` |
| `openarm_control` | library, **no node** | `OpenArmDynamics` (`ct::core::ControlledSystem`, Pinocchio, official URDF), `JointTrackingController` (`ct::core::Controller`): `ctc_feedforward`, `gravity_compensation`; factory `makeTrackingController`; `AccelerationEstimator` |
| `openarm_trajectory` | node `trajectory_node` | plays a trajectory file, publishes `joint_commands` (q, dq) |
| `openarm_controller` | node `controller_node` | subscribes `joint_states` + `joint_commands`, calls the library, publishes `controller/tau_ff` |

## Control law

The motor drivers run MIT control `τ = kp·(q_target − q) + kd·(q̇_target − q̇) + τ_ff`.
The controller node supplies only `τ_ff`. With `controller_type: ctc_feedforward`:

`τ_ff = M(q_d)·q̈_d + C(q_d, q̇_d)·q̇_d + G(q_d)` (model part of computed-torque control; q̈_d is estimated from q̇_d)

Conventions: joint angle = motor angle as in the official OpenArm URDF; torque in N·m; joints are matched by
name (`openarm_left_joint1..7`, `openarm_right_joint1..7`).

## Build and test

```bash
docker build -t openarm_toolbox:humble docker      # ROS 2 Humble + Pinocchio (or use a native Humble install)
docker run --rm -it -v $PWD:/ws -w /ws openarm_toolbox:humble bash
source /opt/ros/humble/setup.bash
colcon build                                       # colcon.meta sets the ct_core options
colcon test && colcon test-result --verbose
```

Tests (7): the library matches 40 golden cases of the validated model to 1e-9 N·m and runs inside a
`ct::core::Integrator` simulation; a launch test runs both nodes over real topics (approach, track, return,
hold; τ_ff for every command, with its stamp and inside the torque limit).

## Run

```bash
source install/setup.bash
ros2 launch openarm_controller trajectory_controller.launch.py           # tissue_wipe_x2 (59 s), right arm
ros2 launch openarm_controller trajectory_controller.launch.py \
  trajectory_file:=/path/to/file.csv controller_type:=gravity_compensation
```

The robot side publishes `joint_states` and sends, for each joint, MIT `(kp, kd, q_target, q̇_target, τ_ff)` from
`joint_commands` and `controller/tau_ff`.

### Node "trajectory" (`openarm_trajectory/trajectory_node`)

Waits for `joint_states`, moves smoothly (quintic, peak speed ≤ `approach_speed`) from the measured pose to the
first row, plays the file, returns to the measured start pose and holds it.

| Topic | Type | Content |
| --- | --- | --- |
| `joint_states` (in) | `sensor_msgs/JointState` | measured pose |
| `joint_commands` (out) | `sensor_msgs/JointState` | `position` = q_target, `velocity` = q̇_target, at `rate_hz` |
| `trajectory/phase` (out, latched) | `std_msgs/String` | `wait_state`, `approach`, `track`, `return`, `hold` |

Parameters: `trajectory_file`, `joint_names` (7, default right arm), `rate_hz` (400), `approach_speed` (0.25 rad/s),
`min_move_s` (2.0), `return_to_start` (true). File: CSV with a header line and columns
`t, q1..q7, dq1..dq7[, ddq1..ddq7]`, first row `t = 0`. Included in `openarm_trajectory/trajectories`:
`tissue_wipe.csv` (29.6 s, ≤ 0.56 rad/s) and `tissue_wipe_x2.csv` (59 s, ≤ 0.28 rad/s): reach a box low in front,
lift, wipe twice sideways, move aside, return. Both were checked against the URDF limits and for self-collision.

### Node "controller" (`openarm_controller/controller_node`)

One output per received command, stamped with the command's stamp. Joints that a command does not contain (the
other arm) are held at their measured pose inside the model and get no output.

| Topic | Type | Content |
| --- | --- | --- |
| `joint_states` (in) | `sensor_msgs/JointState` | measured q, q̇ (+ finger positions) |
| `joint_commands` (in) | `sensor_msgs/JointState` | q_target, q̇_target |
| `controller/tau_ff` (out) | `sensor_msgs/JointState` | `effort` = τ_ff [N·m] for the commanded joints |

Parameters (`openarm_controller/config/controller.yaml`): `joint_names` (7 or 14), `controller_type`,
`torque_limit`, `acceleration_cutoff_hz`, `max_abs_acceleration`, `input_timeout_s`, topic names, `urdf_path`
(empty: the URDF installed with `openarm_control`).

## Library use (without ROS)

```cpp
#include <openarm_control/tracking_controller.hpp>

openarm_control::TrackingControllerConfig config;
config.urdf_path = ".../openarm_v1_bimanual.urdf";
config.joint_names = {"openarm_right_joint1", /* ... */ "openarm_right_joint7"};
auto controller = openarm_control::makeTrackingController<7>("ctc_feedforward", config);

controller->setReference({q_d, dq_d, ddq_d});
openarm_control::State<7> x;  x << q, dq;          // ct::core::StateVector<14>
openarm_control::Torque<7> tau_ff;                   // ct::core::ControlVector<7>
controller->computeControl(x, t, tau_ff);            // ct::core::Controller interface
```

CMake: `find_package(openarm_control REQUIRED)` and `target_link_libraries(app openarm_control::openarm_control)`.

## Adding another controller (e.g. MPC)

1. Derive from `openarm_control::JointTrackingController<NJ>` and implement `torque(state, t)`, `clone()` and
   `name()`. The model is available as `dynamics_` (`OpenArmDynamics`, a `ct::core::ControlledSystem`).
2. Register the name in `makeTrackingController()`.
3. Select it with `controller_type:=<name>`. Nodes, topics and messages stay the same.

For an MPC, add `ct_optcon` from the same control-toolbox commit next to `ct_core`.

## Results on the real robot (right arm, 07/10/2026)

`tissue_wipe_x2`, 400 Hz, driver gains kp 35 / kd 1.2, CTC feedforward (the same model and law as
`ctc_feedforward` here): completed without a safety stop. Joint RMSE J1–J7 = 18.8 / 20.6 / 10.9 / 28.9 / 3.2 /
2.4 / 4.2 mrad. The gravity model matches the measured holding torque within about 5 % on J1 and J4. The remaining
error comes from joint friction, which is not compensated yet.

## License

Apache-2.0 for the `openarm_*` packages (`LICENSE`). `src/control_toolbox` is BSD-2 (ETH Zurich; see its
`LICENCE.txt` and `NOTICE.txt`).
