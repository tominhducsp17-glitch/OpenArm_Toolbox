# Vendored control-toolbox (ETH ADRL)

Source: https://github.com/ethz-adrl/control-toolbox, commit `7d36e42` (2021-07-05), BSD-2 (LICENCE.txt).
Copied: `ct/cmake` (shared CMake helpers) and `ct_core` (header-only core: StateVector, ControlVector,
Controller, ControlledSystem, trajectories, integrators). `ct_core/doc` removed (Doxygen only).
Built by colcon as a plain CMake package (`build_type cmake` in ct_core/package.xml); options in
`ros2/colcon.meta` turn off Python plotting, tests and examples.

Not copied yet: `ct_optcon` (NLOC/MPC, add when an MPC controller is written), `ct_rbd` and `ct_models`
(RobCoGen dynamics; OpenArm dynamics come from Pinocchio in `openarm_control`).
Local change: `ct_core/CMakeLists.txt` adds `doc/` only if it exists (one `if(EXISTS ...)` guard).
Local change: `"${Python_VERSION_MAJOR}"` is quoted in two `if()` so CMake configures when Python is disabled.
