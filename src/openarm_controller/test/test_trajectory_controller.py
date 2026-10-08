"""Integration test: node "trajectory" + node "controller" over real ROS 2 topics.

A fake robot publishes joint_states (both arms hanging) at 200 Hz. The trajectory node must read the
start pose and go approach -> track -> return -> hold, publishing joint_commands (q, dq) at about
400 Hz. The controller node must answer each command with tau_ff for exactly the 7 commanded joints,
with the command's stamp, finite and inside torque_limit; in hold (dq = 0) the right shoulder torque
must equal gravity, which a second controller node of type gravity_compensation provides.
"""

import math
import os
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

RIGHT = [f"openarm_right_joint{j}" for j in range(1, 8)]
LEFT = [f"openarm_left_joint{j}" for j in range(1, 8)]
HANG = [0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0]


def write_trajectory() -> str:
    """1.5 s quintic of the right arm from the hanging pose, 400 Hz rows: t, q1..7, dq1..7."""
    goal = [0.3, 0.1, 0.0, 0.6, 0.0, 0.1, 0.0]
    path = os.path.join(tempfile.mkdtemp(), "test_trajectory.csv")
    with open(path, "w", encoding="utf-8") as f:
        f.write("t," + ",".join(f"q{j}" for j in range(1, 8)) + "," + ",".join(f"dq{j}" for j in range(1, 8)) + "\n")
        for k in range(601):
            t = k / 400.0
            s = t / 1.5
            b, db = s**3 * (10 - 15 * s + 6 * s * s), 30 * s * s * (1 - s) ** 2 / 1.5
            q = [h + b * (g - h) for h, g in zip(HANG, goal)]
            dq = [db * (g - h) for h, g in zip(HANG, goal)]
            f.write(",".join(f"{v:.9g}" for v in [t] + q + dq) + "\n")
    return path


@pytest.mark.launch_test
def generate_test_description():
    trajectory = launch_ros.actions.Node(
        package="openarm_trajectory", executable="trajectory_node", name="trajectory", output="screen",
        parameters=[{"trajectory_file": write_trajectory(), "rate_hz": 400.0, "min_move_s": 0.5,
                     "approach_speed": 1.0}])
    ctc = launch_ros.actions.Node(
        package="openarm_controller", executable="controller_node", name="controller", output="screen",
        parameters=[{"controller_type": "ctc_feedforward"}])
    gravity = launch_ros.actions.Node(
        package="openarm_controller", executable="controller_node", name="controller", output="screen",
        namespace="gravity", parameters=[{"controller_type": "gravity_compensation"}],
        remappings=[("joint_states", "/joint_states"), ("joint_commands", "/joint_commands")])
    return launch.LaunchDescription([trajectory, ctc, gravity, launch_testing.actions.ReadyToTest()])


class TestTrajectoryController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_pipeline(self):
        node = rclpy.create_node("fake_robot")
        commands, taus, gravity, phases = [], {}, {}, []
        node.create_subscription(JointState, "joint_commands",
                                 lambda m: commands.append((time.monotonic(), m)), 50)
        node.create_subscription(JointState, "controller/tau_ff",
                                 lambda m: taus.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m), 50)
        node.create_subscription(JointState, "/gravity/controller/tau_ff",
                                 lambda m: gravity.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m), 50)
        qos = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "trajectory/phase", lambda m: phases.append(m.data), qos)
        state_pub = node.create_publisher(JointState, "joint_states", 10)

        def publish_state():
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = LEFT + RIGHT
            msg.position = HANG + HANG
            msg.velocity = [0.0] * 14
            state_pub.publish(msg)

        node.create_timer(0.005, publish_state)
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline and "hold" not in phases:
            rclpy.spin_once(node, timeout_sec=0.01)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()

        self.assertEqual(phases[:5], ["wait_state", "approach", "track", "return", "hold"], phases)
        self.assertGreater(len(commands), 400)
        stamps = [t for t, _ in commands[-200:]]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        self.assertGreater(rate, 300.0, f"joint_commands at {rate:.0f} Hz")
        first = commands[0][1]
        self.assertEqual(list(first.name), RIGHT)
        self.assertEqual(len(first.velocity), 7)

        matched = 0
        for _, cmd in commands:
            key = (cmd.header.stamp.sec, cmd.header.stamp.nanosec)
            if key not in taus:
                continue
            matched += 1
            tau = taus[key]
            self.assertEqual(list(tau.name), RIGHT)
            self.assertTrue(all(math.isfinite(v) for v in tau.effort))
            self.assertTrue(all(abs(v) <= lim + 1e-9 for v, lim in zip(tau.effort, [15, 15, 10, 10, 3, 3, 3])))
        self.assertGreater(matched, 0.9 * len(commands), "controller answered too few commands")

        # Hold: dq = 0 and the estimator has settled, so CTC feedforward equals gravity at q_target.
        # Use the latest hold command answered by both controllers (the newest may still be in flight).
        held = [(c.header.stamp.sec, c.header.stamp.nanosec) for _, c in commands
                if all(v == 0.0 for v in c.velocity)]
        answered = [k for k in held if k in taus and k in gravity]
        self.assertTrue(answered, "no hold command answered by both controllers")
        key = answered[-1]
        for a, b in zip(taus[key].effort, gravity[key].effort):
            self.assertAlmostEqual(a, b, delta=1e-3)
