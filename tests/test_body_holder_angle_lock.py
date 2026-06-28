import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pymunk
from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.procedure import BodyHolderAngleLock
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import ControlSignal, StateSignal, empty_control_signal


class FakeCoordinator:
    epsilon = 0.1
    rotation_speed = 2.0
    angle_rotation_speed = 0.75
    extension_speed = 4.0
    move_center_speed = 4.0
    num_legs = 3
    dt = 1 / 120
    max_extension = 2.0
    min_extension = 0.2


def make_state(body_angle=0.0, body_rate=0.0, active_grips=None):
    active_grips = active_grips or [False, True, True]
    return StateSignal(
        center_pos=Vec2d(0, 0),
        feet_pos=[Vec2d(0, 40), Vec2d(30, 0), Vec2d(-30, 0)],
        active_grips=active_grips,
        t=0,
        body_angle=body_angle,
        body_angular_velocity=body_rate,
    )


class BodyHolderAngleLockTest(unittest.TestCase):
    def test_locks_active_holding_limbs_when_free_leg_rotates(self):
        holder = BodyHolderAngleLock(FakeCoordinator(), 1, 2)
        signal = empty_control_signal(3)
        signal.rotation[0] = 0.75
        out = holder.adjust_signal(signal, make_state())

        self.assertTrue(out.angle_lock[1])
        self.assertTrue(out.angle_lock[2])
        self.assertEqual(out.rotation[1], 0.0)
        self.assertEqual(out.rotation[2], 0.0)

    def test_unlocks_when_free_leg_stops_rotating(self):
        holder = BodyHolderAngleLock(FakeCoordinator(), 1, 2)
        rotating = empty_control_signal(3)
        rotating.rotation[0] = 0.75
        holder.adjust_signal(rotating, make_state())

        idle = empty_control_signal(3)
        out = holder.adjust_signal(idle, make_state())
        self.assertFalse(out.angle_lock[1])
        self.assertFalse(out.angle_lock[2])


class PymunkBodyLegAngleLockTest(unittest.TestCase):
    def setUp(self):
        instance = load_instance("JumpTest", "instances")
        sim_config = SimConfig(screen_height=400, screen_width=400, fps=120, cell_size=20)
        robot_config = RobotConfig(
            epsilon=0.1,
            extension_speed=4,
            rotation_speed=4,
            move_center_speed=4,
            body_mass=5,
            body_radius=0.4,
            foot_mass=0.2,
            leg_mass=0.2,
            simplified_problem=True,
            leg_spring_stiffness=500,
            leg_spring_damping=1.5,
            min_extension=0.2,
        )
        self.coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
        self.simulator = MonkeyBotSimulator(sim_config)
        self.simulator.start_simulation(self.coordinator)

    def test_body_leg_angle_lock_preserves_relative_angle(self):
        leg_id = 1
        ref = self.simulator._body_leg_rel_angle(leg_id)
        self.simulator.lock_body_leg_angle(leg_id)
        self.assertTrue(self.simulator.is_body_leg_angle_locked(leg_id))

        self.simulator.rotation_motors[0].desired_rate = 1.5
        for _ in range(60):
            self.simulator.rotation_motors[0].step(self.simulator.sim_config.dt)
            self.simulator.space.step(self.simulator.sim_config.dt)

        locked_angle = self.simulator._body_leg_rel_angle(leg_id)
        self.assertAlmostEqual(locked_angle, ref, places=2)
        self.simulator.unlock_body_leg_angle(leg_id)
        self.assertFalse(self.simulator.is_body_leg_angle_locked(leg_id))

    def test_apply_signal_enables_and_disables_lock(self):
        signal = empty_control_signal(self.coordinator.num_legs)
        signal.angle_lock[1] = True
        self.simulator.apply_signal(signal)
        self.assertTrue(self.simulator.is_body_leg_angle_locked(1))

        signal.angle_lock[1] = False
        self.simulator.apply_signal(signal)
        self.assertFalse(self.simulator.is_body_leg_angle_locked(1))


if __name__ == "__main__":
    unittest.main()
