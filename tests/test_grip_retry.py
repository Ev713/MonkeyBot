import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from unittest.mock import patch

from monkey_bot.procedure import AttachCatch, Grabber
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import empty_control_signal, StateSignal


class GripRetryTest(unittest.TestCase):
    def setUp(self):
        instance = load_instance("GPT1", "instances")
        sim_config = SimConfig(screen_height=400, screen_width=400, fps=120, cell_size=20)
        robot_config = RobotConfig(
            epsilon=0.1,
            extension_speed=4,
            rotation_speed=2,
            angle_rotation_speed=0.75,
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

    def test_check_and_grip_does_not_raise_when_out_of_range(self):
        self.simulator.release_grip(0)
        foot_body, _ = self.simulator.feet[0]
        foot_body.position = (0, 0)
        self.assertFalse(self.simulator.check_and_grip(0))

    def test_attach_catch_not_finished_until_gripped(self):
        self.simulator.release_grip(0)
        gp = self.coordinator.grid_to_screen(6, 8)
        catcher = AttachCatch(0, gp, self.coordinator, (1, 2))
        state = self.simulator.get_state()
        self.assertFalse(catcher.is_finished(state))

    def test_skips_move_center_when_foot_already_in_reach(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        foot_near_gp = gp + Vec2d(self.coordinator.cell_size * 0.8, 0)
        state = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[
                self.coordinator.grid_to_screen(6, 8),
                foot_near_gp,
                self.coordinator.grid_to_screen(4, 6),
            ],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        with patch.object(catcher.get_closer, "adjust_signal") as get_closer, patch.object(
            catcher.tracker, "adjust_signal", wraps=catcher.tracker.adjust_signal
        ) as tracker:
            out = catcher.adjust_signal(signal, state)
            get_closer.assert_not_called()
            tracker.assert_called_once()
            self.assertFalse(catcher._in_pull_up_phase(state))

    def test_does_not_skip_move_center_when_foot_far_from_gp(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        foot_far = gp + Vec2d(self.coordinator.cell_size * 3.09, 0)
        center_far = gp + Vec2d(0, -self.coordinator.cell_size * 3.53)
        state = StateSignal(
            center_pos=center_far,
            feet_pos=[
                self.coordinator.grid_to_screen(6, 8),
                foot_far,
                self.coordinator.grid_to_screen(4, 6),
            ],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        with patch.object(catcher.get_closer, "adjust_signal", wraps=catcher.get_closer.adjust_signal) as get_closer, patch.object(
            catcher.tracker, "adjust_signal"
        ) as tracker:
            catcher.adjust_signal(signal, state)
            get_closer.assert_called_once()
            tracker.assert_not_called()
            self.assertTrue(catcher._in_pull_up_phase(state))

    def test_groove_safe_length_caps_reach_extension_rate(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        near_center = StateSignal(
            center_pos=gp + Vec2d(0, self.coordinator.cell_size * 2.5),
            feet_pos=[Vec2d(30, 0), gp + Vec2d(self.coordinator.cell_size * 2.0, 0), Vec2d(-30, 0)],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        out = catcher.adjust_signal(signal, near_center)
        max_rate = catcher.extension_speed * 0.3
        self.assertLessEqual(abs(out.extension[1]), max_rate + 1e-6)

    def test_pull_up_zeros_attach_leg_motors(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        far_center = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(30, 0), Vec2d(0, 0), Vec2d(-30, 0)],
            active_grips=[True, False, True],
            t=0,
            body_angular_velocity=3.0,
        )
        out = catcher.adjust_signal(empty_control_signal(3), far_center)
        self.assertEqual(out.rotation[1], 0.0)
        self.assertEqual(out.extension[1], 0.0)
        self.assertNotEqual(out.extension[0], 0.0)
        self.assertNotEqual(out.extension[2], 0.0)

    def test_attach_catch_skips_tracker_and_body_holder_during_move_center(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        far_center = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(30, 0), Vec2d(0, 0), Vec2d(-30, 0)],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        with patch.object(catcher.tracker, "adjust_signal") as tracker, patch.object(
            catcher.body_holder, "adjust_signal"
        ) as body_holder, patch.object(catcher.grabber, "adjust_signal") as grabber:
            catcher.adjust_signal(signal, far_center)
            tracker.assert_not_called()
            body_holder.assert_not_called()
            grabber.assert_not_called()

    def test_attach_catch_skips_body_holder_while_get_closer_active(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        far_center = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(30, 0), Vec2d(0, 0), Vec2d(-30, 0)],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        signal.rotation[1] = 0.5
        with patch.object(catcher.body_holder, "adjust_signal") as body_holder:
            catcher.adjust_signal(signal, far_center)
            body_holder.assert_not_called()

    def test_attach_catch_uses_tracker_after_move_center(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        near_center = StateSignal(
            center_pos=gp + Vec2d(0, 1),
            feet_pos=[Vec2d(30, 0), Vec2d(0, 0), Vec2d(-30, 0)],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        with patch.object(catcher.get_closer, "adjust_signal") as get_closer, patch.object(
            catcher.tracker, "adjust_signal", wraps=catcher.tracker.adjust_signal
        ) as tracker:
            catcher.adjust_signal(signal, near_center)
            get_closer.assert_not_called()
            tracker.assert_called_once()

    def test_attach_catch_uses_body_holder_after_get_closer_finishes(self):
        gp = self.coordinator.grid_to_screen(4, 12)
        catcher = AttachCatch(1, gp, self.coordinator, (0, 2))
        near_center = StateSignal(
            center_pos=gp + Vec2d(0, 1),
            feet_pos=[Vec2d(30, 0), Vec2d(0, 0), Vec2d(-30, 0)],
            active_grips=[True, False, True],
            t=0,
        )
        signal = empty_control_signal(3)
        signal.rotation[1] = 0.5
        with patch.object(catcher.body_holder, "adjust_signal", wraps=catcher.body_holder.adjust_signal) as body_holder:
            catcher.adjust_signal(signal, near_center)
            body_holder.assert_called_once()

    def test_grabber_requests_grip_only_when_close(self):
        gp = self.coordinator.grid_to_screen(6, 8)
        grabber = Grabber(0, gp, self.coordinator)
        far = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(0, 0), Vec2d(10, 0), Vec2d(-10, 0)],
            active_grips=[False, True, True],
            t=0,
        )
        signal = empty_control_signal(3)
        out = grabber.adjust_signal(signal, far)
        self.assertEqual(out.grip[0], 0)


if __name__ == "__main__":
    unittest.main()
