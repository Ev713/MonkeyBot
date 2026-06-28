import logging
import random
import threading
import time
from typing import Optional

import pygame

from monkey_bot.config import RobotConfig, SimConfig
from monkey_bot.goal_generator import build_replan_instance
from monkey_bot.simplified_graph_planner import analyze_reachability
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.robot_controller import SimplifiedProblemController
from monkey_bot.signals import ControlSignal, empty_control_signal
from monkey_bot.sim_runner import SimRunner


class InfiniteDemoRunner(SimRunner):
    """
    Runs a climbing demo that never stops: execute a plan, reach the goal,
    pick a new nearby goal, replan in the background, and continue.

    The Pymunk/pygame loop runs on the main thread (required for macOS windowing).
    Planning runs in a background thread so solver pauses do not slow the sim clock.
    """

    def __init__(
        self,
        instance: MonkeyBotProblemInstance,
        sim_config: SimConfig,
        robot_config: RobotConfig,
        *,
        seed: Optional[int] = None,
        max_plan_attempts: int = 12,
    ):
        super().__init__(instance, sim_config, robot_config)
        self.base_instance = instance
        self.rng = random.Random(seed)
        self.max_plan_attempts = max_plan_attempts

        self._stop = threading.Event()
        self._plan_request = threading.Event()
        self._new_plan_ready = threading.Event()
        self._signal_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._plan_lock = threading.Lock()

        self._planning = False
        self._segments_completed = 0
        self._segment_zero_failures = 0
        self._max_segment_zero_failures = 3
        self._pending_instance: Optional[MonkeyBotProblemInstance] = None
        self._pending_plan = None
        self._status_message = "Starting..."

        num_legs = self.config.num_legs
        self.current_signal = empty_control_signal(num_legs)
        self.latest_state = None

    def start_controller(self, manual=False, enable_transitional_links=True):
        if manual:
            raise ValueError("InfiniteDemoRunner does not support manual control")
        self.controller = SimplifiedProblemController(
            self.config,
            enable_transition_links=enable_transitional_links,
        )

    def execute_infinite_demo(
        self,
        *,
        enable_transitional_links: bool = True,
    ):
        self.start_controller(enable_transitional_links=enable_transitional_links)

        # Pygame/SDL on macOS must init and render on the main thread.
        self.start_simulator()
        self._plan_request.set()

        control_thread = threading.Thread(
            target=self._control_loop, name="infinite-demo-control", daemon=True
        )
        planner_thread = threading.Thread(
            target=self._planner_loop, name="infinite-demo-planner", daemon=True
        )

        control_thread.start()
        planner_thread.start()

        print("Infinite demo running — close the window or press Ctrl+C to stop.", flush=True)

        try:
            self._sim_loop()
        except KeyboardInterrupt:
            self.stop()
        finally:
            self.stop()
            control_thread.join(timeout=2.0)
            planner_thread.join(timeout=2.0)

    def stop(self):
        self._stop.set()
        self._plan_request.set()
        if self.simulator is not None:
            self.simulator.run = False

    def _sim_loop(self):
        while self.simulator.run and not self._stop.is_set():
            with self._signal_lock:
                signal = self.current_signal

            self.simulator.apply_signal(signal)
            self._update_window_caption()
            self.simulator.step()

            with self._state_lock:
                self.latest_state = self.simulator.get_state()

    def _control_loop(self):
        while self.simulator.run and not self._stop.is_set():
            if self._new_plan_ready.is_set():
                self._apply_pending_plan()

            with self._state_lock:
                state = self.latest_state

            if state is None:
                time.sleep(0.001)
                continue

            if not self.controller.plan:
                hold = self.controller.get_empty_sig()
                with self._signal_lock:
                    self.current_signal = hold
                time.sleep(0.005)
                continue

            if self.controller.finished_plan:
                self._status_message = "Goal reached — planning next segment..."
                self._plan_request.set()
                hold = self.controller.get_empty_sig()
                with self._signal_lock:
                    self.current_signal = hold
                time.sleep(0.005)
                continue

            signal = self.controller.get_sig(state)
            with self._signal_lock:
                self.current_signal = signal
            time.sleep(0.001)

    def _planner_loop(self):
        while not self._stop.is_set():
            if not self._plan_request.wait(timeout=0.1):
                continue
            if self._stop.is_set():
                break
            if self._planning or self._new_plan_ready.is_set():
                continue

            self._plan_request.clear()
            self._planning = True
            try:
                self._plan_next_segment()
            finally:
                self._planning = False

    def _plan_next_segment(self):
        with self._state_lock:
            state = self.latest_state

        if state is None:
            self._plan_request.set()
            return

        if self._segments_completed == 0:
            planning_instance = self.base_instance
        else:
            try:
                planning_instance = build_replan_instance(
                    self.base_instance,
                    self.config,
                    state,
                    self.rng,
                )
            except ValueError as exc:
                logging.warning("Could not snapshot robot state for replanning: %s", exc)
                self._plan_request.set()
                return

        self._status_message = (
            f"Planning segment {self._segments_completed + 1} "
            f"toward {planning_instance.goal_point}..."
        )
        print(self._status_message, flush=True)

        plan = self._solve_with_retries(planning_instance)
        if plan is None:
            self._on_planning_failed(planning_instance)
            return

        self._segment_zero_failures = 0
        solved_instance, parsed_plan = plan
        with self._plan_lock:
            self._pending_instance = solved_instance
            self._pending_plan = parsed_plan
        self._new_plan_ready.set()

    def _on_planning_failed(self, planning_instance: MonkeyBotProblemInstance):
        links = self.controller._cached_transition_links or []
        stats = analyze_reachability(planning_instance, links)
        if self._segments_completed == 0:
            self._segment_zero_failures += 1

        print(
            "Planning failed: no route found in the simplified attach/release/jump model.\n"
            f"  goal={planning_instance.goal_point}\n"
            f"  start_feet={planning_instance.init_feet}\n"
            f"  reachable_states={stats['reachable_states']}\n"
            f"  goal_states={stats['goal_states']}\n"
            f"  transition_links={stats['transition_links']}",
            flush=True,
        )

        if self._segments_completed == 0 and stats["goal_states"] == 0:
            print(
                "This instance goal cannot be reached with the simplified planner "
                "(try Random1 or JumpTest, or pick a closer goal).",
                flush=True,
            )

        if (
            self._segments_completed == 0
            and self._segment_zero_failures >= self._max_segment_zero_failures
        ):
            self._status_message = "Planning failed repeatedly — stopping demo."
            print(self._status_message, flush=True)
            self.stop()
            return

        self._status_message = "Planning failed — retrying..."
        print(self._status_message, flush=True)
        time.sleep(min(2.0, 0.5 * self._segment_zero_failures))
        self._plan_request.set()

    def _solve_with_retries(self, instance: MonkeyBotProblemInstance):
        current = instance
        links = self.controller._cached_transition_links
        for attempt in range(self.max_plan_attempts):
            self.config.instance = current
            self.controller.coordinator = self.config
            try:
                self.controller.create_plan()
                return current, list(self.controller.plan)
            except ValueError as exc:
                if self._segments_completed == 0:
                    print(f"  attempt {attempt + 1}/{self.max_plan_attempts}: {exc}", flush=True)
                    return None
                if self.latest_state is None:
                    return None
                current = build_replan_instance(
                    self.base_instance,
                    self.config,
                    self.latest_state,
                    self.rng,
                )
        return None

    def _apply_pending_plan(self):
        with self._plan_lock:
            instance = self._pending_instance
            plan = self._pending_plan
            self._pending_instance = None
            self._pending_plan = None

        if instance is None or plan is None:
            self._new_plan_ready.clear()
            return

        self.config.instance = instance
        self.controller.coordinator = self.config
        self.controller.reset_for_new_plan(plan)
        self.simulator.update_goal_point(self.config.screen_goal_point())

        self._segments_completed += 1
        self._new_plan_ready.clear()
        self._status_message = (
            f"Segment {self._segments_completed}: moving to {instance.goal_point}"
        )
        print(self._status_message, flush=True)

    def _update_window_caption(self):
        goal = self.config.instance.goal_point
        caption = (
            f"{self.base_instance.name} infinite demo | "
            f"segment {self._segments_completed} | goal {goal} | {self._status_message}"
        )
        try:
            pygame.display.set_caption(caption)
        except pygame.error:
            pass
