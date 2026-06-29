import logging
import random
import threading
import time
from typing import Optional

import pygame

from monkey_bot.config import RobotConfig, SimConfig
from monkey_bot.coords import normalize_point
from monkey_bot.demo_progress import load_record, try_commit_record
from monkey_bot.goal_generator import pick_replan_instance
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
        self._pending_previous_goal: Optional[tuple] = None
        self._goal_update_lock = threading.Lock()
        self._pending_goal_screen = None
        self._caption_goal = normalize_point(instance.goal_point)
        self._status_message = "Starting..."

        self._record_actions = load_record()
        self._session_actions_completed = 0
        self._last_actions_finished = 0

        num_legs = self.config.num_legs
        self.current_signal = empty_control_signal(num_legs)
        self.latest_state = None
        self._active_goal = normalize_point(instance.goal_point)

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
        if self._record_actions:
            print(
                f"Demo action record to beat: {self._record_actions} actions in one run.",
                flush=True,
            )

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
            with self._goal_update_lock:
                pending_goal = self._pending_goal_screen
                self._pending_goal_screen = None
            if pending_goal is not None:
                self.simulator.update_goal_point(pending_goal, self.config)

            with self._signal_lock:
                signal = self.current_signal

            self.simulator.apply_signal(signal)
            self._update_window_caption()
            self.simulator.step()

            if not self.simulator.run and not self._stop.is_set():
                self._status_message = "Simulation stopped (physics unstable)."
                self.stop()
                break

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
                self._status_message = "Goal reached."
                self._plan_request.set()
                hold = self.controller.get_empty_sig()
                with self._signal_lock:
                    self.current_signal = hold
                time.sleep(0.005)
                continue

            signal = self.controller.get_sig(state)
            self._track_action_completions()
            with self._signal_lock:
                self.current_signal = signal
            time.sleep(0.001)

    def _track_action_completions(self):
        finished = self.controller.actions_finished
        if finished <= self._last_actions_finished:
            return

        delta = finished - self._last_actions_finished
        self._last_actions_finished = finished
        self._session_actions_completed += delta
        print(
            f"Actions completed this run: {self._session_actions_completed} "
            f"(record: {self._record_actions})",
            flush=True,
        )
        if self._session_actions_completed > self._record_actions:
            if try_commit_record(self._session_actions_completed):
                self._record_actions = self._session_actions_completed

    def _should_plan_now(self) -> bool:
        if self._new_plan_ready.is_set():
            return False
        if not self.controller.plan:
            return True
        return self.controller.finished_plan

    def _planner_loop(self):
        while not self._stop.is_set():
            if not self._plan_request.wait(timeout=0.1):
                continue
            if self._stop.is_set():
                break
            if self._planning:
                continue
            if not self._should_plan_now():
                self._plan_request.clear()
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

        previous_goal = self._active_goal
        is_initial_plan = not self.controller.plan and self._segments_completed == 0

        if is_initial_plan:
            planning_instance = self.base_instance
            self._status_message = (
                f"Planning path to {planning_instance.goal_point}..."
            )
        else:
            try:
                planning_instance = pick_replan_instance(
                    self.base_instance,
                    self.config,
                    state,
                    self.rng,
                    previous_goal=previous_goal,
                )
            except ValueError as exc:
                logging.warning("Could not pick a new goal for replanning: %s", exc)
                self._plan_request.set()
                return

            self._queue_goal_marker_update(planning_instance)
            self._status_message = (
                f"Planning new path to {planning_instance.goal_point}..."
            )

        print(self._status_message, flush=True)

        plan = self._solve_with_retries(
            planning_instance,
            allow_jumps=is_initial_plan,
        )
        if plan is None:
            self._on_planning_failed(planning_instance)
            return

        self._segment_zero_failures = 0
        solved_instance, parsed_plan = plan
        with self._plan_lock:
            self._pending_instance = solved_instance
            self._pending_plan = parsed_plan
            self._pending_previous_goal = previous_goal
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

    def _queue_goal_marker_update(self, instance: MonkeyBotProblemInstance):
        goal = normalize_point(instance.goal_point)
        self._active_goal = goal
        self._caption_goal = goal
        screen_goal = self.config.grid_to_screen(*goal)
        with self._goal_update_lock:
            self._pending_goal_screen = screen_goal

    def _solve_with_retries(
        self,
        instance: MonkeyBotProblemInstance,
        *,
        allow_jumps: bool = True,
    ):
        current = instance
        saved_instance = self.config.instance
        try:
            for attempt in range(self.max_plan_attempts):
                self.config.instance = current
                self.controller.coordinator = self.config
                try:
                    self.controller.create_plan(allow_jumps=allow_jumps)
                    return current, list(self.controller.plan)
                except ValueError as exc:
                    if self._segments_completed == 0:
                        print(
                            f"  attempt {attempt + 1}/{self.max_plan_attempts}: {exc}",
                            flush=True,
                        )
                        return None
                    if self.latest_state is None:
                        return None
                    try:
                        current = pick_replan_instance(
                            self.base_instance,
                            self.config,
                            self.latest_state,
                            self.rng,
                            previous_goal=self._active_goal,
                        )
                    except ValueError:
                        return None
                    self._queue_goal_marker_update(current)
                    print(
                        f"Planning new path to {current.goal_point}... "
                        f"(retry {attempt + 2}/{self.max_plan_attempts})",
                        flush=True,
                    )
            return None
        finally:
            self.config.instance = saved_instance
            self.controller.coordinator = self.config

    def _apply_pending_plan(self):
        with self._plan_lock:
            instance = self._pending_instance
            plan = self._pending_plan
            previous_goal = self._pending_previous_goal
            self._pending_instance = None
            self._pending_plan = None
            self._pending_previous_goal = None

        if instance is None or plan is None:
            self._new_plan_ready.clear()
            return

        self.config.instance = instance
        self.controller.coordinator = self.config
        self.controller.reset_for_new_plan(plan)
        self._active_goal = normalize_point(instance.goal_point)
        self._caption_goal = self._active_goal
        self._last_actions_finished = 0
        self._plan_request.clear()

        self._segments_completed += 1
        self._new_plan_ready.clear()
        self._status_message = (
            f"Segment {self._segments_completed}: moving to {instance.goal_point}"
        )
        grid_goal = instance.goal_point
        screen_goal = self.config.screen_goal_point()
        print(
            f"New plan ready: {grid_goal} (was {previous_goal}, {len(plan)} actions) "
            f"| screen ({screen_goal.x:.0f}, {screen_goal.y:.0f})",
            flush=True,
        )
        print(self._status_message, flush=True)

    def _update_window_caption(self):
        goal = self._caption_goal
        caption = (
            f"{self.base_instance.name} infinite demo | "
            f"segment {self._segments_completed} | "
            f"actions {self._session_actions_completed}/{self._record_actions} | "
            f"goal {goal} | {self._status_message}"
        )
        try:
            pygame.display.set_caption(caption)
        except pygame.error:
            pass
