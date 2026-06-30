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
from monkey_bot.signals import StateSignal, empty_control_signal
from monkey_bot.sim_runner import SimRunner


class InfiniteDemoRunner(SimRunner):
    """
    Runs a climbing demo that never stops: execute a plan, reach the goal,
    pick a new nearby goal, replan, and continue.

    Single-threaded: each frame reads sim state, plans when needed, computes
    the control signal, and steps physics (same structure as SimRunner.run_step).
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

        self._stop = False
        self._segments_completed = 0
        self._segment_zero_failures = 0
        self._max_segment_zero_failures = 3
        self._pending_goal_screen = None
        self._caption_goal = normalize_point(instance.goal_point)
        self._status_message = "Starting..."

        self._record_actions = load_record()
        self._session_actions_completed = 0
        self._last_actions_finished = 0
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
        self.start_simulator()
        self._update_window_caption()
        self.simulator.pump_display()

        print("Infinite demo running — close the window or press Ctrl+C to stop.", flush=True)
        if self._record_actions:
            print(
                f"Demo action record to beat: {self._record_actions} actions in one run.",
                flush=True,
            )

        try:
            while self.simulator.run and not self._stop:
                self._run_frame()
        except KeyboardInterrupt:
            self.stop()
        finally:
            self.stop()

    def stop(self):
        self._stop = True
        if self.simulator is not None:
            self.simulator.run = False

    def _run_frame(self):
        if self._pending_goal_screen is not None:
            self.simulator.update_goal_point(self._pending_goal_screen, self.config)
            self._pending_goal_screen = None

        state = self.simulator.get_state()

        if self._should_plan_now():
            self._plan_next_segment(state)

        if not self.controller.plan:
            signal = self.controller.get_empty_sig()
        elif self.controller.finished_plan:
            self._status_message = "Goal reached."
            signal = self.controller.get_empty_sig()
        else:
            signal = self.controller.get_sig(state)
            self._track_action_completions()

        self.simulator.apply_signal(signal)
        self._update_window_caption()
        self.simulator.step()

        if not self.simulator.run and not self._stop:
            self._status_message = "Simulation stopped (physics unstable)."
            self.stop()

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
        if not self.controller.plan:
            return True
        return self.controller.finished_plan

    def _plan_next_segment(self, state: StateSignal):
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
                return

            self._queue_goal_marker_update(planning_instance)
            self._status_message = (
                f"Planning new path to {planning_instance.goal_point}..."
            )

        print(self._status_message, flush=True)
        self._update_window_caption()

        plan_result: list = [None]
        plan_error: list = [None]

        def run_planning():
            try:
                plan_result[0] = self._solve_with_retries(
                    planning_instance,
                    state,
                    allow_jumps=is_initial_plan,
                )
            except Exception as exc:
                plan_error[0] = exc

        worker = threading.Thread(target=run_planning, name="infinite-demo-plan", daemon=True)
        worker.start()
        while worker.is_alive():
            if not self.simulator.pump_display():
                self.stop()
                return
            self._update_window_caption()
            time.sleep(0.005)

        if plan_error[0] is not None:
            raise plan_error[0]

        plan = plan_result[0]
        if plan is None:
            self._on_planning_failed(planning_instance)
            return

        self._segment_zero_failures = 0
        solved_instance, parsed_plan = plan
        self._apply_plan(solved_instance, parsed_plan, previous_goal)

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

    def _queue_goal_marker_update(self, instance: MonkeyBotProblemInstance):
        goal = normalize_point(instance.goal_point)
        self._active_goal = goal
        self._caption_goal = goal
        self._pending_goal_screen = self.config.grid_to_screen(*goal)

    def _solve_with_retries(
        self,
        instance: MonkeyBotProblemInstance,
        state: StateSignal,
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
                    try:
                        current = pick_replan_instance(
                            self.base_instance,
                            self.config,
                            state,
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

    def _apply_plan(
        self,
        instance: MonkeyBotProblemInstance,
        plan: list,
        previous_goal: Optional[tuple],
    ):
        self.config.instance = instance
        self.controller.coordinator = self.config
        self.controller.reset_for_new_plan(plan)
        self._active_goal = normalize_point(instance.goal_point)
        self._caption_goal = self._active_goal
        self._last_actions_finished = 0

        self._segments_completed += 1
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
