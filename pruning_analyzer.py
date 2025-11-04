import itertools
import pandas as pd
from unified_planning.plans import PlanKind
from unified_planning.shortcuts import OneshotPlanner
# --- Assuming necessary imports from your project ---
from monkey_bot.robot_controller import Controller
from monkey_bot.simulation_config import InstanceSimulationCoordinator, SimConfig, RobotConfig
from monkey_bot.upf_solver import get_problem, solve_problem
from monkey_bot.monkey_bot_problem_instance import load_instance
from pathlib import Path
import time
from typing import Tuple, Any, Dict


# Helper: Default simulation configuration (matching values found in test_sim_runner.py)
def get_default_sim_robot_configs(instance):
    sim_config = SimConfig(screen_height=1000, screen_width=1000, fps=120, gravity=0.25)
    robot_config = RobotConfig(
        epsilon=0.1, extension_speed=4, rotation_speed=4, move_center_speed=4,
        body_mass=5, body_radius=0.4, foot_mass=0.2, leg_mass=0.2,
        leg_spring_stiffness=500, leg_spring_damping=1.5, min_extension=0.1,
        max_takeoff_speed=20,
    )
    return sim_config, robot_config


def analyze_pruning_combinations(instance_name: str, instances_folder: str = "instances") -> Tuple[
    pd.DataFrame, pd.DataFrame]:
    """
    Analyzes the effect of the three jump transition prunings on the UPF planning process,
    dynamically collecting all metrics reported by the planner.
    """
    # 1. Setup Coordinator and Controller for the instance
    instance = load_instance(instance_name, Path(instances_folder))
    sim_config, robot_config = get_default_sim_robot_configs(instance)
    coordinator = InstanceSimulationCoordinator(instance, sim_config, robot_config)
    controller = Controller(coordinator)

    # 2^3 = 8 combinations (P1, P2, P3)
    pruning_combinations = list(itertools.product([False, True], repeat=3))
    results_list: list[Dict[str, Any]] = []
    pruning_times: Dict[str, Dict[str, Any]] = {}

    for p1, p2, p3 in pruning_combinations:
        pruning_name = f"P1={int(p1)}, P2={int(p2)}, P3={int(p3)}"
        print(f"--- Running {pruning_name} for instance: {instance_name} ---")

        # --- Stage 1: Transition Edge Generation (Pruning P1, P2, P3) ---
        pruning_start_time = time.perf_counter()

        # Call the internal function to get jump transitions
        viable_jumps = controller._get_transition_edges_internal(use_p1=p1, use_p2=p2, use_p3=p3)

        pruning_time = time.perf_counter() - pruning_start_time
        # Remove duplicates
        viable_jumps_unique = list(set(viable_jumps))
        num_transitions = len(viable_jumps_unique)

        pruning_times[pruning_name] = {"Transition Generation Time (s)": pruning_time,
                                       "Transitions Generated": num_transitions}

        # --- Stage 2: UPF Problem Solving ---
        problem = get_problem(instance, viable_jumps_unique)

        solving_start_time = time.perf_counter()
        result = solve_problem(problem)
        solving_time = time.perf_counter() - solving_start_time

        planner_metrics = result.metrics if result.metrics else {}
        plan = result.plan

        # Collect data
        plan_length = len(plan.actions) if plan and plan.kind == PlanKind.SEQUENTIAL_PLAN else 'N/A'

        # --- Dynamic Data Collection ---
        entry = {
            "Pruning Combination": pruning_name,
            "Transitions Generated": num_transitions,
            "Plan Length": plan_length,
            "Solving Time (s) (Manual)": solving_time,
            "Result Status": result.status.name,
        }

        # Dynamically add all key-value pairs from the planner's metrics dictionary
        # This captures all enhsp metrics regardless of their key names.
        entry.update(planner_metrics)

        results_list.append(entry)

    # Format output as a transposed DataFrame
    # Note: pandas will automatically handle the union of all keys across all runs
    df_metrics = pd.DataFrame(results_list).set_index("Pruning Combination").transpose()
    df_pruning = pd.DataFrame.from_dict(pruning_times, orient='index').transpose()

    # --- Explicitly print and save the results ---

    # 1. Metrics Table
    metrics_file = f"{instance_name}_pruning_metrics.csv"
    print(f"\n--- Metrics Table for '{instance_name}' (Saved to {metrics_file}) ---")
    print(df_metrics.to_markdown(numalign="left", stralign="left"))
    df_metrics.to_csv(metrics_file)

    # 2. Pruning/Time Table
    time_file = f"{instance_name}_transition_time.csv"
    print(f"\n--- Transition Generation Time Table (Saved to {time_file}) ---")
    print(df_pruning.to_markdown(numalign="left", stralign="left"))
    df_pruning.to_csv(time_file)

    return df_metrics, df_pruning
# Example Usage (Run this in your environment after making the file changes):
# from pruning_analyzer import analyze_pruning_combinations
# df_metrics, df_pruning = analyze_pruning_combinations("Random", "instances")
# print("--- Metrics Table ---")
# print(df_metrics)
# print("\n--- Pruning & Transition Time ---")
# print(df_pruning)