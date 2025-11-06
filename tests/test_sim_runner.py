import itertools
import os
import random
from itertools import combinations

from pymunk import Vec2d
from pruning_analyzer import analyze_pruning_combinations
from monkey_bot.monkey_bot_problem_instance import load_instance, increase_scale
from monkey_bot.sim_runner import SimRunner
from monkey_bot.simulation_config import SimConfig, RobotConfig

INSTANCES_FOLDER = "instances"
save_simulation=True
sim_config = SimConfig(
        screen_height= 1000,
        screen_width= 1000,
        fps=120,
        gravity=0.5,
        cell_size=50
        )

robot_config = RobotConfig(
    epsilon=0.1,
    extension_speed=4,
    rotation_speed=4,
    move_center_speed=4,
    body_mass=5,
    body_radius=0.4,
    foot_mass=0.2,
    leg_mass=0.2,
    leg_spring_stiffness=500,
    leg_spring_damping=1.5,
    min_extension=0.1,
    max_takeoff_speed=10,
    prune_short_jumps=True,
    prune_in_clique_jumps=True,
    prune_similar_jumps=True,
)

def test_simple_climbing():
    # hyperparameter search space
    instance = load_instance("GPT1", INSTANCES_FOLDER)

    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)

    sim_runner.execute_simulation(save=save_simulation)

def test_jump_procedure_sequence():
    instance = load_instance("JumpTest", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    sim_runner.execute_manual_simulation(save=save_simulation)

def test_angle_adjuster():
    instance = load_instance("TestAngle", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    sim_runner.log_rotation_motors = [0]
    sim_runner.execute_simulation(save=False)


def test_complex_actions_problem(manual=False):
    instance = load_instance("Random1", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    if manual:
        sim_runner.execute_manual_simulation(save=True)
    else:
        sim_runner.execute_simulation(save=True)


if __name__ == "__main__":
    #test_angle_adjuster()
    #test_jump_procedure_sequence()
    #test_simple_climbing()
    test_complex_actions_problem(manual=False)
    #
#    df_metrics, df_pruning = analyze_pruning_combinations("Mixed", "instances")
#    print("--- Metrics Table ---")
#    print(df_metrics)
#    print("\n--- Pruning & Transition Time ---")
#    print(df_pruning)
