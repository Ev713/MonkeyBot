import itertools
import os
import random
from itertools import combinations

from pymunk import Vec2d

from monkey_bot.monkey_bot_problem_instance import load_instance, increase_scale
from monkey_bot.sim_runner import SimRunner
from monkey_bot.simulation_config import SimConfig, RobotConfig

INSTANCES_FOLDER = "instances"
save_simulation=True
sim_config = SimConfig(
        screen_height= 1000,
        screen_width= 1000,
        fps=120,
        gravity=0.25
        )

robot_config = RobotConfig(
    epsilon=0.1,
    extension_speed=1,
    rotation_speed=1,
    move_center_speed=1,
    body_mass=5,
    body_radius=0.5,
    foot_mass=0.2,
    leg_mass=0.2,
    leg_spring_stiffness=400,
    leg_spring_damping=1.5,
    min_extension=0.4,
    max_takeoff_speed=20,
    #max_extension = 2
)

def test_simple_climbing():
    # hyperparameter search space
    instance = load_instance("GPT1", INSTANCES_FOLDER)

    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)

    sim_runner.execute_simulation(save=save_simulation)

def test_jump_procedure_sequence():
    instance = load_instance("JumpTest", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    sim_runner.execute_simulation(save=save_simulation)

def test_complex_actions_problem():
    instance = load_instance("Mixed", INSTANCES_FOLDER)
    increase_scale(instance, 2)
    instance.max_extension = 3
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    sim_runner.log_rotation_motors = [0]
    sim_runner.execute_simulation(save=True)

if __name__ == "__main__":
    #test_jump_procedure_sequence()
    #test_simple_climbing()
    test_complex_actions_problem()
