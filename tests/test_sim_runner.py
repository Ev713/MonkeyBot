import itertools
import os
import random
from itertools import combinations

from pymunk import Vec2d

from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.sim_runner import SimRunner
from monkey_bot.simulation_config import SimConfig

INSTANCES_FOLDER = "instances"
save_simulation=True
sim_config = SimConfig(
        screen_height= 1000,
        screen_width= 800,
        epsilon= 5,
        fps=120,
        extension_speed= 200,
        rotation_speed= 2,
        move_center_speed= 100,
        body_mass= 5,
        foot_mass= 1,
        leg_mass= 1,
        leg_spring_stiffness= 15000,
        leg_spring_damping=100,
        min_extension=10
        )

def test_simple_climbing():
    # hyperparameter search space
    instance = load_instance("GPT1", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config)

    sim_runner.execute_simulation(save=save_simulation)

def test_jump_procedure_sequence():
    instance = load_instance("JumpTest", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config)
    sim_runner.execute_simulation(save=save_simulation)

def test_trajectory_finder():
    instance = load_instance("JumpTest", INSTANCES_FOLDER)
    simulations_run = 0
    while simulations_run < 10:
        init1, init2, init3, p1, p2, p3 = random.choice(list(itertools.product(instance.gripping_points, instance.gripping_points,
                                                  instance.gripping_points, instance.gripping_points,
                                                  instance.gripping_points, instance.gripping_points)))

        init1, init2, init3, p1, p2, p3  = [1, 1], [3, 1], [2, 1], [1,4], [3,4], [3,5]
        center = 2,2#Vec2d(*init1)+Vec2d(0, 1)
        invalid = False
        if any([(center - Vec2d(*p)).length>instance.leg_extension for p in [init1, init2, init3]]):
            continue
        points = init1, init2, init3, p1, p2, p3
        if any([(Vec2d(*x) -Vec2d(*y)).length>2*instance.leg_extension for  x, y in combinations([p1, p2, p3], 2)]):
            invalid = True
        for i in range(5):
            if invalid:
                break
            for j in range(5):
                if i != j and points[i] == points[j]:
                    invalid = True
                    break
        if invalid:
            continue

        simulations_run += 1
        instance.init_feet = [init1, init2, init3]
        instance.init_center = tuple(center)
        instance.name = f"test{init1[0]}_{init1[1]}_{init2[0]}_{init3[1]}_{p1[0]}_{p1[1]}_{p2[0]}_{p2[1]}_{p3[0]}_{p3[1]}"
        with open(os.path.join(f'plans/{instance.name}.txt'), 'w') as f:
            f.write(f"jump(0, 1, G_{p1[0]}_{p1[1]}, G_{p2[0]}_{p2[1]}, G_{p3[0]}_{p3[1]})\n")
        sim_runner = SimRunner(instance, sim_config=sim_config)
        sim_runner.execute_simulation(save=False)



if __name__ == "__main__":
    #test_jump_procedure_sequence()
    #test_simple_climbing()
    test_trajectory_finder()
