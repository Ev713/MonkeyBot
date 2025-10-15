from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.sim_runner import SimRunner
from monkey_bot.simulation_config import SimConfig

INSTANCES_FOLDER = "instances"

sim_config = SimConfig(
        screen_height= 1000,
        screen_width= 800,
        epsilon= 5,
        fps=120,
        extension_speed= 150,
        rotation_speed= 1,
        move_center_speed= 100,
        body_mass= 5,
        foot_mass= 1,
        leg_mass= 1,
        leg_spring_stiffness= 10000,
        leg_spring_damping=100,
        min_extension=10
        )

def test_simple_climbing():
    # hyperparameter search space
    instance = load_instance("GPT1", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config)

    sim_runner.execute_simulation()

def test_jump_procedure_sequence():
    instance = load_instance("JumpTest", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config)
    sim_runner.execute_simulation()




if __name__ == "__main__":
    test_jump_procedure_sequence()
    #test_simple_climbing()
