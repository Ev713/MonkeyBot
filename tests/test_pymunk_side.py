from monkey_bot.monkey_bot_problem_instance import load_instance, list_instances
from monkey_bot.pymunk_simulator import MonkeyBotSimulator

instances_folder = "instances"

def test_pymunk_env_load():

    inst = load_instance("5StepsLadder", instances_folder)
    sim = MonkeyBotSimulator(inst)

    sim.init_controller()
    sim.start_simulation()
    sim.controller.create_plan()

    sim.run()

if __name__ == "__main__":
    test_pymunk_env_load()