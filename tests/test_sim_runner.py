from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.sim_runner import SimRunner

instances_folder = "instances"
instance_name = "5StepsLadder"

def test_sim_runner():
    instance = load_instance("5StepsLadder", instances_folder)
    sim_runner = SimRunner(instance)
    sim_runner.execute_simulation(test=True)



if __name__ == "__main__":
    test_sim_runner()