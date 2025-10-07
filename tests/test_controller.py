from monkey_bot import robot_controller
from monkey_bot.monkey_bot_problem_instance import load_instance, list_instances
from monkey_bot.pymunk_simulator import MonkeyBotSimulator

instances_folder = "instances"


def test_controller():
    controller = robot_controller.Controller(load_instance("5StepsLadder", instances_folder))
    controller.create_plan()
    print(controller.get_signals())

if __name__ == "__main__":
    test_controller()