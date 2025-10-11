import time

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.robot_controller import Controller, TestController
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator


class SimRunner:
    def __init__(self, instance:MonkeyBotProblemInstance, sim_config:SimConfig):


        self.instance = instance
        self.inst_sim_coordiator = InstanceSimulationCoordinator(instance, sim_config)

        self.simulator=None
        self.controller=None
        self.tes=False

        self.start_controller(test=True)
        self.start_simulator()

    def start_simulator(self):
        self.simulator = MonkeyBotSimulator(self.inst_sim_coordiator.config)
        self.simulator.start_simulation( self.inst_sim_coordiator)

    def start_controller(self, test):
        if not test:
            self.controller = Controller(self.inst_sim_coordiator)
        else:
            self.controller = TestController(self.inst_sim_coordiator)


    def execute_simulation(self, test=False):
        self.start_simulator()
        self.start_controller(test=test)
        if not test:
            # self.controller.create_plan()
            self.controller.load_plan("plan_example.txt")
        while self.simulator.run:
            #input('Press any key for next frame')
            state = self.simulator.get_state()
            self.controller.update(state)
            sig = self.controller.get_sig()
            self.simulator.apply_signal(sig)
            self.simulator.step()
            if self.controller.finished_plan:
                self.simulator.writer.cose()
                break

