import time

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.robot_controller import Controller
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator


class SimRunner:
    def __init__(self, instance:MonkeyBotProblemInstance, sim_config:SimConfig):


        self.instance = instance
        self.inst_sim_coordiator = InstanceSimulationCoordinator(instance, sim_config)

        self.simulator=None
        self.controller=None
        self.tes=False

        self.start_controller()
        self.start_simulator()

    def start_simulator(self, save_simulation=True):
        self.simulator = MonkeyBotSimulator(self.inst_sim_coordiator.config)
        self.simulator.start_simulation( self.inst_sim_coordiator, save_simulation=save_simulation)


    def start_controller(self):
        self.controller = Controller(self.inst_sim_coordiator)



    def execute_simulation(self, plans_folder="plans", save=False):
        self.controller.create_or_read_plan(folder=plans_folder)
        if save:
            self.simulator.initiate_saver()
        while self.simulator.run:
            #input('Press any key for next frame')
            state = self.simulator.get_state()
            sig = self.controller.get_sig(state)
            self.simulator.apply_signal(sig)
            self.simulator.step()
            if self.controller.finished_plan:
                self.simulator.run=False
                if self.simulator.writer is not None:
                    self.simulator.writer.close()
                break

