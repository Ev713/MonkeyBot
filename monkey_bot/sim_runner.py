import time
from itertools import combinations

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.robot_controller import Controller, ManualController
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator, RobotConfig


class SimRunner:
    def __init__(self, instance:MonkeyBotProblemInstance, sim_config:SimConfig, robot_config:RobotConfig):

        self.instance = instance
        self.inst_sim_coordiator = InstanceSimulationCoordinator(instance, sim_config, robot_config)

        self.simulator=None
        self.controller=None
        self.tes=False
        self.log_rotation_motors = []

    def start_simulator(self, save_simulation=True):
        self.simulator = MonkeyBotSimulator(self.inst_sim_coordiator.sim_config)
        self.simulator.start_simulation(self.inst_sim_coordiator)
        for i in self.log_rotation_motors:
            self.log_rotation_motor(i)

    def start_controller(self, manual=False):
        if manual:
            self.controller = ManualController(self.inst_sim_coordiator)
        else:
            self.controller = Controller(self.inst_sim_coordiator)

    def run_step(self):
        state = self.simulator.get_state()
        sig = self.controller.get_sig(state)
        self.simulator.apply_signal(sig)
        self.simulator.step()
        if self.controller.finished_plan:
            self.simulator.run = False
            if self.simulator.writer is not None:
                self.simulator.writer.close()
            return

    def log_rotation_motor(self, i):
        self.simulator.rotation_motors[i].enable_log()

    def execute_manual_simulation(self, ):
        self.start_controller(manual=True)
        self.start_simulator()
        while self.simulator.run:
            self.run_step()

    def execute_simulation(self, plans_folder="plans", save=False):
        self.start_controller()
        self.controller.create_or_read_plan(folder=plans_folder)
        self.start_simulator()
        if save:
            self.simulator.initiate_saver()
        while self.simulator.run:
            self.run_step()

