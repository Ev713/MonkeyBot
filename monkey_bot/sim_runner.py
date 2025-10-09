from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.robot_controller import Controller, TestController
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator


class SimRunner:
    def __init__(self, instance:MonkeyBotProblemInstance):
        sim_config = SimConfig(screen_height=1000,
                               screen_width=800,
                               epsilon=5,
                               extension_speed=50,
                               twist_speed=1,
                               move_center_speed=50,
                               fps=60)

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
        args = [self.instance, self.epsilon, self.extension_speed,
                                     self.twist_speed, self.move_center_speed, self.screen_width, self.screen_height]
        if not test:
            self.controller = Controller(*args)
        else:
            self.controller = TestController(*args)


    def grid_to_screen(self, x, y):
        """
        Convert grid coordinates (x, y) to screen coordinates (px, py).

        The grid is centered on the screen and scaled so that there is
        at least one grid cell margin on all sides.
        """
        cell_size = self._cell_size()

        center_x, center_y = self.screen_width / 2, self.screen_height / 2
        screen_x = center_x + cell_size * (x - self.grid_width/2)
        screen_y = center_y - cell_size * (y - self.grid_height / 2)

        return screen_x, screen_y

    def execute_simulation(self, test=False):



        self.start_simulator()
        self.start_controller(test=test)
        if not test:
            self.controller.create_plan()
        while self.simulator.run:
            input('Press any key for next frame')
            state = self.simulator.get_state()
            self.controller.update(state)
            sig = self.controller.get_sig()
            self.simulator.apply_signal(sig)
            self.simulator.step()

