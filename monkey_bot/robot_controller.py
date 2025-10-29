import itertools
import logging
import math
import os
import time
from curses.ascii import isdigit
from typing import Tuple
from unicodedata import digit

from pymunk import Vec2d
from unified_planning.shortcuts import SequentialSimulator

from monkey_bot import upf_solver
from monkey_bot.action import parse_action, Action
import monkey_bot.upf_solver
from monkey_bot.procedure import MoveCenter, ReleaseGrip, Mixed, Tracker, Grabber, AdjustLength, DynamicCatch, \
    MultiOptionalDynamicCatch
from monkey_bot.signals import ControlSignal, StateSignal
from monkey_bot.simulation_config import InstanceSimulationCoordinator
from monkey_bot.trajectory_finder import Launcher, GeometricLauncher
from monkey_bot.upf_solver import get_problem, solve_problem
from tests.test_upf_side import simulate

def grid_distance(p1:Tuple[int, int], p2:Tuple[int, int]):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

class Controller:
    def __init__(self, coordinator:InstanceSimulationCoordinator):
        self.finished_plan = False
        self.plan = []
        self.coordinator = coordinator

        self.t = 0
        self.actions_finished = 0

        self._dir_deltas = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}
        self.longest_action_time=0
        self._start_last_action_time=0
        self.procedures= None
        self.proc_id = 0

        self.launch_instructions = {}


    @property
    def _last_grid_center_pos(self):
        x, y = self.coordinator.instance.init_center
        for i in range(self.actions_finished):
            action = self.plan[i]
            name = action.name
            if "move_center" in name:
                delta_x, delta_y = self.get_center_grid_delta(name)
                x, y = x + delta_x, y + delta_y
            if "tran" in name:
                tran_name = action.args[0]
                _, _, _, _, _, c_x, c_y = tran_name.split('_')
                c_x = int(c_x)
                c_y = int(c_y)
                x, y = c_x, c_y
        return x, y

    def find_cliques(self):
        cliques = {}
        for i, gp in enumerate(self.coordinator.instance.gripping_points):
            cliques[i] = {gp}
        irreducible=False
        while not irreducible:
            irreducible = True
            for i in cliques:
                for j in range(i):
                    if j not in cliques:
                        continue
                    if all(math.hypot(gp1[0]-gp2[0], gp1[1]-gp2[1]) >= self.coordinator.instance.max_extension
                           for gp1 in cliques[i] for gp2 in cliques[j]):
                        continue
                    irreducible = False
                    cliques[j] = cliques[i]|cliques[j]
                    cliques.pop(i)
                    break
                if not irreducible:
                    break

        return cliques

    def get_current_action(self):
        return self.plan[self.actions_finished]

    def create_plan(self, plan_path=None):
        edge_transitions = self.get_transition_edges()
        for e in edge_transitions:
            assert e in self.launch_instructions
        logging.debug(f"Found {len(edge_transitions):} transition edges")
        problem = get_problem(self.coordinator.instance, edge_transitions)
        #simulate(problem, )
        start= time.perf_counter()
        logging.debug("Started solving the problem")
        plan = solve_problem(problem)
        logging.debug(f"Finished solving the problem. Took {time.perf_counter() - start} seconds")
        if plan is None:
            raise ValueError("Planning problem is unsolvable")
        if plan_path:
            self.save_plan(plan, plan_path)
        self.plan = [parse_action(a) for a in plan.actions]

    def save_plan(self, plan, plan_path=None):
        with open(os.path.join(plan_path), 'w') as f:
            for a in plan.actions:
                f.write(f"{a}\n")

    def load_plan(self, filepath):
        with open(filepath) as f:
            self.plan = [parse_action(a) for a in f]


    def get_center_grid_delta(self, action_name):
        chosen_dir = None
        for d in self._dir_deltas:
            if d in action_name:
                chosen_dir = d
                break
        return self._dir_deltas[chosen_dir]


    def get_empty_sig(self):
        return ControlSignal([0 for _ in range(self.coordinator.num_legs)],
                      [0 for _ in range(self.coordinator.num_legs)],
                      [0 for _ in range(self.coordinator.num_legs)])

    def get_sig(self, state_info:StateSignal):
        sig = self.get_empty_sig()
        if self.actions_finished == len(self.plan):
            return sig

        if not self.procedures:
            current_action = self.get_current_action()
            if current_action is None:
                self.finished_plan = True
                return sig
            if "move_center" in current_action.name:
                self.setup_move_center_procedure_sequence(current_action)
            elif "move_foot" in current_action.name:
                self.setup_move_foot_procedure_sequence(current_action)
            elif "tran" in current_action.name:
                self.setup_jump_procedure_sequence(current_action, state_info)
            elif "release" in current_action.name:
                self.setup_release_foot_procedure_sequence(current_action)
            else:
                raise Exception(f"Unknown action {current_action.name}")
            if self.procedures is not None:
                print(f"Starting new procedure {self.procedures[self.proc_id]}")
        p = self.procedures[self.proc_id]

        if p.is_finished(state_info):
            self.proc_id += 1
            if self.proc_id == len(self.procedures):
                self.finish_action()
                self.procedures = None
            if self.procedures is not None:
                print(f"Starting new procedure {self.procedures[self.proc_id]}")
        return p.adjust_signal(sig, state_info)

    def move_foot_is_redundant(self, state_info:StateSignal, action):
        goal_point = self.coordinator.gp_name_to_screen_point(action.args[0])
        limb_id = int(action.name.split('_')[-1])
        foot_pos = state_info.feet_pos[limb_id]
        return (goal_point - foot_pos).length < self.coordinator.epsilon and state_info.active_grips[limb_id]

    def finish_action(self):
        self._start_last_action_time = 0
        #print(f"Finished Action: {self.plan[self.actions_finished]}")
        self.actions_finished+=1
        if len(self.plan)>self.actions_finished:
            #pass
            print(f"Starting action {self.plan[self.actions_finished]}")
        else:
            self.finished_plan =True
            print(f"Goal achieved")

    def read_plan(self, file_path):
        self.plan = [parse_action(l) for l in open(file_path)]

    def create_or_read_plan(self, folder):
        edge_transitions = self.get_transition_edges()
        for e in edge_transitions:
            assert e in self.launch_instructions
        print(f"Found {len(edge_transitions)} transition edges")
        plan_path = f"{folder}/{self.coordinator.instance.name}.txt"

        if os.path.exists(plan_path):
            self.read_plan(plan_path)
            print(f"Loaded existing plan from {plan_path}")
        else:
            self.create_plan(plan_path=plan_path)
            print(f"Created and saved new plan to {plan_path}")

    def setup_move_center_procedure_sequence(self, current_action):
        next_grid_center_point = tuple(Vec2d(*self._last_grid_center_pos)
                                        +Vec2d(*self.get_center_grid_delta(current_action.name)))
        goal_point = Vec2d(*self.coordinator.grid_to_screen(*next_grid_center_point))
        self.procedures = [MoveCenter(goal_point, self.coordinator)]
        self.proc_id = 0


    def setup_move_foot_procedure_sequence(self, current_action:Action):
        goal_point = self.coordinator.gp_name_to_screen_point(current_action.args[0])
        limb_id = int(current_action.name.split('_')[-1])
        self.procedures = []

        self.procedures.append(ReleaseGrip(limb_id, self.coordinator))
        catcher = Mixed([Tracker(limb_id, goal_point, self.coordinator),
                        Grabber(limb_id, goal_point, self.coordinator)], self.coordinator)
        self.procedures.append(catcher)

        self.proc_id = 0

    def get_transition_edges(self):
        edge_transitions = []
        seen_configs = set()
        cliques = self.find_cliques()
        reverse_cliques = {gp:i for gp in self.coordinator.instance.gripping_points for i in cliques if gp in cliques[i]}
        for init1, init2, p1, p2, p3 in itertools.combinations(self.coordinator.instance.gripping_points, 5):

            if (init1 == p1 and init2 == p2 or ()) or init1 == init2 or p1 == p2 or p2 == p3 or p1==p3:
                continue

            max_ext = self.coordinator.instance.max_extension
            center = self.get_grid_arbitrary_center(p1, p2, p3)

            if center is None or grid_distance(init1, init2) > 2 * max_ext:
                continue

            if (grid_distance(center, init1) <= max_ext and
                    grid_distance(center, init2) <= max_ext):
                continue

            if all(reverse_cliques[p] == reverse_cliques[init1] for p in [init2, p1, p2, p3]):
                continue

            init1_screen = self.coordinator.grid_to_screen(*init1)
            init2_screen = self.coordinator.grid_to_screen(*init2)
            p1_screen = self.coordinator.grid_to_screen(*p1)
            p2_screen = self.coordinator.grid_to_screen(*p2)
            p3_screen = self.coordinator.grid_to_screen(*p3)

            reachable = frozenset(self.get_all_catchable_gripping_points(center))
            config_key = (init1, init2, reachable)
            if config_key in seen_configs:
                continue

            key = self.hash_jump_params(init1, init2, center)
            launch = self.check_jump_is_possible(init1_screen, init2_screen, p1_screen, p2_screen, p3_screen)
            if launch:
                seen_configs.add(config_key)
                self.launch_instructions[key]=launch
                edge_transitions.append((init1, init2, center))
        return edge_transitions

    def get_all_catchable_gripping_points(self, center_point):
        return [gp for gp in self.coordinator.instance.gripping_points if
                0 < math.hypot(center_point[0] - gp[0], center_point[1] - gp[1]) < self.coordinator.instance.max_extension]

    def hash_jump_params(self, from_1, from_2, center):
        return tuple((round(k[0]), round(k[1])) for k in [from_1, from_2,center])

    def get_grid_arbitrary_center(self, p1, p2, p3):
        min_x = min(p1[0], p2[0], p3[0])
        max_x = max(p1[0], p2[0], p3[0])
        min_y = min(p1[1], p2[1], p3[1])
        max_y = max(p1[1], p2[1], p3[1])
        points = [(x, y) for x, y in itertools.product(range(min_x, max_x + 1), range(min_y, max_y + 1))]
        best = None
        best_r = float("inf")
        for x, y in points:
            dists = [((x - a) ** 2 + (y - b) ** 2) ** 0.5 for a, b in (p1, p2, p3)]
            if all(0 < d <= self.coordinator.instance.max_extension for d in dists):
                r = max(dists)
                if r < best_r:
                    best, best_r = (x, y), r
        return best

    def check_jump_is_possible(self, from_1, from_2, p1, p2, p3):
        jump_validator = GeometricLauncher(
            self.coordinator.gravity,
            self.coordinator.dt,
            [p1, p2, p3],
            [from_1, from_2],
            self.coordinator.sim_config.screen_width,
            self.coordinator.sim_config.screen_height,
            max_extension=self.coordinator.max_extension,
            min_extension=self.coordinator.min_extension,
            max_take_off_speed=self.coordinator.max_jump_speed,
        )

        start_point, takeoff_vector = jump_validator.suggest_trajectory()

        if jump_validator.setup_achieves_goal(start_point, takeoff_vector):
            return start_point, takeoff_vector
        else:
            return None

    def setup_jump_procedure_sequence(self, current_action:Action, state_info:StateSignal):
        init1_x, init1_y, init2_x, init2_y, c_x, c_y = [int(x) for x in current_action.args[0].split('_') if x.isnumeric()]

        jumping_leg_1_pos = self.coordinator.grid_to_screen(init1_x, init1_y)
        jumping_leg_2_pos = self.coordinator.grid_to_screen(init2_x, init2_y)

        # Finding legs that are "close" - closer then cellsize/3 (3 is arbitrary)
        jumping_leg_1_id = [i for i in range(self.coordinator.num_legs) if
                            (state_info.feet_pos[i] - jumping_leg_1_pos).length<self.coordinator.cell_size()/3][0]
        jumping_leg_2_id = [i for i in range(self.coordinator.num_legs) if
                            (state_info.feet_pos[i] - jumping_leg_2_pos).length<self.coordinator.cell_size()/3][0]

        catch_points = self.get_all_catchable_gripping_points((c_x, c_y))[:3]

        screen_catch_points = [self.coordinator.grid_to_screen(*p) for p in catch_points]

        ignore_legs = [leg_id for leg_id in range(self.coordinator.num_legs)
                       if leg_id not in (jumping_leg_1_id, jumping_leg_2_id)]

        key = self.hash_jump_params((init1_x, init1_y), (init2_x, init2_y), (c_x, c_y))

        assert key in self.launch_instructions
        starting_point, jumping_vector = self.launch_instructions[key]
        self.procedures = []
        take_off_point = starting_point
        while (state_info.feet_pos[jumping_leg_1_id] - take_off_point).length < self.coordinator.max_extension \
                and (state_info.feet_pos[jumping_leg_2_id] - take_off_point).length < self.coordinator.max_extension:
            take_off_point += self.coordinator.dt * jumping_vector
        take_off_point -= self.coordinator.dt * jumping_vector*5

        releasers = []
        for i in ignore_legs:
            releasers.append(ReleaseGrip(i, self.coordinator))

        self.procedures.append(Mixed(releasers, self.coordinator))
        retractors = []
        for i in ignore_legs:
            retractors.append(AdjustLength(i, self.coordinator.min_extension, self.coordinator))
        self.procedures.append(Mixed(retractors, self.coordinator))

        on_position = MoveCenter(tuple(starting_point), self.coordinator, ignore_legs)
        self.procedures.append(on_position)

        launch = MoveCenter(take_off_point, self.coordinator, ignore_legs)
        launch.move_center_speed = jumping_vector.length
        self.procedures.append(launch)

        takeoff_releasers = [ReleaseGrip(i, self.coordinator) for i in (jumping_leg_1_id, jumping_leg_2_id)]
        self.procedures.append(Mixed(takeoff_releasers, self.coordinator))
        admissable_targets = [self.coordinator.grid_to_screen(*p) for p in self.get_all_catchable_gripping_points((c_x, c_y))]

        catcher = MultiOptionalDynamicCatch(admissable_targets, state_info, self.coordinator)
        self.procedures.append(catcher)

        move_center = MoveCenter(self.coordinator.grid_to_screen(c_x, c_y), self.coordinator,)
        self.procedures.append(move_center)

        self.proc_id = 0

    def setup_release_foot_procedure_sequence(self, current_action):
        limb_id = int(current_action.name.split('_')[-1])
        self.procedures = []

        releaser = ReleaseGrip(limb_id, self.coordinator)

        self.procedures.append(releaser)
        retractor = AdjustLength(limb_id, self.coordinator.min_extension, self.coordinator)
        self.procedures.append(retractor)
        self.proc_id = 0


class ManualController(Controller):
    def __init__(self, coordinator):
        Controller.__init__(self, coordinator)
        self.actions = None
        edge_transitions = self.get_transition_edges()
        for e in edge_transitions:
            assert e in self.launch_instructions
        logging.debug(f"Found {len(edge_transitions):} transition edges")
        self.problem = get_problem(self.coordinator.instance, edge_transitions)
        self.upf_simulator = SequentialSimulator(self.problem)
        self.state = self.upf_simulator.get_initial_state()
        self.t = 0
        self.parsed_actions = []
        self.actions_finished = None

    def check_possible_actions(self):
        self.actions = list(self.upf_simulator.get_applicable_actions(self.state))
        self.parsed_actions = [parse_action(str(a[0].name)+str(a[1])) for a in self.actions]

    def state_to_dict(self,):
        return {str(key): val for key, val in self.state._values.items()}

    def state_to_str(self):
        state_str = ''
        state_dict = self.state_to_dict()
        for key in state_dict:
            state_str += f'{key}: {state_dict[key]}\n'
        return state_str

    def print_possible_actions(self):
        self.check_possible_actions()
        if len(self.actions) == 0:
            print('No legal actions')
            return
        print('Actions: ')

        for i, a in enumerate(self.parsed_actions):
            print(f'{i}: {a.name}{tuple(a.args)}')

    def ask_action_from_user(self):
        choice = None
        while choice is None:
            try:
                choice = int(input('Choice: '))
            except:
                pass
            if choice == -1:
                return None
            if choice not in range(len(self.actions)):
                choice = None
                print('Invalid index. Try again.')
        return choice

    def check_is_solved(self):
        if self.upf_simulator.is_goal(self.state):
            print("Goal reached!")
            return True
        return False

    def get_current_action(self):
        self.print_possible_actions()
        choice = self.ask_action_from_user()
        if choice is None:
            return None
        action = self.actions[choice]
        parsed_action = self.parsed_actions[int(choice)]
        try:
            self.state = self.upf_simulator.apply(self.state, action[0], action[1])
        except Exception as e:
            raise Exception(f'Applying action resulted in: \n{e}')
        return parsed_action

    @property
    def _last_grid_center_pos(self):
        raise NotImplementedError

    def finish_action(self):
        return
