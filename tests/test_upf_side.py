import random
from pathlib import Path

from unified_planning.shortcuts import OneshotPlanner, SequentialSimulator

from monkey_bot import upf_solver
from monkey_bot.monkey_bot_problem_instance import load_instance, increase_scale
from monkey_bot.upf_solver import get_problem, solve_problem

instances_folder = "../instances"

def state_to_dict(state):
    return {str(key): val for key, val in state._values.items()}

def state_to_str(state):
    state_str = ''
    state_dict = state_to_dict(state)
    for key in state_dict:
        state_str+=f'{key}: {state_dict[key]}\n'
    return state_str

def simulate(problem, print_state=False, random_walk=False):
    with SequentialSimulator(problem) as simulator:
        state = simulator.get_initial_state()
        if print_state:
            print(state_to_str(state))
        t = 0
        print(f't = {t}\nCalculating actions...')
        actions = [a for a in simulator.get_applicable_actions(state)]
        if len(actions) == 0:
            print('No legal actions')
            return
        print('Actions: ')
        for i, a in enumerate(actions):
            print(f'{i}: {a[0].name}{a[1]}')
        while True:
            try:
                while True:
                    if random_walk:
                        choice = random.choice(range(len(actions)))
                    else:
                        try:
                            choice = int(input('Choice: '))
                        except:
                            continue
                    if choice not in range(len(actions)):
                        print('Invalid index. Try again.')
                        continue
                    action = actions[int(choice)]
                    state = simulator.apply(state, action[0], action[1])
                    break
            except Exception as e:
                print(f'Applying action resulted in: \n{e}')
                return
            print(f'Action {action[0].name, action[1]} applied.')
            if print_state:
                print(state_to_str(state))

            if simulator.is_goal(state):
                print("Goal reached!")
                return

            t += 1
            actions = [a for a in simulator.get_applicable_actions(state)]
            if len(actions) == 0:
                print('No legal actions')
                return
            print(f't = {t}\nActions:')
            for i, a in enumerate(actions):
                print(str(i) + '.', a[0].name, a[1])

def get_ladder_prob(steps=5):
    return get_problem(load_instance(f"{steps}StepsLadder", instances_folder))

def get_jump_test_prob():
    return get_problem(load_instance(f"JumpTest", instances_folder))


def test_ladder():
    problem = get_ladder_prob()
    assert problem is not None
    with OneshotPlanner(name='enhsp') as planner:
        result = planner.solve(problem)
        plan = result.plan
        assert plan is not None
        print("%s returned:" % planner.name)
        print(plan)

if __name__ == "__main__":
    instance = load_instance(f"Random", instances_folder)
    instance = increase_scale(instance, 2)
    viable_jumps = [((8, 10), (10, 8), (12, 16)), ((8, 10), (10, 8), (18, 15)), ((8, 10), (10, 8), (16, 16)), ((8, 10), (10, 8), (17, 16)), ((8, 10), (10, 8), (19, 16)), ((8, 10), (10, 8), (19, 15)), ((8, 10), (10, 8), (12, 17)), ((8, 10), (10, 8), (12, 18)), ((8, 10), (10, 8), (19, 18)), ((8, 10), (10, 8), (18, 16)), ((8, 10), (10, 8), (20, 18)), ((8, 10), (10, 8), (12, 19)), ((8, 10), (10, 8), (20, 19)), ((10, 8), (12, 8), (10, 17)), ((10, 10), (12, 10), (10, 17))]

    problem = get_problem(instance, viable_jumps)
    simulate(problem)
    with OneshotPlanner(name='enhsp') as planner:
        result = planner.solve(problem)
        print(result)
        plan = result.plan
        assert plan is not None
        print("%s returned:" % planner.name)
        print(plan)