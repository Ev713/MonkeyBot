import random
from pathlib import Path

from unified_planning.shortcuts import OneshotPlanner, SequentialSimulator

from monkey_bot import upf_solver
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.upf_solver import get_dummy_numeric_problem

instances_folder = "../instances"

def state_to_dict(state):
    return {str(key): val for key, val in state._values.items()}

def state_to_str(state):
    state_dict = state_to_dict(state)
    for key in state_dict:
        print(f'{key}: {state_dict[key]}')

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
    return upf_solver.get_problem(load_instance(f"{steps}StepsLadder", instances_folder))

def test_upf_solver():
    problem = get_dummy_numeric_problem()
    with OneshotPlanner(name="enhsp") as planner:
        result = planner.solve(problem)
        plan = result.plan
        assert plan is not None
        print("%s returned:" % planner.name)
        print(plan)

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
    test_ladder()