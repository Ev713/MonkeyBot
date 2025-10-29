import itertools
import math
from typing import Tuple

from pygame.examples.stars import move_stars
from unified_planning.shortcuts import *
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance

LEG_MIN_EXT = 0.25

def dist_prec(x1, y1, x2, y2, val):
    return LE(get_dist_squard_fluent(x1,y1,x2,y2), Times(val, val))

def get_dist_squard_fluent(x1,y1,x2,y2):
    return Plus(Times(x1-x2,x1-x2), Times(y1-y2,y1-y2))

def median_is_index_prec(x1, x2, x3, i):
    if i == 0:
        med_x = x1
        not_med_x_1 = x2
        not_med_x_2 = x3
    elif i == 1:
        med_x = x2
        not_med_x_1 = x1
        not_med_x_2 = x3
    else:
        med_x = x3
        not_med_x_1 = x1
        not_med_x_2 = x2

    x_is_med_option_1_prec = And(LE(med_x, not_med_x_1), LE(not_med_x_2, med_x))
    x_is_med_option_2_prec = And(LE(med_x, not_med_x_2), LE(not_med_x_1, med_x))
    return Or(x_is_med_option_1_prec, x_is_med_option_2_prec)

def leg_order_precondition(center_x, center_y, legs_x:List, legs_y:List):
    n = len(legs_x)
    if n < 3:
        return True  # can't cross with <3 legs

    vectors = []
    for i in range(n):
        vx = Minus(legs_x[i], center_x)
        vy = Minus(legs_y[i], center_y)
        vectors.append((vx, vy))

    cross_products = []
    for i in range(n):
        j = (i + 1) % n  # next leg (wraps around)
        vi = vectors[i]
        vj = vectors[j]
        cross = Minus(Times(vi[0],vj[1]),Times(vi[1], vj[0]))
        cross_products.append(cross)

    signs = [GT(c, 0) for c in cross_products]
    at_most_one = None
    for i, _ in enumerate(signs):
        all_except_i = None
        for j, s in enumerate(signs):
            if i == j:
                continue
            if all_except_i is None:
                all_except_i = s
            all_except_i = And(all_except_i, s)
        if at_most_one is None:
            at_most_one = all_except_i
        else:
            at_most_one = Or(at_most_one, all_except_i)
    return at_most_one


def get_problem(instance: MonkeyBotProblemInstance, viable_jumps=None):
    leg_extension = instance.max_extension
    grid_size_x = instance.grid_size_x
    grid_size_y = instance.grid_size_y
    detached_pos = (-instance.max_extension, -instance.max_extension)
    non_deterministic_pos = (-2*instance.max_extension, -2*instance.max_extension)

    problem = Problem()

    GrippingPoint = UserType("Gripping_point")
    gripping_point_x = Fluent("gripping_point_x", IntType(),p=GrippingPoint)
    gripping_point_y = Fluent("gripping_point_y", IntType(), p=GrippingPoint)

    center_x = Fluent("center_x", IntType(1, grid_size_x))
    center_y = Fluent("center_y", IntType(1, grid_size_y))
    problem.add_fluent(center_x)
    problem.add_fluent(center_y)

    foot_1_x = Fluent(f"foot_1_x", IntType())
    foot_1_y = Fluent(f"foot_1_y", IntType())
    foot_2_x = Fluent(f"foot_2_x", IntType())
    foot_2_y = Fluent(f"foot_2_y", IntType())
    foot_3_x = Fluent(f"foot_3_x", IntType())
    foot_3_y = Fluent(f"foot_3_y", IntType())
    feet_pos = ((foot_1_x, foot_1_y), (foot_2_x, foot_2_y), (foot_3_x, foot_3_y))

    is_detached =[]
    is_attached = []
    for foot_x, foot_y in feet_pos:
        is_detached.append(And(Equals(foot_x, detached_pos[0]), Equals(foot_y, detached_pos[1])))
        is_attached.append(And(GE(foot_x, 0), GE(foot_y, 0)))
    all_attached = And(And(is_attached[0], is_attached[1]) ,is_attached[2])

    for i in range(3):
        other_1, other_2 = [x for x in range(3) if x != i]

        move_foot = InstantaneousAction(f"move_foot_{i}", p_to=GrippingPoint)

        p_to = move_foot.parameter('p_to')
        to_x = gripping_point_x(p_to)
        to_y = gripping_point_y(p_to)

        move_foot.add_precondition(Not(And(Equals(center_x, to_x), Equals(center_y, to_y))))
        move_foot.add_precondition(dist_prec(center_x, center_y, to_x, to_y, leg_extension))

        for j in range(3):
            move_foot.add_precondition(Not(And(Equals(feet_pos[j][0], to_x), Equals(feet_pos[j][1], to_y))))

        move_foot.add_precondition(And(Not(is_detached[other_1]), Not(is_detached[other_2])))

        move_foot.add_effect(feet_pos[i][0], to_x)
        move_foot.add_effect(feet_pos[i][1], to_y)
        problem.add_action(move_foot)

        release_foot = InstantaneousAction(f'release_foot_{i}')
        release_foot.add_precondition(is_attached[other_1])
        release_foot.add_precondition(is_attached[other_2])
        release_foot.add_effect(feet_pos[i][0], detached_pos[0])
        release_foot.add_effect(feet_pos[i][1], detached_pos[1])
        problem.add_action(release_foot)


    for direc in ["up", "down", "left", "right"]:
        move_center = InstantaneousAction(f"move_center_{direc}")
        #move_center.add_precondition(
        #    leg_order_precondition(center_x, center_y, [foot_x for foot_x, _ in feet_pos], [foot_y for _, foot_y in feet_pos]))

        mod = {"up": (0, 1), "down": (0, -1), "left": (-1, 0), "right": (1, 0)}[direc]

        close_enough = []
        for foot_x, foot_y in feet_pos:
            close_enough.append(dist_prec(foot_x, foot_y, center_x + mod[0], center_y + mod[1], leg_extension))

        grip_prec = all_attached
        for i, (foot_x, foot_y) in enumerate(feet_pos):
            other_1, other_2 = [x for x in range(3) if x != i]
            grip_prec = Or(grip_prec, And(And(is_detached[i], is_attached[other_1]), is_attached[other_2]))
            move_center.add_precondition(Or(close_enough[i], is_detached[i]))
            move_center.add_precondition(
                (Not(And(Equals(center_x + mod[0], foot_x), Equals(center_y + mod[1], foot_y)))))
        move_center.add_precondition(grip_prec)

        if direc == "up":
            move_center.add_precondition(LT(center_y, grid_size_y))
            move_center.add_effect(center_y, center_y+1)
        if direc == "down":
            move_center.add_precondition(GT(center_y, 1))
            move_center.add_effect(center_y, center_y - 1)
        if direc == "left":
            move_center.add_precondition(GT(center_x, 1))
            move_center.add_effect(center_x, center_x - 1)
        if direc == "right":
            move_center.add_precondition(LT(center_x, grid_size_x))
            move_center.add_effect(center_x, center_x + 1)
        problem.add_action(move_center)

    gripping_points = {}
    for i, (gp_x, gp_y) in enumerate(instance.gripping_points):
        gp = Object(f"GP_{gp_x}_{gp_y}", GrippingPoint)
        gripping_points[(gp_x, gp_y)] = gp

        problem.add_object(gp)
        problem.set_initial_value(gripping_point_x(gp), gp_x)
        problem.set_initial_value(gripping_point_y(gp), gp_y)

    for i, (init_foot_x, init_foot_y) in enumerate(instance.init_feet):
        foot_x, foot_y = feet_pos[i]
        problem.set_initial_value(foot_x, init_foot_x)
        problem.set_initial_value(foot_y, init_foot_y)


    init_center_x, init_center_y = instance.init_center
    problem.set_initial_value(center_x, init_center_x)
    problem.set_initial_value(center_y, init_center_y)

    goal_x, goal_y = instance.goal_point
    problem.add_goal(Equals(center_x, goal_x))
    problem.add_goal(Equals(center_y, goal_y))

    if viable_jumps is not None:

        TransitionLink = UserType("TransitionLink")

        jumping_leg_1_x = Fluent("jumping_leg_1_x", IntType(), tran=TransitionLink)
        jumping_leg_2_x = Fluent("jumping_leg_2_x", IntType(), tran=TransitionLink)
        jumping_leg_1_y = Fluent("jumping_leg_1_y", IntType(), tran=TransitionLink)
        jumping_leg_2_y = Fluent("jumping_leg_2_y", IntType(), tran=TransitionLink)

        jumping_legs = [(jumping_leg_1_x, jumping_leg_1_y), (jumping_leg_2_x, jumping_leg_2_y)]

        new_center_x = Fluent("new_center_x", IntType(), tran=TransitionLink)
        new_center_y = Fluent("new_center_y", IntType(), tran=TransitionLink)
        problem.add_fluent(jumping_leg_1_x)
        problem.add_fluent(jumping_leg_2_x)
        problem.add_fluent(jumping_leg_1_y)
        problem.add_fluent(jumping_leg_2_y)

        problem.add_fluent(new_center_x)
        problem.add_fluent(new_center_y)

        use_tran = InstantaneousAction("use_tran", tran=TransitionLink)
        tran = use_tran.parameter("tran")
        foot_1_x, foot_1_y = feet_pos[0]
        foot_2_x, foot_2_y = feet_pos[1]
        foot_3_x, foot_3_y = feet_pos[2]

        foot_at_init = {}
        for i, j in itertools.product([0, 1, 2], [0, 1]):
            foot_x = feet_pos[i][0]
            foot_y = feet_pos[i][1]
            init_x = jumping_legs[j][0](tran)
            init_y = jumping_legs[j][1](tran)

            foot_at_init[i, j] = Equals(foot_x, init_x), Equals(foot_y, init_y)
        feet_at_init = None
        for i, j in itertools.combinations(range(3), 2):
            if j == i:
                continue
            new_case = And(foot_at_init[i, 0], foot_at_init[j, 1])
            if feet_at_init is None:
                feet_at_init = new_case
            else:
                feet_at_init = Or(feet_at_init, new_case)

        use_tran.add_precondition(feet_at_init)
        use_tran.add_effect(center_x, new_center_x(tran))
        use_tran.add_effect(center_y, new_center_y(tran))

        use_tran.add_effect(foot_1_x, non_deterministic_pos[0])
        use_tran.add_effect(foot_2_x, non_deterministic_pos[0])
        use_tran.add_effect(foot_3_x, non_deterministic_pos[0])
        use_tran.add_effect(foot_1_y, non_deterministic_pos[1])
        use_tran.add_effect(foot_2_y, non_deterministic_pos[1])
        use_tran.add_effect(foot_3_y, non_deterministic_pos[1])
        problem.add_action(use_tran)
        for (init1_x, init1_y),(init2_x, init2_y), (c_x, c_y) in viable_jumps:

            tran = Object(f"tran_{init1_x}_{init1_y}_{init2_x}_{init2_y}_{c_x}_{c_y}", TransitionLink)
            problem.add_object(tran)

            problem.set_initial_value(jumping_leg_1_x(tran), init1_x)
            problem.set_initial_value(jumping_leg_2_x(tran), init2_x)
            problem.set_initial_value(jumping_leg_1_y(tran), init1_y)
            problem.set_initial_value(jumping_leg_2_y(tran), init2_y)

            problem.set_initial_value(new_center_x(tran), c_x)
            problem.set_initial_value(new_center_y(tran), c_y)

    return problem

def solve_problem(problem):
    with OneshotPlanner(name='enhsp') as planner:
        result = planner.solve(problem)
        plan = result.plan
        return plan

def solve_instance(instance: MonkeyBotProblemInstance):
    return solve_problem(get_problem(instance))




