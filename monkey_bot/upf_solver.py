import itertools
from typing import Tuple

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


def get_problem(instance: MonkeyBotProblemInstance, jump_actions_allowed=None):
    if jump_actions_allowed is None:
        jump_actions_allowed = []
    leg_num = len(instance.init_feet)
    leg_extension = instance.leg_extension
    grid_size_x = instance.grid_size_x
    grid_size_y = instance.grid_size_y

    problem = Problem()

    GrippingPoint = UserType("Gripping_point")
    gripping_point_x = Fluent("gripping_point_x", IntType(),p=GrippingPoint)
    gripping_point_y = Fluent("gripping_point_y", IntType(), p=GrippingPoint)
    is_free = Fluent("is_free", BoolType(), p=GrippingPoint)
    problem.add_fluent(is_free, default_initial_value=True)

    center_x = Fluent("center_x", IntType(1, grid_size_x))
    center_y = Fluent("center_y", IntType(1, grid_size_y))
    problem.add_fluent(center_x)
    problem.add_fluent(center_y)

    feet = []
    for i in range(leg_num):
        foot_at = Fluent(f"foot_{i}_at", BoolType(), p=GrippingPoint)
        foot_x = Fluent(f"foot_{i}_x", IntType())
        foot_y = Fluent(f"foot_{i}_y", IntType())
        feet.append((foot_at, foot_x, foot_y))
        problem.add_fluent(foot_at, default_initial_value=False)
        move_foot = InstantaneousAction(f"move_foot_{i}", p_from=GrippingPoint, p_to=GrippingPoint)
        p_from = move_foot.parameter('p_from')
        p_to = move_foot.parameter('p_to')
        move_foot.add_precondition(foot_at(p_from))
        move_foot.add_precondition(is_free(p_to))
        move_foot.add_precondition(
            dist_prec(center_x, center_y, gripping_point_x(p_to), gripping_point_y(p_to), leg_extension))
        new_feet_x = [foot_x for _, foot_x, _ in feet]
        new_feet_x[i] = gripping_point_x(p_to)
        new_feet_y = [foot_y for _, foot_y, _ in feet]
        new_feet_y[i] = gripping_point_y(p_to)
        move_foot.add_precondition(leg_order_precondition(center_x, center_y, new_feet_x, new_feet_y))

        move_foot.add_effect(foot_at(p_from), False)
        move_foot.add_effect(foot_at(p_to), True)
        move_foot.add_effect(is_free(p_from), True)
        move_foot.add_effect(is_free(p_to), False)
        move_foot.add_effect(foot_x, gripping_point_x(p_to))
        move_foot.add_effect(foot_y, gripping_point_y(p_to))

        problem.add_action(move_foot)

    jump_actions = {}
    for leg_1_id, leg_2_id, med_x_id, med_y_id in itertools.product([0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2],):
        jump_action_name = f"jump_foot_{leg_1_id}_foot_{leg_2_id}_med_x_{med_x_id}_med_y_{med_y_id}"
        jump = InstantaneousAction(jump_action_name,
                                   from_1 =GrippingPoint, from_2=GrippingPoint, p_1=GrippingPoint, p_2=GrippingPoint, p_3=GrippingPoint)

        can_perform_jump = Fluent(f"can_perform_{jump_action_name}", BoolType(), from_1=GrippingPoint,
                                  from_2=GrippingPoint, p_1=GrippingPoint, p_2=GrippingPoint, p_3=GrippingPoint)
        problem.add_fluent(can_perform_jump, default_initial_value=False)
        from_1 = jump.parameter('from_1')
        from_2 = jump.parameter('from_2')
        p_1 = jump.parameter('p_1')
        p_2 = jump.parameter('p_2')
        p_3 = jump.parameter('p_3')
        jump.add_precondition(can_perform_jump(from_1, from_2, p_1, p_2, p_3))
        p1_x = gripping_point_x(p_1)
        p2_x = gripping_point_x(p_1)
        p3_x = gripping_point_x(p_1)
        p1_y = gripping_point_y(p_1)
        p2_y = gripping_point_y(p_2)
        p3_y = gripping_point_y(p_3)
        jump.add_precondition(median_is_index_prec(p1_x, p2_x, p3_x, med_x_id))
        jump.add_precondition(median_is_index_prec(p1_y, p2_y, p3_y, med_y_id))
        jump_actions[jump_action_name] = jump


    for direc in ["up", "down", "left", "right"]:
        move_center = InstantaneousAction(f"move_center_{direc}")
        move_center.add_precondition(
            leg_order_precondition(center_x, center_y, [foot_x for _, foot_x, _ in feet], [foot_y for _, _, foot_y in feet]))
        for _, foot_x, foot_y in feet:
            mod = {"up":(0,1), "down":(0, -1), "left":(-1, 0), "right":(1, 0)}[direc]
            move_center.add_precondition(dist_prec(foot_x, foot_y,
                                                   center_x + mod[0], center_y + mod[1], leg_extension))
            move_center.add_precondition((Not(And(Equals(center_x, foot_x), Equals(center_y, foot_y)))))
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
        foot_at, foot_x, foot_y = feet[i]
        gp = gripping_points[(init_foot_x, init_foot_y)]
        problem.set_initial_value(foot_at(gp), True)

        problem.set_initial_value(foot_x, init_foot_x)
        problem.set_initial_value(foot_y, init_foot_y)

        problem.set_initial_value(is_free(gripping_points[(init_foot_x, init_foot_y)]), False)

    for (jump_action_name, params), val in jump_actions_allowed.items():
        jump_gripping_points = [gripping_points[p] for p in params]
        problem.set_initial_value(jump_actions[jump_action_name](*jump_gripping_points), val)

    init_center_x, init_center_y = instance.init_center
    problem.set_initial_value(center_x, init_center_x)
    problem.set_initial_value(center_y, init_center_y)

    goal_x, goal_y = instance.goal_point
    problem.add_goal(Equals(center_x, goal_x))
    problem.add_goal(Equals(center_y, goal_y))

    return problem

def get_plan(instance: MonkeyBotProblemInstance):
    with OneshotPlanner(name='enhsp') as planner:
        result = planner.solve(get_problem(instance))
        plan = result.plan
        return plan




