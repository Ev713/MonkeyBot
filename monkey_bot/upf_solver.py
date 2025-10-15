from typing import Tuple

from unified_planning.shortcuts import *
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance

LEG_MIN_EXT = 0.25

def dist_prec(x1, y1, x2, y2, val):
    return LE(get_dist_squard_fluent(x1,y1,x2,y2), Times(val, val))

def get_dist_squard_fluent(x1,y1,x2,y2):
    return Plus(Times(x1-x2,x1-x2), Times(y1-y2,y1-y2))

def leg_order_precondition(center_x:int, center_y:int, legs_x:List[int], legs_y:List[int]):
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


def get_problem(instance: MonkeyBotProblemInstance):
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
        feet.append(foot_at)
        problem.add_fluent(foot_at, default_initial_value=False)
        move_foot = InstantaneousAction(f"move_foot_{i}", p_from=GrippingPoint, p_to=GrippingPoint)
        p_from = move_foot.parameter('p_from')
        p_to = move_foot.parameter('p_to')
        move_foot.add_precondition(foot_at(p_from))
        move_foot.add_precondition(is_free(p_to))
        move_foot.add_precondition(
            dist_prec(center_x, center_y, gripping_point_x(p_to), gripping_point_y(p_to), leg_extension))

        move_foot.add_effect(foot_at(p_from), False)
        move_foot.add_effect(foot_at(p_to), True)
        move_foot.add_effect(is_free(p_from), True)
        move_foot.add_effect(is_free(p_to), False)
        problem.add_action(move_foot)

    for direc in ["up", "down", "left", "right"]:
        move_center = InstantaneousAction(f"move_center_{direc}", **{f"p{i}": GrippingPoint for i in range(leg_num)})
        leg_points = [move_center.parameter(f"p{i}") for i in range(leg_num) ]
        move_center.add_precondition(
            leg_order_precondition(center_x, center_y, [gripping_point_x(leg_point) for leg_point in leg_points],
                                   [gripping_point_y(leg_point) for leg_point in leg_points ]))
        for leg_point, foot_at in zip(leg_points, feet):
            move_center.add_precondition(foot_at(leg_point))
            mod = {"up":(0,1), "down":(0, -1), "left":(-1, 0), "right":(1, 0)}[direc]
            move_center.add_precondition(dist_prec(gripping_point_x(leg_point), gripping_point_y(leg_point),
                                                   center_x + mod[0], center_y + mod[1], leg_extension))
            move_center.add_precondition((Not(And(Equals(center_x, gripping_point_x(leg_point)),
                                                  Equals(center_y, gripping_point_y(leg_point))))))
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
        problem.set_initial_value(feet[i](gripping_points[(init_foot_x, init_foot_y)]), True)
        problem.set_initial_value(is_free(gripping_points[(init_foot_x, init_foot_y)]), False)

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




