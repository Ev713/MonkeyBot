from unified_planning.shortcuts import *
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance


def dist_prec(x1, y1, x2, y2, val):
    return LE(get_dist_squard_fluent(x1,y1,x2,y2), Times(val, val))

def get_dist_squard_fluent(x1,y1,x2,y2):
    return Plus(Times(x1-x2,x1-x2), Times(y1-y2,y1-y2))

def get_dummy_numeric_problem():
    Counter = UserType('Counter')

    value = Fluent('value', IntType(), m=Counter)
    inc = InstantaneousAction('increment', c=Counter)
    c = inc.parameter('c')
    inc.add_precondition(LE(value(c), 10))
    inc.add_increase_effect(value(c), 1)

    dec = InstantaneousAction('decrement', c=Counter)
    c = dec.parameter('c')
    dec.add_precondition(GT(value(c), 0))
    dec.add_decrease_effect(value(c), 1)
    problem = Problem('problem')

    problem.add_fluent(value, default_initial_value=0)
    C0 = Object('c0', Counter)
    C1 = Object('c1', Counter)
    C2 = Object('c2', Counter)
    problem.add_object(C0)
    problem.add_object(C1)
    problem.add_object(C2)
    problem.add_action(inc)
    problem.add_action(dec)
    problem.add_goal(And(GE(value(C2), Plus(value(C1), 1)), GE(value(C1), Plus(value(C0), 1))))
    N = 9  # This is the number of counters

    p2 = Problem('Large_problems')

    p2.add_fluent(value, default_initial_value=0)
    p2.add_objects([Object(f'c{i}', Counter) for i in range(N)])
    p2.add_action(inc)
    p2.add_action(dec)

    for i in range(N - 1):
        p2.add_goal(GE(value(p2.object(f'c{i + 1}')), Plus(value(p2.object(f'c{i}')), 1)))
    return problem

def get_problem(instance: MonkeyBotProblemInstance):
    leg_num = len(instance.init_feet)
    leg_extension = instance.leg_extension
    grid_size_x = instance.grid_size_x
    grid_size_y = instance.grid_size_y

    problem = Problem()

    GrippingPoint = UserType("Gripping_point")
    gripping_point_x = Fluent("gripping_point_x", IntType(),p=GrippingPoint)
    gripping_point_y = Fluent("gripping_point_y", IntType(), p=GrippingPoint)

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
        move_foot.add_precondition(
            dist_prec(center_x, center_y, gripping_point_x(p_to), gripping_point_y(p_to), leg_extension))
        move_foot.add_effect(foot_at(p_from), False)
        move_foot.add_effect(foot_at(p_to), True)
        problem.add_action(move_foot)

    for direc in ["up", "down", "left", "right"]:
        move_center = InstantaneousAction(f"move_center_{direc}", **{f"p{i}": GrippingPoint for i in range(leg_num)})
        leg_points = [move_center.parameter(f"p{i}") for i in range(leg_num) ]
        for leg_point, foot_at in zip(leg_points, feet):
            move_center.add_precondition(foot_at(leg_point))
            mod = {"up":(0,1), "down":(0, -1), "left":(-1, 0), "right":(1, 0)}[direc]
            move_center.add_precondition(dist_prec(gripping_point_x(leg_point), gripping_point_y(leg_point),
                                                   center_x + mod[0], center_y + mod[1], leg_extension))
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

    init_center_x, init_center_y = instance.init_center
    problem.set_initial_value(center_x, init_center_x)
    problem.set_initial_value(center_y, init_center_y)

    goal_x, goal_y = instance.goal_point
    problem.add_goal(Equals(center_x, goal_x))
    problem.add_goal(Equals(center_y, goal_y))

    return problem

def get_plan(instance: MonkeyBotProblemInstance):
    with OneshotPlanner(name="enhsp") as planner:
        result = planner.solve(get_problem(instance))
        plan = result.plan
        return plan




