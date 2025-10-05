from unified_planning.shortcuts import *
import MonkeyBotProblemInstance

LEG_NUM = 3
LEG_EXTENSION = 3
GRID_SIZE = 100


def dist_prec(x1, y1, x2, y2, val, ):
    return LE(Plus(Times(x1-x2,x1-x2), Times(y1-y2,y1-y2)), Times(val, val))


def get_problem(instance: MonkeyBotProblemInstance):
    problem = Problem()

    GrippingPoint = UserType("Gripping_point")
    gripping_point_x = Fluent("gripping_point_x", IntType(),p=GrippingPoint)
    gripping_point_y = Fluent("gripping_point_y", IntType(), p=GrippingPoint)

    center_x = Fluent("center_x", IntType(0, GRID_SIZE))
    center_y = Fluent("center_y", IntType(0, GRID_SIZE))
    problem.add_fluent(center_x)
    problem.add_fluent(center_y)

    feet = []
    for i in range(LEG_NUM):
        foot_x = Fluent(f"robot_foot_{i}_x", BoolType(), p=GrippingPoint)
        foot_y = Fluent(f"robot_foot_{i}_y", BoolType(), p=GrippingPoint)
        feet.append((foot_x, foot_y))

        move_foot = InstantaneousAction(f"move_foot_{i}", p=GrippingPoint)
        p = move_foot.parameter('p')
        move_foot.add_precondition(
            dist_prec(center_x, center_y, gripping_point_x(p), gripping_point_y(p), LEG_EXTENSION))
        move_foot.add_effect(foot_x, gripping_point_x(p))
        move_foot.add_effect(foot_y, gripping_point_y(p))
        problem.add_action(move_foot)

    move_center_up = InstantaneousAction(f"move_center_up",)
    for foot_x, foot_y in feet:
        move_center_up.add_precondition(dist_prec(foot_x, foot_y, center_x, center_y+1, LEG_EXTENSION))
    move_center_up.add_precondition(LE(center_y, GRID_SIZE))
    move_center_up.add_effect(center_y, center_y+1)
    problem.add_action(move_center_up)

    move_center_down = InstantaneousAction(f"move_center_down",)
    for foot_x, foot_y in feet:
        move_center_down.add_precondition(dist_prec(foot_x, foot_y, center_x, center_y-1, LEG_EXTENSION))
    move_center_down.add_precondition(GE(center_y, 1))
    move_center_down.add_effect(center_y, center_y-1)
    problem.add_action(move_center_down)

    move_center_left = InstantaneousAction(f"move_center_left",)
    for foot_x, foot_y in feet:
        move_center_left.add_precondition(dist_prec(foot_x, foot_y, center_x-1, center_y, LEG_EXTENSION))
    move_center_left.add_precondition(GE(center_x, 1))
    move_center_left.add_effect(center_x, center_x-1)
    problem.add_action(move_center_left)

    move_center_right = InstantaneousAction(f"move_center_right", )
    for foot_x, foot_y in feet:
        move_center_right.add_precondition(dist_prec(foot_x, foot_y, center_x + 1, center_y, LEG_EXTENSION))
    move_center_right.add_precondition(LE(center_x, GRID_SIZE))
    move_center_right.add_effect(center_x, center_x + 1)
    problem.add_action(move_center_right)

    for (gp_x, gp_y), i in enumerate(instance.gripping_points):
        gp = Object()





