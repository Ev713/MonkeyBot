from unified_planning.shortcuts import *

LEG_NUM = 3

def get_problem():
    grid_size = 100
    gripping_point = UserType("Gripping_point")
    gripping_point_x = Fluent("gripping_point_x", IntType(),p=gripping_point)
    gripping_point_y = Fluent("gripping_point_y", IntType(), p=gripping_point)

    feet = []
    for i in range(LEG_NUM):
        foot_x = Fluent(f"robot_foot_{i}_x", BoolType(), p=gripping_point)
        foot_y = Fluent(f"robot_foot_{i}_y", BoolType(), p=gripping_point)
        feet.append((foot_x, foot_y))

    center = Fluent("center", IntType(0, 100))




    for i in range(LEG_NUM):
        move_foot = InstantaneousAction(f"move_foot_{i}",)
