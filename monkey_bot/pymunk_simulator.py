import pygame
import pymunk
import pymunk.pygame_util
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.robot_controller import Controller

WIDTH, HEIGHT = 1000, 800


class MonkeyBotSimulator:
    def __init__(self, instance: MonkeyBotProblemInstance):
        pygame.init()
        self.instance = instance

        self.epsilon = 10
        self.screen_width = WIDTH
        self.screen_height = HEIGHT
        self.fps = 60
        self.dt = 1/self.fps

        self.grid_width = instance.grid_size_x
        self.grid_height = instance.grid_size_y
        self.grip_points = instance.gripping_points
        self.goal_point = instance.goal_point
        self.init_feet_pos = self.instance.init_feet
        self.init_center_pos = self.instance.init_center

        self.foot_constraints = []
        self.active_grips = [None for _ in enumerate(instance.init_feet)]
        self.leg_constraints = []
        self.feet = []
        self.body = None
        self.legs = []
        self.space = None
        self.window = None
        self.controller = None


    def init_controller(self):
        self.controller = Controller(
            self.instance,
            dt=self.dt,
            move_center_speed=1*self.fps,
            extension_speed= 1*self.fps,
            twist_speed= 1*self.fps,
            epsilon = 10,
            screen_width=1000,
            screen_height=800)

    def start_simulation(self):
        self.space = pymunk.Space()
        self.space.gravity = (0, 981)
        self.window = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.create_boundaries(self.screen_width, self.screen_height)
        self.create_monkey_robot()

    def get_foot_pos(self, foot_id):
        return self.feet[foot_id][0].position

    def get_center_pos(self):
        return self.body.position

    def check_and_grip(self, foot_index: int):
        """
        Check the specified foot (by index); if it is near a gripping point,
        attach it to that point with a PivotJoint.
        """
        # get the chosen foot and its current grip state
        foot_body, _ = self.feet[foot_index]

        # skip if already gripping
        if self.active_grips[foot_index] is not None:
            return

        # iterate over available grip points
        for gp in self.grip_points:
            screen_gp = self.controller.grid_to_screen(*gp)
            dist = (foot_body.position - screen_gp).length
            if dist <= self.epsilon:
                # Create static body joint to hold
                grip_joint = pymunk.PivotJoint(foot_body, self.space.static_body, screen_gp)
                grip_joint.collide_bodies = False
                self.space.add(grip_joint)
                self.active_grips[foot_index] = grip_joint
                print(f"Foot {foot_index} gripped at {gp}")
                break

    def release_grip(self, i):
        """
        Release the foot i if it's currently gripping.
        """
        grip_joint = self.active_grips[i]
        if grip_joint is not None:
            self.space.remove(grip_joint)
            self.active_grips[i] = None
            print(f"Foot {i} released")

    def create_monkey_robot(self):
        robot_body = pymunk.Body(moment = 1000)
        robot_body.position = self.controller.grid_to_screen(*self.init_center_pos)
        body_shape = pymunk.Circle(robot_body, 10)
        body_shape.mass = 10
        body_shape.color = (255, 0, 0, 100)
        self.space.add(robot_body, body_shape)
        self.body = robot_body

        # === LEGS ===
        leg_thickness = 3
        foot_radius = 5

        for i, (x, y) in enumerate(self.init_feet_pos):
            # convert to screen position
            foot_screen_pos = pymunk.Vec2d(*self.controller.grid_to_screen(x, y))

            # compute direction from body to foot
            body_pos = robot_body.position
            vec = foot_screen_pos - body_pos
            length = vec.length
            direction = vec.normalized()

            # === LEG ===
            leg_body = pymunk.Body(mass=1)
            leg_body.position = body_pos  # base of leg at body center
            # leg points toward foot
            leg_shape = pymunk.Segment(leg_body, (0, 0), (direction.x * length, direction.y * length), leg_thickness)
            leg_shape.mass = 2
            leg_shape.color = (0, 0, 255, 100)
            self.space.add(leg_body, leg_shape)

            # === FOOT ===
            foot_body = pymunk.Body(mass=1, moment=1000)
            foot_body.position = foot_screen_pos
            foot_shape = pymunk.Circle(foot_body, foot_radius)
            foot_shape.color = (0, 255, 0, 100)
            self.space.add(foot_body, foot_shape)

            # connect leg base to body center
            leg_joint = pymunk.PivotJoint(robot_body, leg_body, body_pos)
            leg_joint.collide_bodies=False
            self.space.add(leg_joint)

            # connect foot to leg tip
            foot_joint = pymunk.PivotJoint(leg_body, foot_body, foot_screen_pos)
            foot_joint.collide_bodies = False
            self.space.add(foot_joint)

            # store parts for reference
            self.legs.append((leg_body, leg_shape))
            self.feet.append((foot_body, foot_shape))
            self.leg_constraints.append(leg_joint)
            self.foot_constraints.append(foot_joint)

        for _, leg_shape in self.legs :
            leg_shape.filter = pymunk.ShapeFilter(group=1)

        for i, _ in enumerate(self.feet):
            self.check_and_grip(i)




    def draw(self, draw_options, gripping_points, goal):
        self.window.fill("white")
        for (x, y) in gripping_points:
            x, y = self.controller.grid_to_screen(x, y)
            pygame.draw.circle(self.window, 'grey', (int(x), int(y)), 5)
        if goal is not None:
            goal_x, goal_y = goal
            pygame.draw.circle(self.window, 'red', (self.controller.grid_to_screen(goal_x, goal_y)), 5)

        self.space.debug_draw(draw_options)
        pygame.display.update()

    def create_boundaries(self, width, height):
        rects = [
            [(width/2, height-10), (width, 20)],
            [(width / 2, 10), (width, 20)],
            [(10, height /2), (20, height)],
            [(width - 10, height/2), (20, height)],
        ]

        for pos, size in rects:
            body = pymunk.Body(body_type = pymunk.Body.STATIC)
            body.position = pos
            shape = pymunk.Poly.create_box(body, size)
            self.space.add(body, shape)

    def apply_signal(self, signal):
        """
        Apply control signal by setting angular and linear velocities,
        not by directly modifying positions.
        """
        twist_speed = self.controller.twist_speed  # radians/sec
        extension_speed = self.controller.extension_speed  # pixels/sec
        center_pos = self.body.position

        for i, ((leg_body, leg_shape), (foot_body, foot_shape)) in enumerate(zip(self.legs, self.feet)):
            # 1 HIP TWIST: rotate leg around body center
            tw = signal["hip_twist_speeds"][i]
            if tw != 0:
                # Compute desired angular velocity for the leg
                leg_body.angular_velocity = tw * twist_speed
            else:
                leg_body.angular_velocity = 0

            # 2 LEG EXTENSION: move foot along leg direction
            ext = signal["leg_speeds"][i]
            if ext != 0:
                # Compute direction from body to foot
                leg_vec = foot_body.position - center_pos
                if leg_vec.length > 1e-3:
                    direction = leg_vec.normalized()
                    foot_body.velocity = direction * (ext * extension_speed)
            else:
                # If no extension command, let damping stop it naturally
                foot_body.velocity *= 0.9

            # 3 GRIP CONTROL
            if signal["perform_grip"][i]== 1:
                self.check_and_grip(i)
            elif signal["perform_grip"][i] == -1:
                self.release_grip(i)

    def run(self):
        run = True
        clock = pygame.time.Clock()
        fps = 60
        dt = 1/fps
        frame = 0
        draw_options = pymunk.pygame_util.DrawOptions(self.window)
        while run:
            frame += 1
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False
                    break
            self.draw(draw_options, self.grip_points, self.goal_point)
            self.space.step(dt)
            self.update_controller_data()
            sig = self.controller.get_signals()
            self.apply_signal(sig)
            clock.tick(fps)

        pygame.quit()

    def update_controller_data(self):
        self.controller.center_pos = self.get_center_pos()
        self.controller.feet_pos = [self.get_foot_pos(i) for i, _ in enumerate(self.feet)]
        self.controller.active_grips = [a for a in self.active_grips]

