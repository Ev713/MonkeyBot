import math
from operator import truediv

import pygame
import pymunk
import pymunk.pygame_util
from pymunk import SimpleMotor

from monkey_bot.robot_controller import Signal
from monkey_bot.simulation_config import InstanceSimulationCoordinator

WIDTH, HEIGHT = 1000, 800

class SpringMotor:
    def __init__(self, spring, min_extension, max_extension):
        self.spring = spring
        self.min_extension = min_extension
        self.max_extension = max_extension
        self.extension_speed = 0

    def set_extension_speed(self, speed):
        self.extension_speed = speed

    def step(self, dt):
        if self.extension_speed== 0:
            return
        new_length = self.spring.rest_length + self.extension_speed * dt
        if new_length <= self.max_extension:
            self.spring.rest_length = new_length

class MonkeyBotSimulator:
    def __init__(self, simulator_config):

        self.sim_config = simulator_config
        self.t = 0
        self.run=False
        self.clock = pygame.time.Clock()

        self.draw_options = None
        self.grip_points = []
        self.goal_point = None

        self.foot_constraints = []
        self.active_grips = []
        self.leg_constraints = []
        self.feet = []
        self.body = None
        self.legs = []
        self.space = None
        self.window = None
        self.shoulders = []
        self.leg_springs = []
        self.leg_angle_locks = []
        self.leg_grooves = []
        self.max_extension = 0
        self.spring_motors = []



    def start_simulation(self, coordinator:InstanceSimulationCoordinator):
        pygame.init()
        self.space = pymunk.Space()
        self.space.gravity = (0, 981)
        self.window = pygame.display.set_mode((self.sim_config.screen_width, self.sim_config.screen_height))
        self.draw_options = pymunk.pygame_util.DrawOptions(self.window)
        self.create_boundaries(self.sim_config.screen_width, self.sim_config.screen_height)
        self.create_grip_points(coordinator.screen_grip_points())
        self.create_goal_point(coordinator.screen_goal_point())
        self.create_monkey_robot(coordinator.screen_init_feet(),
                                 coordinator.screen_init_center(),
                                 coordinator.max_extension)
        self.run=True
        self.draw()

    def get_foot_pos(self, foot_id):
        return self.feet[foot_id][0].position

    def get_center_pos(self):
        return self.body.position

    def get_gp_pos(self, gp_i):
        return self.grip_points[gp_i][0].position

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
        for i, (gp, _) in enumerate(self.grip_points):
            dist = (foot_body.position - gp.position).length
            if dist <= self.epsilon:
                self.active_grips[foot_index] = self._create_pivot_joint(foot_body, gp, gp.position)
                print(f"Foot {foot_index} gripped at {gp.position}")
                return
        raise Exception(f"Grip Failed for foot {foot_index}")

    def release_grip(self, i):
        """
        Release the foot i if it's currently gripping.
        """
        grip_joint = self.active_grips[i]
        if grip_joint is not None:
            self.space.remove(grip_joint)
            self.active_grips[i] = None
            print(f"Foot {i} released")

    def create_grip_points(self, grip_points):
        for (x, y) in grip_points:
            dot = self.create_dot(x,y,(128, 128, 128,100), 5)
            self.grip_points.append(dot)

    def create_goal_point(self, goal_point):
        if goal_point is not None:
            goal_x, goal_y = goal_point
            self.goal_point = self.create_dot(goal_x, goal_y, (255,0,0, 100), 5)

    def create_dot(self, x, y, color, radius):
        body = pymunk.Body(body_type=pymunk.Body.STATIC)
        body.elasticity = 0.5
        body.friction = 0.5
        body.position = (x, y)
        shape = pymunk.Circle(body, radius=radius)
        shape.sensor = True
        shape.color = color
        self.space.add(body, shape)
        return body, shape

    def _create_robot_body(self, init_center_pos):
        robot_body = pymunk.Body(moment=float("inf"))
        robot_body.position = init_center_pos
        body_shape = pymunk.Circle(robot_body, 10)
        body_shape.mass = 10
        body_shape.color = (255, 0, 0, 100)
        body_shape.filter = pymunk.ShapeFilter(group=1)
        self.space.add(robot_body, body_shape)
        self.body = robot_body
        return robot_body

    def _create_leg(self, start, direction, length, leg_thickness=3, ):
        leg_body = pymunk.Body(mass=1, moment=10)
        leg_body.position = start  # base of leg at body center
        leg_body.elasticity = 0.5
        leg_body.friction = 0.5
        # leg points toward foot
        leg_shape = pymunk.Segment(leg_body, (0, 0), (direction.x * length, direction.y * length), leg_thickness)
        leg_shape.mass = 2
        leg_shape.color = (0, 0, 255, 100)
        leg_shape.filter = pymunk.ShapeFilter(group=1)
        self.space.add(leg_body, leg_shape)
        self.legs.append((leg_body, leg_shape))

        return leg_body


    def _create_foot(self, x, y, foot_radius = 5):
        foot_body = pymunk.Body(mass=1, moment=1)
        foot_body.position = (x, y)
        foot_body.elasticity = 0.5
        foot_body.friction=0.5
        foot_shape = pymunk.Circle(foot_body, foot_radius)
        foot_shape.color = (0, 255, 0, 100)
        foot_shape.filter = pymunk.ShapeFilter(group=1)
        self.space.add(foot_body, foot_shape)
        self.feet.append((foot_body, foot_shape))
        return foot_body

    def _create_pivot_joint(self, a, b, p):
        joint = pymunk.PivotJoint(a, b, p, )
        joint.collide_bodies = False
        self.space.add(joint)
        return joint

    def _create_motor(self, a, b):
        m = SimpleMotor(a, b, 1)
        self.space.add(m)
        return m

    def _create_leg_groove(self, leg_body, foot_body, direction, max_extension):
        """
        Creates a groove joint that lets the foot slide linearly
        along the leg's axis (like a telescoping rod).

        leg_body: the leg segment body
        foot_body: the foot body
        length: the leg length (defines the groove span)
        """
        # Groove runs from leg base (0, 0) to tip (length, 0)
        groove_a = pymunk.Vec2d(0, 0)
        groove_b = pymunk.Vec2d(max_extension*direction[0], max_extension*direction[1])
        self.max_extension = max_extension

        groove_joint = pymunk.GrooveJoint(leg_body, foot_body, groove_a, groove_b, (0, 0))

        self.space.add(groove_joint)

        self.leg_grooves.append(groove_joint)

        return groove_joint

    def _create_leg_spring(self, body, foot_body, length, stiffness=10000, damping=50):
        """
        Creates:
          1. A spring between the robot body and the foot (controls radial extension).
          2. A groove joint along the leg's axis (allows the foot to slide along the leg).
          3. A rotary limit joint to keep the foot's rotation aligned with the leg.
        """
        # --- Anchors ---
        body_anchor = pymunk.Vec2d(0, 0)  # center of robot body
        foot_anchor = pymunk.Vec2d(0, 0)  # center of foot

        spring = pymunk.DampedSpring(
            body, foot_body,
            body_anchor, foot_anchor,
            rest_length=length,
            stiffness=stiffness,
            damping=damping,
        )

        self.space.add(spring)
        self.leg_springs.append(spring)
        return spring


    def create_monkey_robot(self, init_feet_pos, init_center_pos, max_extension):
        body = self._create_robot_body(init_center_pos)

        for i, (x, y) in enumerate(init_feet_pos):
            # convert to screen position
            foot_screen_pos = pymunk.Vec2d(x, y)

            # compute direction from body to foot
            body_pos = self.body.position
            vec = foot_screen_pos - body_pos
            length = vec.length
            direction = vec.normalized()

            leg = self._create_leg(body_pos, direction, length)
            foot = self._create_foot(*foot_screen_pos)

            leg_joint = self._create_pivot_joint(body, leg, body_pos)
            m = self._create_motor(body, leg)
            self.shoulders.append(m)

            self._create_leg_groove(leg, foot, direction, max_extension)
            spring = self._create_leg_spring(body, foot, length)
            self.spring_motors.append(SpringMotor(spring, 0 , self.max_extension))

            self.leg_constraints.append(leg_joint)

        self.active_grips = [None for _ in enumerate(self.legs)]
        for i, _ in enumerate(self.feet):
            self.check_and_grip(i)


    def draw(self):
        self.window.fill("white")
        self.space.debug_draw(self.draw_options)
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

    def set_rotation_speed(self, i, v):
        m = self.shoulders[i]
        if m.rate != v:
            m.rate=v
            print(f"Shoulder {i} rate set to: {m.rate}")


    def set_extension_speed(self, i, v):
        self.spring_motors[i].set_extension_speed(v)


    def apply_signal(self, signal:Signal):
        """
        Apply control signal by setting angular and linear velocities,
        not by directly modifying positions.
        """
        for i, _ in enumerate(self.legs):
            if signal.grip[i] == 1:
                self.check_and_grip(i)
            if signal.grip[i] == -1:
                self.release_grip(i)
            self.set_rotation_speed(i, signal.rotation[i])
            self.set_extension_speed(i, signal.extension[i])


    def step(self):
        for m in self.spring_motors:
            m.step(self.dt)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.run=False
                pygame.quit()
                break
        self.t += 1
        self.draw()
        self.space.step(self.dt)
        self.clock.tick(self.fps)

    def get_state(self):
        return {"center_pos" : self.get_center_pos(),
            "feet_pos" : [self.get_foot_pos(i) for i, _ in enumerate(self.feet)],
            "active_grips" : [a for a in self.active_grips],
            "t":self.t}

