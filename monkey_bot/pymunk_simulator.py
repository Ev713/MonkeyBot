import math
from operator import truediv

import numpy as np
import pygame
import imageio as imageio  # install: pip install imageio
import pygame
import pymunk
import pymunk.pygame_util
from pymunk import SimpleMotor, Transform, Vec2d
from scipy.optimize import newton_krylov

import monkey_bot.runtime_debug as runtime_debug
from monkey_bot.signals import ControlSignal, StateSignal
from monkey_bot.config import InstanceSimulationConfig


def _is_finite_scalar(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _coords_ok(*values) -> bool:
    for value in values:
        if isinstance(value, Vec2d):
            if not (_is_finite_scalar(value.x) and _is_finite_scalar(value.y)):
                return False
        elif isinstance(value, (tuple, list)) and len(value) >= 2:
            if not (_is_finite_scalar(value[0]) and _is_finite_scalar(value[1])):
                return False
        elif not _is_finite_scalar(value):
            return False
    return True


class SafeDrawOptions(pymunk.pygame_util.DrawOptions):
    """Skip debug-draw primitives with non-finite coordinates (avoids pygame spam)."""

    def draw_circle(self, pos, angle, radius, outline_color, fill_color):
        if not _coords_ok(pos, angle, radius):
            return
        super().draw_circle(pos, angle, radius, outline_color, fill_color)

    def draw_segment(self, a, b, color):
        if not _coords_ok(a, b):
            return
        super().draw_segment(a, b, color)

    def draw_fat_segment(self, a, b, radius, outline_color, fill_color):
        if not _coords_ok(a, b, radius):
            return
        super().draw_fat_segment(a, b, radius, outline_color, fill_color)

    def draw_polygon(self, verts, radius, outline_color, fill_color):
        if not all(_coords_ok(v) for v in verts):
            return
        super().draw_polygon(verts, radius, outline_color, fill_color)

    def draw_dot(self, size, pos, color):
        if not _coords_ok(pos, size):
            return
        super().draw_dot(size, pos, color)


class RotationMotor:
    def __init__(self, body, leg):
        self.motor = SimpleMotor(body, leg, 0)
        self.body = body
        self.leg = leg
        self.desired_rate = 0
        self.last_angle = 0
        self.last_body_angle = 0
        self.log = None

    def enable_log(self, path="experiments/results/logs/adjust_angle_data.txt"):
        import os
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.log = open(path, 'w')

    def step(self, dt):
        real_rate = self.actual_rate(dt)
        if self.desired_rate is None:
            new_rate = real_rate
        elif self.desired_rate == 0:
            new_rate = self.desired_rate
        else:
            new_rate = self.desired_rate
        self.motor.rate = new_rate
        self.last_angle = self.curr_angle
        self.last_body_angle = self.curr_body_angle
        if self.log is not None:
            self.log.write(f"{self.desired_rate}, {real_rate}, {new_rate}, {self.body_rate(dt)}\n")

    def body_rate(self, dt):
        return -self.normalize_angle(self.curr_body_angle - self.last_body_angle) / dt

    def actual_rate(self, dt):
        return -self.normalize_angle(self.curr_angle - self.last_angle)/dt

    def normalize_angle(self, angle):
        return  (angle + math.pi) % (2 * math.pi) - math.pi

    @property
    def curr_angle(self):
        return self.normalize_angle(self.leg.angle - self.body.angle)

    @property
    def curr_body_angle(self):
        angle_diff = self.body.angle  # =  self.body.angle - self.leg.angle
        return self.normalize_angle(angle_diff)

class SpringMotor:
    def __init__(self, spring, min_extension, max_extension):
        self.spring = spring
        self.min_extension = min_extension
        self.max_extension = max_extension
        self.extension_speed = 0

    def set_extension_speed(self, speed):
        self.extension_speed = speed

    def calibrate(self):
        actual_length = min((self.spring.a.position - self.spring.b.position).length, self.max_extension)
        if actual_length - self.spring.rest_length > 5:
            self.spring.rest_length = actual_length

    def relax_to_current(self):
        actual_length = min(
            (self.spring.a.position - self.spring.b.position).length,
            self.max_extension,
        )
        self.spring.rest_length = actual_length
        self.extension_speed = 0

    def step(self, dt):
        if self.extension_speed== 0:
            return
        new_length = self.spring.rest_length + self.extension_speed * dt
        if self.min_extension<=new_length <= self.max_extension:
            self.spring.rest_length = new_length


class MonkeyBotSimulator:
    def __init__(self, simulator_config):

        self.epsilon = None
        self.simulation_name = "simulation.mp4"
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
        self.rotation_motors = []
        self.leg_springs = []
        self.leg_angle_locks = []
        self.body_leg_angle_locks: list[pymunk.RotaryLimitJoint | None] = []
        self.leg_grooves = []
        self.spring_motors = []
        self.writer = None
        self._display_active = False
        self._halted = False
        self.coordinator = None
        self._last_signal: ControlSignal | None = None

        self._max_body_angular_velocity = 50.0
        self._max_speed_px_s = 2000.0


    def start_simulation(self, coordinator:InstanceSimulationConfig):
        pygame.init()
        self.coordinator = coordinator
        self._last_signal = None
        self._display_active = True
        self._halted = False
        self.space = pymunk.Space()
        self.epsilon = coordinator.epsilon
        self.grip_tolerance = coordinator.epsilon + coordinator.foot_radius
        self.space.gravity = (0, coordinator.gravity)
        self.simulation_name = coordinator.instance.name
        self.window = pygame.display.set_mode((self.sim_config.screen_width, self.sim_config.screen_height))
        self.draw_options = SafeDrawOptions(self.window)
        self.configure_render_transform()
        self.create_grip_points(coordinator.screen_grip_points())
        self.create_goal_point(coordinator.screen_goal_point())
        self.create_monkey_robot(coordinator)
        self.run=True
        self.draw()

    def configure_render_transform(self):
        render_scale = getattr(self.sim_config, "render_scale", 1) or 1
        center_x = self.sim_config.screen_width / 2
        center_y = self.sim_config.screen_height / 2
        self.draw_options.transform = Transform(
            render_scale,
            0,
            0,
            render_scale,
            center_x * (1 - render_scale),
            center_y * (1 - render_scale),
        )


    def get_foot_pos(self, foot_id):
        return Vec2d(*self.feet[foot_id][0].position)

    def initiate_saver(self, name=None):
        if name is None:
            name = f"videos/{self.simulation_name}.mp4"
        self.writer = imageio.get_writer(name, fps=self.sim_config.fps, codec='libx264',)

    def get_center_pos(self):
        return Vec2d(*self.body.position)

    def get_gp_pos(self, gp_i):
        return Vec2d(*self.grip_points[gp_i][0].position)

    def check_and_grip(self, foot_index: int) -> bool:
        foot_body, _ = self.feet[foot_index]
        if self.active_grips[foot_index] is not None:
            return True
        tolerance = getattr(self, "grip_tolerance", self.epsilon)
        for _i, (gp, _) in enumerate(self.grip_points):
            gp_pos = Vec2d(*gp.position)
            dist = (foot_body.position - gp_pos).length
            if dist <= tolerance:
                if not self.coordinator.disable_assist:
                    self._snap_foot_to_grip_point(foot_index, gp_pos)
                self.active_grips[foot_index] = self._create_pivot_joint(
                    foot_body, gp, gp_pos
                )
                return True
        return False

    def _snap_foot_to_grip_point(self, foot_index: int, gp_pos: Vec2d) -> None:
        """Move the foot onto the GP center when a grip engages."""
        foot_body, _ = self.feet[foot_index]
        foot_body.position = gp_pos
        foot_body.velocity = (0, 0)
        foot_body.angular_velocity = 0
        if foot_index < len(self.spring_motors):
            self.spring_motors[foot_index].calibrate()

    def release_grip(self, i):
        self.unlock_body_leg_angle(i)
        grip_joint = self.active_grips[i]
        if grip_joint is not None:
            self.spring_motors[i].calibrate()
            self.space.remove(grip_joint)
            self.active_grips[i] = None

    def create_grip_points(self, grip_points):
        for (x, y) in grip_points:
            dot = self.create_dot(x,y,(128, 128, 128,100), 5)
            self.grip_points.append(dot)

    def create_goal_point(self, goal_point):
        if goal_point is not None:
            goal_x, goal_y = goal_point
            self.goal_point = self.create_dot(goal_x, goal_y, (255, 0, 0, 100), 5)

    def update_goal_point(self, goal_point, coordinator=None):
        if goal_point is None:
            return
        if self.goal_point is not None:
            body, shape = self.goal_point
            self.space.remove(body, shape)
            self.goal_point = None
        goal_x, goal_y = float(goal_point.x), float(goal_point.y)
        self.goal_point = self.create_dot(goal_x, goal_y, (255, 0, 0, 100), 5)
        if coordinator is not None:
            grid = coordinator.screen_to_grid(goal_point)
            print(
                f"Goal marker moved to grid {grid} "
                f"screen ({goal_x:.0f}, {goal_y:.0f})",
                flush=True,
            )

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

    def _create_robot_body(self, init_center_pos, body_radius, body_mass, body_moment):
        robot_body = pymunk.Body(moment=body_moment)
        robot_body.position = init_center_pos
        body_shape = pymunk.Circle(robot_body, body_radius)
        body_shape.mass = body_mass
        body_shape.color = (255, 0, 0, 100)
        body_shape.filter = pymunk.ShapeFilter(group=1)
        self.space.add(robot_body, body_shape)
        self.body = robot_body
        return robot_body

    def _create_leg_body(self, start, direction, length, leg_thickness, mass):
        moment = pymunk.moment_for_segment(mass, (0, 0), (length, 0), leg_thickness)
        leg_body = pymunk.Body(mass=mass, moment=moment)
        leg_body.position = start
        leg_body.angle = math.atan2(direction.y, direction.x)
        leg_body.elasticity = 0.5
        leg_body.friction = 0.5
        # Leg extends along local +x from the hip pivot at (0, 0).
        leg_shape = pymunk.Segment(leg_body, (0, 0), (length, 0), leg_thickness)
        leg_shape.mass = mass
        leg_shape.color = (0, 0, 255, 100)
        leg_shape.filter = pymunk.ShapeFilter(group=1)
        self.space.add(leg_body, leg_shape)
        self.legs.append((leg_body, leg_shape))

        return leg_body


    def _create_foot(self, x, y, foot_radius, mass):
        foot_body = pymunk.Body(mass=mass, moment=0.01)
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

    def _create_rotation_motor(self, a, b):
        m = RotationMotor(a, b)
        self.space.add(m.motor)
        return m

    def _create_leg_groove(self, leg_body, foot_body, max_extension):
        """
        Lets the foot slide linearly along the leg's local +x axis (telescoping rod).
        """
        groove_a = pymunk.Vec2d(0, 0)
        groove_b = pymunk.Vec2d(max_extension, 0)
        self.max_extension = max_extension

        groove_joint = pymunk.GrooveJoint(leg_body, foot_body, groove_a, groove_b, (0, 0))
        self.space.add(groove_joint)
        self.leg_grooves.append(groove_joint)
        return groove_joint

    def _normalize_rel_angle(self, angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _body_leg_rel_angle(self, leg_id: int) -> float:
        body = self.body
        leg = self.legs[leg_id][0]
        return self._normalize_rel_angle(leg.angle - body.angle)

    def lock_body_leg_angle(self, leg_id: int):
        if self.body_leg_angle_locks[leg_id] is not None:
            return
        body = self.body
        leg = self.legs[leg_id][0]
        ref = self._body_leg_rel_angle(leg_id)
        lock = pymunk.RotaryLimitJoint(body, leg, ref, ref)
        lock.collide_bodies = False
        self.space.add(lock)
        self.body_leg_angle_locks[leg_id] = lock
        self.rotation_motors[leg_id].desired_rate = 0

    def unlock_body_leg_angle(self, leg_id: int):
        lock = self.body_leg_angle_locks[leg_id]
        if lock is None:
            return
        self.space.remove(lock)
        self.body_leg_angle_locks[leg_id] = None

    def is_body_leg_angle_locked(self, leg_id: int) -> bool:
        return self.body_leg_angle_locks[leg_id] is not None

    def _create_leg_foot_lock(self, leg_body, foot_body):
        """Keep the foot aligned with the leg segment."""
        lock = pymunk.RotaryLimitJoint(leg_body, foot_body, 0, 0)
        self.space.add(lock)
        self.leg_angle_locks.append(lock)
        return lock

    def _create_leg_spring(self, leg_body, foot_body, length, stiffness, damping):
        """
        Extension spring along the leg axis (hip to foot), not body to foot.
        """
        spring = pymunk.DampedSpring(
            leg_body,
            foot_body,
            pymunk.Vec2d(0, 0),
            pymunk.Vec2d(0, 0),
            rest_length=length,
            stiffness=stiffness,
            damping=damping,
        )

        self.space.add(spring)
        self.leg_springs.append(spring)
        return spring


    def create_monkey_robot(self, coordinator:InstanceSimulationConfig):
        body = self._create_robot_body(
            coordinator.screen_init_center(),
            coordinator.body_radius,
            coordinator.body_mass,
            coordinator.body_moment,
        )
        for i, (x, y) in enumerate(coordinator.screen_init_feet()):
            # convert to screen position
            foot_screen_pos = pymunk.Vec2d(x, y)

            # compute direction from body to foot
            body_pos = self.body.position
            vec = foot_screen_pos - body_pos
            length = vec.length
            direction = vec.normalized()

            leg = self._create_leg_body(body_pos, direction, coordinator.min_extension,
                                        coordinator.leg_thickness, coordinator.leg_mass)
            foot = self._create_foot(*foot_screen_pos, coordinator.foot_radius, coordinator.foot_mass)

            leg_joint = self._create_pivot_joint(body, leg, body_pos)
            m = self._create_rotation_motor(body, leg)
            self.rotation_motors.append(m)

            self._create_leg_groove(leg, foot, coordinator.max_extension)
            self._create_leg_foot_lock(leg, foot)
            spring = self._create_leg_spring(leg, foot, length, coordinator.stiffness, coordinator.damping)
            self.spring_motors.append(SpringMotor(spring, coordinator.min_extension , coordinator.max_extension))

            self.leg_constraints.append(leg_joint)

        self.body_leg_angle_locks = [None] * len(self.legs)
        self.active_grips = [None for _ in enumerate(self.legs)]
        for i, _ in enumerate(self.feet):
            self.check_and_grip(i)


    def _halt_simulation(self, reason: str):
        if self._halted:
            return
        self._halted = True
        self.run = False
        print(reason, flush=True)
        self._print_halt_diagnostics(reason)
        if self.writer is not None:
            self.writer.close()
            self.writer = None
        if self._display_active:
            self._display_active = False
            try:
                pygame.quit()
            except pygame.error:
                pass

    def _physics_to_screen(self, x: float, y: float) -> tuple[int, int]:
        t = self.draw_options.transform
        sx = t.a * x + t.c * y + t.tx
        sy = t.b * x + t.d * y + t.ty
        return int(round(sx)), int(round(sy))

    def _draw_procedure_overlays(self) -> None:
        orange = (255, 140, 0)
        line = runtime_debug.move_center_line
        if line is not None:
            (x0, y0), (x1, y1) = line
            if _coords_ok(x0, y0, x1, y1):
                pygame.draw.line(
                    self.window,
                    orange,
                    self._physics_to_screen(x0, y0),
                    self._physics_to_screen(x1, y1),
                    3,
                )
        target = runtime_debug.attach_target
        if target is not None:
            (cx, cy), radius = target
            if _coords_ok(cx, cy, radius):
                t = self.draw_options.transform
                screen_r = max(4, int(round(abs(radius * t.a))))
                pygame.draw.circle(
                    self.window,
                    orange,
                    self._physics_to_screen(cx, cy),
                    screen_r,
                    3,
                )

    def draw(self):
        if not self.run or not self._display_active or self.window is None:
            return
        if not self._physics_is_valid():
            self._halt_simulation("Simulation unstable (invalid physics state).")
            return
        try:
            self.window.fill("white")
            self.space.debug_draw(self.draw_options)
            self._draw_procedure_overlays()
            pygame.display.update()
        except (TypeError, pygame.error):
            self._halt_simulation("Simulation display failed (physics likely unstable).")

    def _vec_is_finite(self, vec) -> bool:
        return math.isfinite(vec.x) and math.isfinite(vec.y)

    def _speed_px_s(self, body) -> float:
        return math.hypot(body.velocity.x, body.velocity.y)

    def _collect_halt_issues(
        self,
        label: str,
        body,
        *,
        check_angle: bool = False,
        max_angular_velocity: float | None = None,
    ) -> list[str]:
        """Issues that should stop the simulation (non-finite state, runaway body spin)."""
        issues = []
        pos = body.position
        vel = body.velocity
        if not self._vec_is_finite(pos):
            issues.append(f"{label}: NON-FINITE position ({pos.x}, {pos.y})")
        if not self._vec_is_finite(vel):
            issues.append(f"{label}: NON-FINITE velocity ({vel.x}, {vel.y})")
        if check_angle:
            if not math.isfinite(body.angle):
                issues.append(f"{label}: NON-FINITE angle")
            spin = body.angular_velocity
            if not math.isfinite(spin):
                issues.append(f"{label}: NON-FINITE angular velocity")
            limit = (
                max_angular_velocity
                if max_angular_velocity is not None
                else self._max_body_angular_velocity
            )
            if math.isfinite(spin) and abs(spin) > limit:
                issues.append(f"{label}: extreme spin {spin:.2f} rad/s")
        return issues

    def _collect_speed_warnings(self, label: str, body) -> list[str]:
        """High speeds are logged but do not halt — transient spikes are not always fatal."""
        speed = self._speed_px_s(body)
        if math.isfinite(speed) and speed > self._max_speed_px_s:
            return [f"{label}: high speed {speed:.0f} px/s"]
        return []

    def _physics_halt_issues(self) -> list[str]:
        if self.body is None:
            return []
        issues = self._collect_halt_issues(
            "body",
            self.body,
            check_angle=True,
            max_angular_velocity=self._max_body_angular_velocity,
        )
        for i, (foot_body, _) in enumerate(self.feet):
            leg_num = i + 1
            grip = "GRIPPED" if self.active_grips[i] is not None else "FREE"
            issues.extend(
                self._collect_halt_issues(f"leg {leg_num} foot ({grip})", foot_body)
            )
        for i, (leg_body, _) in enumerate(self.legs):
            leg_num = i + 1
            grip = "GRIPPED" if self.active_grips[i] is not None else "FREE"
            issues.extend(
                self._collect_halt_issues(f"leg {leg_num} segment ({grip})", leg_body)
            )
        return issues

    def _physics_speed_warnings(self) -> list[str]:
        if self.body is None:
            return []
        warnings = self._collect_speed_warnings("body", self.body)
        for i, (foot_body, _) in enumerate(self.feet):
            leg_num = i + 1
            grip = "GRIPPED" if self.active_grips[i] is not None else "FREE"
            warnings.extend(
                self._collect_speed_warnings(f"leg {leg_num} foot ({grip})", foot_body)
            )
        for i, (leg_body, _) in enumerate(self.legs):
            leg_num = i + 1
            grip = "GRIPPED" if self.active_grips[i] is not None else "FREE"
            warnings.extend(
                self._collect_speed_warnings(f"leg {leg_num} segment ({grip})", leg_body)
            )
        return warnings

    def _physics_issues(self) -> list[str]:
        return self._physics_halt_issues() + self._physics_speed_warnings()

    def _physics_is_valid(self) -> bool:
        return not self._physics_halt_issues()

    def _format_grid_pos(self, pos) -> str:
        if self.coordinator is None:
            return f"({pos.x:.1f},{pos.y:.1f})px"
        gx, gy = self.coordinator.screen_to_grid(Vec2d(pos.x, pos.y))
        return f"({gx:.1f},{gy:.1f})"

    def _format_speed(self, speed_px_s: float) -> str:
        if self.coordinator is None or self.coordinator.cell_size <= 0:
            return f"{speed_px_s:.0f} px/s"
        cells_s = speed_px_s / self.coordinator.cell_size
        return f"{speed_px_s:.0f} px/s ({cells_s:.1f} cells/s)"

    def _leg_spring_length(self, limb_id: int) -> tuple[float, float]:
        spring = self.leg_springs[limb_id]
        actual = (spring.a.position - spring.b.position).length
        return actual, spring.rest_length

    def _print_halt_diagnostics(self, reason: str) -> None:
        print(f"\n=== Simulation halt diagnostics ===", flush=True)
        print(f"reason: {reason}", flush=True)
        print(f"frame: {self.t}", flush=True)
        if runtime_debug.procedure_status:
            print(f"procedure: {runtime_debug.procedure_status}", flush=True)

        issues = self._physics_halt_issues()
        if issues:
            print("halt triggers:", flush=True)
            for issue in issues:
                print(f"  * {issue}", flush=True)
        else:
            print("halt triggers: (none — check warnings below)", flush=True)

        warnings = self._physics_speed_warnings()
        if warnings:
            print("speed warnings (non-fatal):", flush=True)
            for warning in warnings:
                print(f"  * {warning}", flush=True)

        if self.body is None:
            return

        body_speed = self._speed_px_s(self.body)
        print(
            f"body: pos={self._format_grid_pos(self.body.position)} "
            f"vel={self._format_speed(body_speed)} "
            f"spin={self.body.angular_velocity:.3f} rad/s "
            f"angle={math.degrees(self.body.angle):.1f}°",
            flush=True,
        )

        for i, (foot_body, _) in enumerate(self.feet):
            leg_num = i + 1
            foot_speed = self._speed_px_s(foot_body)
            leg_body, _ = self.legs[i]
            leg_speed = self._speed_px_s(leg_body)
            actual_len, rest_len = self._leg_spring_length(i)
            cell = self.coordinator.cell_size if self.coordinator else 1.0
            rot_cmd = ext_cmd = "?"
            if self._last_signal is not None:
                rot_cmd = f"{self._last_signal.rotation[i]:+.3f}"
                ext_cmd = f"{self._last_signal.extension[i]:+.3f}"
            motor_rate = self.rotation_motors[i].desired_rate
            ext_rate = self.spring_motors[i].extension_speed
            grip = "GRIPPED" if self.active_grips[i] is not None else "no grip"
            print(
                f"leg {leg_num} [{grip}]: "
                f"foot {self._format_grid_pos(foot_body.position)} "
                f"foot_vel={self._format_speed(foot_speed)} | "
                f"leg_segment_vel={self._format_speed(leg_speed)} | "
                f"spring {actual_len / cell:.2f}/{rest_len / cell:.2f} cells "
                f"(actual/rest) | "
                f"cmd rot={rot_cmd} ext={ext_cmd} | "
                f"motor rot={motor_rate:+.3f} ext_rate={ext_rate:+.3f}",
                flush=True,
            )

        if self._last_signal is not None:
            sig = self._last_signal
            print(
                f"last signal: grip={sig.grip} damp_body={sig.damp_body} "
                f"relax_spring={sig.relax_spring}",
                flush=True,
            )
        print("=== end diagnostics ===\n", flush=True)

    def _damp_body_motion(
        self,
        *,
        angular_factor: float = 0.85,
        linear_factor: float = 0.95,
        free_foot_linear_factor: float = 0.80,
    ):
        self.body.angular_velocity *= angular_factor
        self.body.velocity = self.body.velocity * linear_factor
        for leg_body, _ in self.legs:
            leg_body.angular_velocity *= angular_factor
        for i, (foot_body, _) in enumerate(self.feet):
            foot_body.angular_velocity *= angular_factor
            foot_factor = (
                free_foot_linear_factor
                if self.active_grips[i] is None
                else linear_factor
            )
            foot_body.velocity = foot_body.velocity * foot_factor

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
        m = self.rotation_motors[i]
        m.desired_rate = v

    def set_extension_speed(self, i, v):
        self.spring_motors[i].set_extension_speed(v)


    def apply_signal(self, signal:ControlSignal):
        """
        Apply control signal by setting angular and linear velocities,
        not by directly modifying positions.
        """
        self._last_signal = signal
        lock_commands = signal.angle_lock if signal.angle_lock else None
        relax_commands = signal.relax_spring if signal.relax_spring else None
        for i, _ in enumerate(self.legs):
            if relax_commands is not None and relax_commands[i]:
                self.spring_motors[i].relax_to_current()
            if signal.grip[i] == 1:
                self.check_and_grip(i)
            if signal.grip[i] == -1:
                self.release_grip(i)
            if lock_commands is not None:
                if lock_commands[i]:
                    self.lock_body_leg_angle(i)
                else:
                    self.unlock_body_leg_angle(i)
            if self.is_body_leg_angle_locked(i):
                self.set_rotation_speed(i, 0)
            else:
                self.set_rotation_speed(i, signal.rotation[i])
            self.set_extension_speed(i, signal.extension[i])

        if signal.damp_body:
            self._damp_body_motion()


    def gone_wrong(self):
        if not self._physics_is_valid():
            return True
        return (self.body.position[0] < 0 or self.body.position[1] < 0 or
                self.body.position[0] > self.sim_config.screen_width or
                self.body.position[1] > self.sim_config.screen_height)

    def _enforce_gripped_min_extension(self) -> None:
        """Keep gripped legs from compressing below min_extension in physics."""
        if self.coordinator is None or self.coordinator.robot_config.disable_assist:
            return
        min_ext = self.coordinator.min_extension
        body_pos = self.body.position
        for i, grip in enumerate(self.active_grips):
            if grip is None:
                continue
            foot_pos = self.feet[i][0].position
            delta = foot_pos - body_pos
            length = delta.length
            if length < min_ext and length > 1e-9:
                delta_hat = delta / length
                self.body.position = foot_pos - delta_hat * min_ext

    def step(self):
        if not self.run or self._halted:
            return
        if not self._physics_is_valid():
            self._halt_simulation("Simulation unstable (invalid physics state).")
            return

        for m in self.spring_motors:
            m.step(self.sim_config.dt)
        for m in self.rotation_motors:
            m.step(self.sim_config.dt)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._halt_simulation("Simulation window closed.")
                return
            if self.gone_wrong():
                self._halt_simulation("Simulation left the visible area.")
                return

        self.space.step(self.sim_config.dt)

        if not self._physics_is_valid():
            self._halt_simulation("Simulation became unstable after physics step.")
            return

        self._enforce_gripped_min_extension()

        if not self._physics_is_valid():
            self._halt_simulation("Simulation became unstable after min-extension enforce.")
            return

        if self.writer:
            frame = pygame.surfarray.array3d(pygame.display.get_surface())
            frame = frame.swapaxes(0, 1)
            self.writer.append_data(frame)
        self.t += 1
        self.draw()
        if getattr(self.sim_config, "realtime", True):
            self.clock.tick(self.sim_config.fps)

    def pump_display(self, *, fps: int = 30) -> bool:
        """Redraw and process window events without advancing physics."""
        if not self.run or self._halted or not self._display_active:
            return False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._halt_simulation("Simulation window closed.")
                return False
        self.draw()
        self.clock.tick(fps)
        return True

    def get_state(self):
        return StateSignal(
            center_pos=self.get_center_pos(),
            feet_pos=[self.get_foot_pos(i) for i, _ in enumerate(self.feet)],
            active_grips=[a is not None for a in self.active_grips],
            t=self.t,
            body_angle=self.body.angle,
            body_angular_velocity=self.body.angular_velocity,
        )
