import argparse
import math
import random
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import pygame

WIDTH, HEIGHT = 960, 640
BACKGROUND_COLOR = (16, 18, 28)
BOID_COLOR = (235, 240, 255)
GOAL_COLOR = (255, 180, 90)
TEXT_COLOR = (210, 220, 240)


@dataclass
class Vector2:
    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vector2":
        return Vector2(self.x * scalar, self.y * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> "Vector2":
        if scalar == 0:
            return Vector2(self.x, self.y)
        return Vector2(self.x / scalar, self.y / scalar)

    def magnitude(self) -> float:
        return math.hypot(self.x, self.y)

    def normalize(self) -> "Vector2":
        magnitude = self.magnitude()
        if magnitude == 0:
            return Vector2()
        return self / magnitude

    def limit(self, maximum: float) -> "Vector2":
        magnitude = self.magnitude()
        if magnitude > maximum:
            return self.normalize() * maximum
        return Vector2(self.x, self.y)

    def distance_to(self, other: "Vector2") -> float:
        return (self - other).magnitude()

    def as_int_tuple(self) -> tuple[int, int]:
        return int(self.x), int(self.y)


class Boid:
    def __init__(
        self,
        width: int,
        height: int,
        max_speed: float,
        max_force: float,
        neighbor_radius: float,
        separation_radius: float,
    ) -> None:
        self.bounds_width = width
        self.bounds_height = height
        self.position = Vector2(random.uniform(0, width), random.uniform(0, height))
        angle = random.uniform(0, math.tau)
        speed = random.uniform(max_speed * 0.4, max_speed)
        self.velocity = Vector2(math.cos(angle), math.sin(angle)) * speed
        self.acceleration = Vector2()
        self.max_speed = max_speed
        self.max_force = max_force
        self.neighbor_radius = neighbor_radius
        self.separation_radius = separation_radius
        self.size = 8

    def apply_force(self, force: Vector2) -> None:
        self.acceleration = self.acceleration + force

    def flock(
        self,
        boids: list["Boid"],
        separation_weight: float,
        alignment_weight: float,
        cohesion_weight: float,
        goal: Vector2 | None = None,
        goal_weight: float = 0.0,
    ) -> None:
        separation = self.separation(boids) * separation_weight
        alignment = self.alignment(boids) * alignment_weight
        cohesion = self.cohesion(boids) * cohesion_weight

        self.apply_force(separation)
        self.apply_force(alignment)
        self.apply_force(cohesion)

        if goal is not None and goal_weight > 0:
            self.apply_force(self.seek(goal) * goal_weight)

    def separation(self, boids: list["Boid"]) -> Vector2:
        steering = Vector2()
        count = 0

        for other in boids:
            if other is self:
                continue
            distance = self.position.distance_to(other.position)
            if 0 < distance < self.separation_radius:
                offset = (self.position - other.position).normalize()
                steering = steering + (offset / max(distance, 0.001))
                count += 1

        if count == 0:
            return Vector2()

        steering = (steering / count).normalize() * self.max_speed
        steering = (steering - self.velocity).limit(self.max_force)
        return steering

    def alignment(self, boids: list["Boid"]) -> Vector2:
        average_velocity = Vector2()
        count = 0

        for other in boids:
            if other is self:
                continue
            if self.position.distance_to(other.position) < self.neighbor_radius:
                average_velocity = average_velocity + other.velocity
                count += 1

        if count == 0:
            return Vector2()

        average_velocity = (average_velocity / count).normalize() * self.max_speed
        return (average_velocity - self.velocity).limit(self.max_force)

    def cohesion(self, boids: list["Boid"]) -> Vector2:
        center = Vector2()
        count = 0

        for other in boids:
            if other is self:
                continue
            if self.position.distance_to(other.position) < self.neighbor_radius:
                center = center + other.position
                count += 1

        if count == 0:
            return Vector2()

        return self.seek(center / count)

    def seek(self, target: Vector2) -> Vector2:
        desired = (target - self.position).normalize() * self.max_speed
        return (desired - self.velocity).limit(self.max_force)

    def update(self) -> None:
        self.velocity = (self.velocity + self.acceleration).limit(self.max_speed)
        self.position = self.position + self.velocity
        self.acceleration = Vector2()
        self.wrap_around_screen()

    def wrap_around_screen(self) -> None:
        if self.position.x < 0:
            self.position.x = self.bounds_width
        elif self.position.x > self.bounds_width:
            self.position.x = 0

        if self.position.y < 0:
            self.position.y = self.bounds_height
        elif self.position.y > self.bounds_height:
            self.position.y = 0

    def draw(self, screen: Any, color: tuple[int, int, int]) -> None:
        import pygame

        angle = math.atan2(self.velocity.y, self.velocity.x)
        forward = Vector2(math.cos(angle), math.sin(angle))
        right = Vector2(-math.sin(angle), math.cos(angle))

        tip = self.position + forward * self.size
        rear_left = self.position - forward * (self.size * 0.7) + right * (self.size * 0.45)
        rear_right = self.position - forward * (self.size * 0.7) - right * (self.size * 0.45)

        pygame.draw.polygon(
            screen,
            color,
            [tip.as_int_tuple(), rear_left.as_int_tuple(), rear_right.as_int_tuple()],
        )


class Simulation:
    def __init__(self, args: argparse.Namespace) -> None:
        import pygame

        self.pygame = pygame
        pygame.init()
        pygame.display.set_caption("Boids Flocking Simulation")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)

        self.show_hud = True
        self.goal_point: Vector2 | None = None
        self.goal_weight = args.goal_weight
        self.separation_weight = args.separation_weight
        self.alignment_weight = args.alignment_weight
        self.cohesion_weight = args.cohesion_weight

        self.boids = [
            Boid(
                WIDTH,
                HEIGHT,
                max_speed=args.max_speed,
                max_force=args.max_force,
                neighbor_radius=args.neighbor_radius,
                separation_radius=args.separation_radius,
            )
            for _ in range(args.num_boids)
        ]

    def run(self) -> None:
        running = True
        while running:
            self.clock.tick(60)
            pygame = self.pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_h:
                        self.show_hud = not self.show_hud
                    elif event.key == pygame.K_c:
                        self.goal_point = None
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self.goal_point = Vector2(*event.pos)

            self.screen.fill(BACKGROUND_COLOR)
            for boid in self.boids:
                boid.flock(
                    self.boids,
                    separation_weight=self.separation_weight,
                    alignment_weight=self.alignment_weight,
                    cohesion_weight=self.cohesion_weight,
                    goal=self.goal_point,
                    goal_weight=self.goal_weight,
                )
                boid.update()
                boid.draw(self.screen, BOID_COLOR)

            if self.goal_point is not None:
                pygame.draw.circle(self.screen, GOAL_COLOR, self.goal_point.as_int_tuple(), 7, width=2)

            if self.show_hud:
                self.draw_hud()

            pygame.display.flip()

        pygame.quit()

    def draw_hud(self) -> None:
        messages = [
            f"boids: {len(self.boids)}",
            f"weights sep/aln/coh: {self.separation_weight:.2f}/{self.alignment_weight:.2f}/{self.cohesion_weight:.2f}",
            "left click: set goal point",
            "C: clear goal   H: toggle HUD",
        ]

        for index, message in enumerate(messages):
            surface = self.font.render(message, True, TEXT_COLOR)
            self.screen.blit(surface, (12, 12 + index * 22))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2D boids flocking simulation with pygame.")
    parser.add_argument("--num-boids", type=int, default=40, help="Number of boids to simulate.")
    parser.add_argument("--neighbor-radius", type=float, default=55.0, help="Radius for alignment and cohesion.")
    parser.add_argument("--separation-radius", type=float, default=22.0, help="Radius for close-range separation.")
    parser.add_argument("--max-speed", type=float, default=4.0, help="Maximum boid speed.")
    parser.add_argument("--max-force", type=float, default=0.09, help="Maximum steering force.")
    parser.add_argument("--separation-weight", type=float, default=1.6, help="Weight for separation steering.")
    parser.add_argument("--alignment-weight", type=float, default=1.0, help="Weight for alignment steering.")
    parser.add_argument("--cohesion-weight", type=float, default=1.0, help="Weight for cohesion steering.")
    parser.add_argument("--goal-weight", type=float, default=0.35, help="Weight for optional mouse goal seeking.")
    return parser.parse_args()


if __name__ == "__main__":
    Simulation(parse_args()).run()
