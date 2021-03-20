import math
import pygame

from sim import colors
from sim import pose

class Camera:
    def __init__(self, surface):
        self._surface = surface
        self._height = surface.get_height()

        self._robot_image = pygame.image.load("husky.png")
        # TODO: Remove this by editing the image
        self._robot_image = pygame.transform.rotate(self._robot_image, -90)

        self._pan_start = None
        self._offset = pygame.Vector2(0, 0)

        self._scale = 1
        self._scale_bounds = (0.5, 5)
        self._rescale_image = False
        self._zoomed_image = self._robot_image.copy()

        self._zero = pose.Pose2D(0, 0, 0)

        pygame.init()

    # Event Handlers
    def on_click(self, event):
        if event.button == 1:
            self._pan_start = event.pos

    def on_drag(self, event):
        if self._pan_start is not None:
            self._offset.x += (event.pos[0] - self._pan_start[0])
            self._offset.y -= (event.pos[1] - self._pan_start[1])
            self._pan_start = event.pos

    def on_release(self, event):
        if event.button == 1:
            self._pan_start = None

    def on_scroll(self, event):
        self._scale += event.y * 0.1
        if self._scale < self._scale_bounds[0]: self._scale = self._scale_bounds[0]
        elif self._scale_bounds[1] < self._scale: self._scale = self._scale_bounds[1]

        self._rescale_image = True

    # Rendering
    def _get_render_position(self, world_position):
        return (
            self._scale * world_position[0] + self._offset.x,
            self._height - self._scale * world_position[1] - self._offset.y
        )

    def _draw_axes(self, pose, length = 25, thickness = 2):
        rad = math.pi * pose.theta / 180
        rsin = length * math.sin(rad); rcos = length * math.cos(rad)
        px, py = self._get_render_position(pose.position)

        pygame.draw.line(self._surface, colors.DodgerBlue, (px, py), (px + rcos, py - rsin), thickness)
        pygame.draw.line(self._surface, colors.Firebrick, (px, py), (px - rsin, py - rcos), thickness)

    def render(self, planner, fleet):
        self._surface.fill(colors.White)

        if self._rescale_image:
            new_width = int(self._robot_image.get_width() * self._scale)
            new_height = int(self._robot_image.get_height() * self._scale)
            self._zoomed_image = pygame.transform.scale(self._robot_image, (new_width, new_height))
            self._rescale_image = False

        for id, pose in fleet.poses.items():
            image = pygame.transform.rotate(self._zoomed_image, pose.theta)
            rect = image.get_rect(center = self._get_render_position(pose.position))
            self._surface.blit(image, rect)

            self._draw_axes(pose)

            try: waypoints = fleet.trajectories[id]
            except KeyError: continue

            previous = rect.center
            for wp in waypoints:
                next = self._get_render_position(wp)
                pygame.draw.line(self._surface, (63, 72, 204), previous, next, 2)
                pygame.draw.circle(self._surface, (63, 72, 204), previous, 5)
                previous = next

        self._draw_axes(self._zero, length = 75, thickness = 5)

        pygame.display.flip()