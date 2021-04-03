import colors
import math
import pygame

MM_PER_PIXEL = 10

class Camera:
    def __init__(self, surface, events):
        self._surface = surface

        events.handle(pygame.MOUSEBUTTONDOWN, self._on_click)
        events.handle(pygame.MOUSEMOTION, self._on_drag)
        events.handle(pygame.MOUSEBUTTONUP, self._on_release)
        events.handle(pygame.MOUSEWHEEL, self._on_scroll)

        self._last_mouse_position = None
        self._offset = pygame.Vector2(0, self._surface.get_height())

        self._zoom = 1
        self._zoom_bounds = (0.3, 2)

        self._world_rect = None

        self._bus_image = pygame.image.load("bus.png")
        self._husky_image = pygame.image.load("husky.png")

        self._trash_image = pygame.Surface((50, 50))
        self._trash_image.set_colorkey(colors.Black)
        pygame.draw.circle(self._trash_image, colors.OrangeRed, (25, 25), 25)

        self._frame_image = pygame.Surface((50, 50))
        self._frame_image.set_colorkey(colors.Black)
        pygame.draw.line(self._frame_image, colors.Firebrick, (25, 25), (50, 25), 2)
        pygame.draw.line(self._frame_image, colors.DodgerBlue, (25, 25), (25, 50), 2)
        self._frame_image = pygame.transform.flip(self._frame_image, False, True)

        self._font = pygame.font.SysFont("Calibri", 24)

    # Event Handlers
    def _on_click(self, event):
        if event.button == 1:
            self._last_mouse_position = event.pos

    def _on_drag(self, event):
        if self._last_mouse_position is not None:
            self._offset.x -= (event.pos[0] - self._last_mouse_position[0])
            self._offset.y -= (event.pos[1] - self._last_mouse_position[1])

            self._last_mouse_position = event.pos
            self._world_rect = None

    def _on_release(self, event):
        if event.button == 1:
            self._last_mouse_position = None

    def _on_scroll(self, event):
        self._zoom += math.copysign(0.1, event.y)
        if self._zoom < self._zoom_bounds[0]: self._zoom = self._zoom_bounds[0]
        elif self._zoom_bounds[1] < self._zoom: self._zoom = self._zoom_bounds[1]

        self._world_rect = None

    # Rendering
    def _get_render_position(self, world_position):
        relative_x = world_position[0] - self._world_rect.left
        relative_y = world_position[1] - self._world_rect.top
        render_position = (
            int(self._zoom * relative_x // MM_PER_PIXEL),
            self._surface.get_height() - int(self._zoom * relative_y // MM_PER_PIXEL)
        )
        return render_position

    def _get_render_dimensions(self, world_dimensions):
        render_dimensions = (
            int(self._zoom * world_dimensions[0] // MM_PER_PIXEL),
            int(self._zoom * world_dimensions[1] // MM_PER_PIXEL)
        )
        return render_dimensions

    def _draw_entity(self, entity, image, draw_frame = False):
        footprint = pygame.Rect(entity.pose.x, entity.pose.y, entity.dimensions.x, entity.dimensions.y)
        if not self._world_rect.colliderect(footprint): return

        image = pygame.transform.scale(image, self._get_render_dimensions(entity.dimensions))

        if entity.pose.a != 0:
            image = pygame.transform.rotate(image, entity.pose.a)

        render_position = self._get_render_position(entity.pose.position)

        self._surface.blit(image, image.get_rect(center = render_position))

        if draw_frame:
            frame_image = pygame.transform.rotate(self._frame_image, entity.pose.a)
            self._surface.blit(frame_image, frame_image.get_rect(center = render_position))

    def render(self, carrier, robots, trash):
        self._surface.fill(colors.White)

        if self._world_rect is None:
            surface_width, surface_height = self._surface.get_size()

            world_width, world_height = (
                surface_width * MM_PER_PIXEL / self._zoom,
                surface_height * MM_PER_PIXEL / self._zoom
            )

            world_center = (
                self._offset.x * MM_PER_PIXEL / self._zoom,
                (surface_height - self._offset.y) * MM_PER_PIXEL / self._zoom
            )

            self._world_rect = pygame.Rect(
                world_center[0] - world_width / 2,
                world_center[1] - world_height / 2,
                world_width,
                world_height
            )

        self._draw_entity(carrier, self._bus_image, draw_frame = True)

        for id, robot in robots.items():
            image = self._husky_image.copy()

            text = self._font.render(str(id), True, colors.Gold)
            text = pygame.transform.rotate(text, -90)
            image.blit(text, text.get_rect(center = self._husky_image.get_rect().center))

            self._draw_entity(robot, image)

            render_position = self._get_render_position(robot.pose.position)
            radius, _ = self._get_render_dimensions((1000, 0))
            pygame.draw.circle(self._surface, (63, 72, 204), render_position, radius, 2)

        for item in trash:
            self._draw_entity(item, self._trash_image)

    def debug(self, planner):
        for id, pose in planner.poses.items():
            if not self._world_rect.collidepoint(pose[0], pose[1]): continue

            pose_rp = self._get_render_position(pose)
            pygame.draw.circle(self._surface, colors.Pink, pose_rp, 3)
