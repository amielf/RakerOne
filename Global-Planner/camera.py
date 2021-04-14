import colors
import math
import pygame
import tf
import util

# The dimensions of the Clearpath Husky are 660x990 mm
# The image is scaled to be 66x99 pixels, so at 1.0x zoom, one pixel covers 10mm
MM_PER_PIXEL = 10

class Camera:
    def __init__(self, surface, events):
        self._surface = surface

        events.handle(pygame.MOUSEBUTTONDOWN, self._on_click)
        events.handle(pygame.MOUSEMOTION, self._on_drag)
        events.handle(pygame.MOUSEBUTTONUP, self._on_release)
        events.handle(pygame.MOUSEWHEEL, self._on_scroll)
        events.handle(pygame.KEYDOWN, self._on_keydown)

        self._last_mouse_position = None
        self._offset = pygame.Vector2(0, self._surface.get_height())

        self._zoom = 1
        self._zoom_bounds = (0.1, 2)

        self._visible_world_rect = None

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

        self._pause_image = pygame.image.load("pause.png")
        self._pause_image = pygame.transform.scale(self._pause_image, (64, 64))

        self._show_lidar = False
        self._show_yolo = False

        self._help_lines = [
            "Simulation:",
            "P - Toggle Pause",
            "",
            "Visualization:",
            "L - Toggle LIDAR visibility",
            "Y - Toggle YOLO visibility"
        ]
        self._help_lines.reverse()

    # Event Handlers
    def _on_click(self, event):
        if event.button == 1:
            self._last_mouse_position = event.pos

    def _on_drag(self, event):
        if self._last_mouse_position is not None:
            self._offset.x -= (event.pos[0] - self._last_mouse_position[0])
            self._offset.y -= (event.pos[1] - self._last_mouse_position[1])

            self._last_mouse_position = event.pos
            self._visible_world_rect = None

    def _on_release(self, event):
        if event.button == 1:
            self._last_mouse_position = None

    def _on_scroll(self, event):
        self._zoom += math.copysign(0.1, event.y)
        if self._zoom < self._zoom_bounds[0]: self._zoom = self._zoom_bounds[0]
        elif self._zoom_bounds[1] < self._zoom: self._zoom = self._zoom_bounds[1]

        self._visible_world_rect = None

    def _on_keydown(self, event):
        if event.key == pygame.K_l: self._show_lidar = not self._show_lidar
        if event.key == pygame.K_y: self._show_yolo = not self._show_yolo

    # Drawing
    def _draw_world(self, world):
        row_min = max(self._visible_world_rect.top // world.resolution, 0)
        row_max = math.ceil(self._visible_world_rect.bottom / world.resolution)

        col_min = max(self._visible_world_rect.left // world.resolution, 0)
        col_max = math.ceil(self._visible_world_rect.right / world.resolution)

        for r in range(row_min, row_max):
            y = util.clip(0, r * world.resolution, world.height)

            x_min = util.clip(0, self._visible_world_rect.left, world.width)
            x_max = util.clip(0, self._visible_world_rect.right, world.width)

            start = self._get_render_position((x_min, y))
            end = self._get_render_position((x_max, y))
            pygame.draw.line(self._surface, colors.LightGray, start, end, 1)

        for c in range(col_min, col_max):
            x = util.clip(0, c * world.resolution, world.width)

            # This looks backwards but that's because the rect is defined with +y going down
            y_min = util.clip(0, self._visible_world_rect.top, world.height)
            y_max = util.clip(0, self._visible_world_rect.bottom, world.height)

            start = self._get_render_position((x, y_min))
            end = self._get_render_position((x, y_max))
            pygame.draw.line(self._surface, colors.LightGray, start, end, 1)

    def _draw_entity(self, entity, image, draw_frame = False, pose_offset = (0, 0)):
        footprint = pygame.Rect(
            0,
            0,
            entity.dimensions.x,
            entity.dimensions.y
        )
        footprint.center = entity.pose.position

        if not self._visible_world_rect.colliderect(footprint): return

        image = pygame.transform.scale(image, self._get_render_dimensions(entity.dimensions))
        if entity.pose.a != 0:
            image = pygame.transform.rotate(image, entity.pose.a)

        pose_render_position = self._get_render_position(entity.pose.position)
        image_render_position = self._get_render_position((entity.pose.x + pose_offset[0], entity.pose.y + pose_offset[1]))
        self._surface.blit(image, image.get_rect(center = image_render_position))

        if draw_frame:
            frame_image = self._frame_image

            if entity.pose.a != 0:
                frame_image = pygame.transform.rotate(self._frame_image, entity.pose.a)

            self._surface.blit(frame_image, frame_image.get_rect(center = pose_render_position))

    def _draw_planner_info(self, world, carrier, planner):
        # Map
        cell_render_size = self._get_render_dimensions((world.resolution, world.resolution))
        cell_rect = pygame.Rect(0, 0, *cell_render_size)

        for row, col in planner.map.area:
            cell_rect.bottomleft = self._get_render_position((col * world.resolution, row * world.resolution))
            pygame.draw.rect(self._surface, colors.LightGray, cell_rect, 1)

        for row, col in planner.map.frontier:
            cell_rect.bottomleft = self._get_render_position((col * world.resolution, row * world.resolution))
            pygame.draw.rect(self._surface, colors.Red, cell_rect, 1)

        # Tasks
        for location, type, _ in planner.tasks.unassigned_litter:
            render_position = self._get_render_position(location)
            pygame.draw.circle(self._surface, colors.DarkRed, render_position, 10, 2)

        # Lanes
        for low, high in planner.flow.spans:
            render_low, _ = self._get_render_position((low, 0))
            render_high, _ = self._get_render_position((high, 0))

            pygame.draw.line(self._surface, colors.Black, (render_low, self._surface.get_height()), (render_low, 0), 1)
            pygame.draw.line(self._surface, colors.Black, (render_high, self._surface.get_height()), (render_high, 0), 1)

        # Robots
        for id, robot in planner.robots.items():
            # The planner's robot pose is relative to the carrier; transform back to global coordinates to draw
            robot_pose_absolute = tf.absolute_pose(carrier.pose, robot.pose)
            if not self._visible_world_rect.collidepoint(robot_pose_absolute.x, robot_pose_absolute.y): continue

            robot_render_position = self._get_render_position(robot_pose_absolute.position)
            pygame.draw.circle(self._surface, colors.HotPink, robot_render_position, 3)

            if len(robot.todo) > 0:
                target_render_position = self._get_render_position(robot.todo[0].location)
                pygame.draw.line(self._surface, colors.LightSeaGreen, robot_render_position, target_render_position, 3)

    # Rendering
    def _get_render_position(self, world_position):
        relative_x = world_position[0] - self._visible_world_rect.left
        relative_y = world_position[1] - self._visible_world_rect.top

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

    def render(self, planner, world, carrier, huskies, litter, paused):
        self._surface.fill(colors.White)

        if self._visible_world_rect is None:
            surface_width, surface_height = self._surface.get_size()

            scale = MM_PER_PIXEL / self._zoom

            world_width, world_height = (
                surface_width * scale,
                surface_height * scale
            )

            self._visible_world_rect = pygame.Rect(0, 0, world_width, world_height)
            self._visible_world_rect.center = (
                self._offset.x * scale,
                (surface_height - self._offset.y) * scale
            )

        # World
        # self._draw_world(world)

        # Litter
        for trash in litter:
            self._draw_entity(trash, self._trash_image)

        # Huskies
        for id, husky in huskies.items():
            husky_image = self._husky_image.copy()

            text = self._font.render(str(id), True, colors.Gold)
            text = pygame.transform.rotate(text, -90)
            husky_image.blit(text, text.get_rect(center = self._husky_image.get_rect().center))

            self._draw_entity(husky, husky_image)

            husky_render_position = self._get_render_position(husky.pose.position)

            if self._show_lidar:
                lidar_radius, _ = self._get_render_dimensions((husky.lidar_range, 0))

                lidar_arc_points = []
                half_arc = husky.lidar_arc // 2

                for a in range(int(-half_arc + husky.pose.a), int(husky.pose.a + half_arc)):
                    rad = -math.radians(a)
                    lidar_arc_points.append((
                        husky_render_position[0] + lidar_radius * math.cos(rad),
                        husky_render_position[1] + lidar_radius * math.sin(rad)
                    ))

                lidar_arc_points.append(husky_render_position)

                pygame.draw.polygon(self._surface, (112, 146, 190), lidar_arc_points, 2)

            if self._show_yolo:
                yolo_radius, _ = self._get_render_dimensions((husky.yolo_range, 0))

                yolo_arc_points = []
                half_arc = husky.lidar_arc // 2

                for a in range(int(-half_arc + husky.pose.a), int(husky.pose.a + half_arc)):
                    rad = -math.radians(a)
                    yolo_arc_points.append((
                        husky_render_position[0] + yolo_radius * math.cos(rad),
                        husky_render_position[1] + yolo_radius * math.sin(rad)
                    ))

                yolo_arc_points.append(husky_render_position)

                pygame.draw.polygon(self._surface, (63, 72, 204), yolo_arc_points, 2)

        # Carrier
        # self._draw_entity(carrier, self._bus_image, draw_frame = True, pose_offset = (-carrier.dimensions[0] / 2, 0))

        self._draw_planner_info(world, carrier, planner)

        if paused:
            self._surface.blit(self._pause_image, self._pause_image.get_rect(topleft = (10, 10)))

        # Help Text
        y = self._surface.get_height()
        for line in self._help_lines:
            text = self._font.render(line, True, colors.Gray)
            self._surface.blit(text, text.get_rect(bottomleft = (20, y)))
            y -= text.get_height()