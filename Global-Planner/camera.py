import colors
import math
import pygame

import common
import tasks
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

        self._frame_image = pygame.Surface((50, 50))
        self._frame_image.set_colorkey(colors.Black)
        pygame.draw.line(self._frame_image, colors.Firebrick, (25, 25), (50, 25), 2)
        pygame.draw.line(self._frame_image, colors.DodgerBlue, (25, 25), (25, 50), 2)
        self._frame_image = pygame.transform.flip(self._frame_image, False, True)

        self._large_font = pygame.font.SysFont("Calibri", 24)
        self._small_font = pygame.font.SysFont("Calibri", 12)

        self._pause_image = pygame.image.load("pause.png")
        self._pause_image = pygame.transform.scale(self._pause_image, (64, 64))

        self._show_frames = False
        self._show_lidar = False
        self._show_tasks = False
        self._show_poses = False
        self._show_yolo = False

        self._help_lines = [
            "Simulation:",
            "P - Toggle Pause",
            "",
            "Visualization:",
            "F - Toggle frame visibility",
            "L - Toggle LIDAR visibility",
            "T - Toggle tasks visibility",
            "W - Toggle poses visibility",
            "Y - Toggle YOLO visibility",
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
        if event.key == pygame.K_f: self._show_frames = not self._show_frames
        if event.key == pygame.K_l: self._show_lidar = not self._show_lidar
        if event.key == pygame.K_t: self._show_tasks = not self._show_tasks
        if event.key == pygame.K_w: self._show_poses = not self._show_poses
        if event.key == pygame.K_y: self._show_yolo = not self._show_yolo

    # Drawing
    def _draw_entity(self, entity, image, draw_frame = False, pose_offset = (0, 0)):
        footprint = pygame.Rect(
            0,
            0,
            entity.dimensions.x,
            entity.dimensions.y
        )
        footprint.center = (entity.pose.x, entity.pose.y)

        if not self._visible_world_rect.colliderect(footprint): return

        image = pygame.transform.scale(image, self._get_render_dimensions(entity.dimensions))
        if entity.pose.a != 0:
            image = pygame.transform.rotate(image, entity.pose.a)

        pose_render_position = self._get_render_position(entity.pose.location)
        image_render_position = self._get_render_position(
            common.Location(
                entity.pose.x + pose_offset[0],
                entity.pose.y + pose_offset[1]
            )
        )
        self._surface.blit(image, image.get_rect(center = image_render_position))

        if draw_frame:
            frame_image = self._frame_image

            if entity.pose.a != 0:
                frame_image = pygame.transform.rotate(self._frame_image, entity.pose.a)

            self._surface.blit(frame_image, frame_image.get_rect(center = pose_render_position))

    def _draw_planner_info(self, origin, planner):
        # Map
        cell_render_size = self._get_render_dimensions((planner.map.resolution, planner.map.resolution))
        cell_rect = pygame.Rect(0, 0, *cell_render_size)

        for row, col in planner.map.grid:
            center = origin.get_absolute(planner.map.get_center_location(row, col))
            cell_rect.center = self._get_render_position(center)
            pygame.draw.rect(self._surface, colors.LightGray, cell_rect, 1)

            # text = self._small_font.render(f"({row}, {col})", True, colors.DarkGray)
            # self._surface.blit(text, text.get_rect(center = cell_rect.center))

        for row, col in planner.map.frontier:
            center = origin.get_absolute(planner.map.get_center_location(row, col))
            cell_rect.center = self._get_render_position(center)
            pygame.draw.rect(self._surface, colors.Red, cell_rect, 1)

        # Tasks
        for task in planner.tasks.retrieval_task_queue:
            location_absolute = origin.get_absolute(task.location)
            render_position = self._get_render_position(location_absolute)
            pygame.draw.circle(self._surface, colors.LightGray, render_position, 10, 2)

        # Lanes
        for low, high in planner.flow.lanes:
            render_low, _ = self._get_render_position(common.Location(low, 0))
            render_high, _ = self._get_render_position(common.Location(high, 0))

            pygame.draw.line(self._surface, colors.Black, (render_low, self._surface.get_height()), (render_low, 0), 1)
            pygame.draw.line(self._surface, colors.Black, (render_high, self._surface.get_height()), (render_high, 0), 1)

        # Robots
        for id, robot in planner.robots.items():
            for task in robot.todo:
                if not isinstance(task, tasks.Retrieve): continue

                location_absolute = origin.get_absolute(task.location)
                render_position = self._get_render_position(location_absolute)
                pygame.draw.circle(self._surface, colors.Gold, render_position, 10, 2)

            # The planner's robot pose is relative to the carrier; transform back to global coordinates to draw
            robot_pose_absolute = origin.get_absolute(robot.pose)
            if not self._visible_world_rect.collidepoint(robot_pose_absolute.x, robot_pose_absolute.y): continue

            robot_render_position = self._get_render_position(robot_pose_absolute.location)

            if self._show_poses:
                pygame.draw.circle(self._surface, colors.HotPink, robot_render_position, 3)

            if self._show_tasks and len(robot.todo) > 0:
                task = robot.todo[0]

                color = colors.Black
                if isinstance(task, tasks.Service): color = colors.OrangeRed
                elif isinstance(task, tasks.Retrieve): color = colors.DarkGoldenrod
                elif isinstance(task, tasks.Explore): color = colors.LightSeaGreen

                location_absolute = origin.get_absolute(task.location)
                target_render_position = self._get_render_position(location_absolute)

                pygame.draw.line(self._surface, color, robot_render_position, target_render_position, 1)

    # Rendering
    def _get_render_position(self, world_position):
        relative_x = world_position.x - self._visible_world_rect.left
        relative_y = world_position.y - self._visible_world_rect.top

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

    def render(self, planner, carrier, huskies, litter, paused, run_time):
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

        # Litter
        for id, trash in litter.items():
            if not self._visible_world_rect.collidepoint((trash.pose.x, trash.pose.y)): continue

            render_position = self._get_render_position(trash.pose.location)
            radius, _ = self._get_render_dimensions((50, 0))

            color = colors.LightSteelBlue if trash.type == "glass_bottle" \
                    else colors.SaddleBrown if trash.type == "paper_bag" \
                    else colors.Orange if trash.type == "cig" \
                    else colors.Black

            pygame.draw.circle(self._surface, color, render_position, radius)

        # Huskies
        for id, husky in huskies.items():
            husky_image = self._husky_image.copy()

            # end_effector_image = self._end_effector_images[husky.end_effector]
            # husky_image.blit(end_effector_image, end_effector_image.get_rect(center = husky_image.get_rect().center))

            husky_center = self._husky_image.get_rect().center

            id_text = self._large_font.render(str(id), True, colors.WhiteSmoke)
            id_text = pygame.transform.rotate(id_text, -90)
            husky_image.blit(id_text, id_text.get_rect(center = husky_center))

            end_effector_text = self._large_font.render(husky.end_effector[0:2], False, colors.WhiteSmoke)
            end_effector_text = pygame.transform.rotate(end_effector_text, -90)
            husky_image.blit(end_effector_text, end_effector_text.get_rect(center = (husky_center[0] - 20, husky_center[1])))

            self._draw_entity(husky, husky_image, draw_frame = self._show_frames)

            husky_render_position = self._get_render_position(husky.pose.location)

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
                half_arc = husky.yolo_arc // 2

                for a in range(int(-half_arc + husky.pose.a), int(husky.pose.a + half_arc)):
                    rad = -math.radians(a)
                    yolo_arc_points.append((
                        husky_render_position[0] + yolo_radius * math.cos(rad),
                        husky_render_position[1] + yolo_radius * math.sin(rad)
                    ))

                yolo_arc_points.append(husky_render_position)

                pygame.draw.polygon(self._surface, (63, 72, 204), yolo_arc_points, 2)

        # Carrier
        self._draw_entity(carrier, self._bus_image, draw_frame = True, pose_offset = (-carrier.dimensions[0] / 2, 0))

        self._draw_planner_info(carrier.pose, planner)

        if paused:
            self._surface.blit(self._pause_image, self._pause_image.get_rect(topleft = (10, 10)))

        run_time_text = self._large_font.render(f"{run_time} ms", False, colors.DarkGray)
        self._surface.blit(run_time_text, run_time_text.get_rect(right = self._surface.get_width() - 20))

        # Help Text
        y = self._surface.get_height()
        for line in self._help_lines:
            id_text = self._small_font.render(line, True, colors.Gray)
            self._surface.blit(id_text, id_text.get_rect(bottomleft = (20, y - 20)))
            y -= id_text.get_height()