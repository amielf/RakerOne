import camera
import world
import debug
import entities
import json
import pose
import pygame
import random
import tf

class _IntervalTimer:
    def __init__(self, interval, func):
        self._interval = interval
        self._func = func
        self._countdown = 0

    def tick(self, dt, *args, **kwargs):
        self._countdown -= dt
        if self._countdown <= 0:
            self._func(*args, **kwargs)
            self._countdown = self._interval

class _EventPump:
    def __init__(self, owner):
        self._owner_id = id(owner)
        self._handlers = {}

    def handle(self, event, handler):
        try: self._handlers[event].append(handler)
        except KeyError: self._handlers[event] = [handler]

    def process(self, caller):
        if id(caller) != self._owner_id:
            raise Exception(f"Caller {caller} is not authorized to call {self.process}")

        for event in pygame.event.get():
            if event.type in self._handlers:
                for handler in self._handlers[event.type]:
                    handler(event)

class Simulation:
    def __init__(self, config_filename):
        pygame.init()

        with open(config_filename, "r") as file:
            config = json.load(file)

        simulation_settings = config["simulation_settings"]
        interval_settings = config["interval_settings"]
        world_settings = config["world_settings"]
        robot_settings = config["robot_settings"]

        self._world = world.World(
            world_settings["width_mm"],
            world_settings["height_mm"],
            robot_settings["lidar_resolution_mm"]
        )

        self._carrier = entities.Carrier(
            simulation_settings["carrier_speed_mph"] * 0.44704  # mph => mm per sec
        )

        self._robots = {
            i + 1: entities.Husky(
                robot_settings["lidar_range_mm"],
                robot_settings["yolo_range_mm"]
            )

            for i in range(simulation_settings["n_robots"])
        }

        self._litter = [
            entities.Trash("bottle", 0.99)
            for _ in range(100)
        ]

        self._update_plan_timer = _IntervalTimer(interval_settings["update_plan_interval_ms"], self._update_plan)
        self._sync_poses_timer = _IntervalTimer(interval_settings["sync_poses_interval_ms"], self._sync_poses)
        self._sync_robots_timer = _IntervalTimer(interval_settings["sync_robots_interval_ms"], self._sync_robots)

        self._events = _EventPump(self)
        self._events.handle(pygame.QUIT, self._on_quit)
        self._events.handle(pygame.KEYDOWN, self._on_keydown)

        self._clock = pygame.time.Clock()

        surface = pygame.display.set_mode((1200, 700))
        pygame.display.set_caption("Global Planner Simulator")
        self._camera = camera.Camera(surface, self._events)

        self._paused = False

    # Event Handlers
    def _on_quit(self, _):
        pygame.quit()
        quit(0)

    def _on_keydown(self, event):
        if event.key == pygame.K_p:
            self._paused = not self._paused

    # Interval Functions
    def _update_plan(self, planner):
        plan = planner.get()
        debug.log(f"Plan Updated: {plan}")

        for id, command in plan.items():
            self._robots[id].execute(command)

    def _sync_poses(self, planner):
        poses = {}
        for id, robot in self._robots.items():
            pose_relative_to_carrier = tf.relative(self._carrier.pose, robot.pose)
            poses[id] = pose_relative_to_carrier

        planner.sync_poses(poses)

    def _sync_robots(self, planner):
        for id, robot in self._robots.items():
            grid = robot.get_partial_grid(self._world)
            litter = robot.get_visible_litter(self._litter)

            report = None
            if robot.action is not None:
                report = (robot.action.done, robot.action.failed, robot.action.payload)

            planner.sync_robot(id, grid, litter, report)

    # Run
    def run(self, planner):
        for id, robot in self._robots.items():
            robot.pose.x = 2000 * id
            robot.pose.y = 2000
            robot.pose.a = 90

        padding = 1000
        for trash in self._litter:
            trash.pose = pose.Pose2D(
                random.randint(padding, self._world.width - padding),
                random.randint(padding, 19000),
                0
            )

        self._sync_poses(planner)

        while True:
            self._events.process(self)

            dt = self._clock.tick(60)

            if not self._paused:
                self._carrier.pose.y += self._carrier.speed * dt

                self._update_plan_timer.tick(dt, planner)

                for id, robot in self._robots.items():
                    robot.update(dt)

                self._sync_poses_timer.tick(dt, planner)
                self._sync_robots_timer.tick(dt, planner)

            # Visualize
            self._camera.render(
                planner,
                self._world,
                self._carrier,
                self._robots,
                self._litter,
                self._paused
            )

            pygame.display.flip()
