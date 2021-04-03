import action
import camera
import debug
import entities
import json
import pose
import pygame

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

        self._carrier = entities.Carrier()
        self._robots = {i + 1: entities.Robot() for i in range(simulation_settings["n_robots"])}
        self._trash = [entities.Trash("bottle", 0.99) for _ in range(1)]
        self._obstacles = []

        self._update_plan_timer = _IntervalTimer(interval_settings["update_plan"], self._update_plan)
        self._sync_poses_timer = _IntervalTimer(interval_settings["sync_poses"], self._sync_poses)
        self._sync_robots_timer = _IntervalTimer(interval_settings["sync_robots"], self._sync_robots)

        self._actions = {}

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
            self._actions[id] = action.create(command)

    def _sync_poses(self, planner):
        poses = {}
        for id, robot in self._robots.items():
            pose_relative = pose.relative_to(self._carrier.pose, robot.pose)
            pose_tuple = (
                pose_relative.x,
                pose_relative.y,
                pose_relative.a
            )
            poses[id] = pose_tuple

        planner.sync_poses(poses)

    def _sync_robots(self, planner):
        discoveries = {}

        for id, robot in self._robots.items():
            terrain_seen = []
            # TODO: Implement true LIDAR visibility shape


            litter_seen = []
            for item in self._trash:
                # TODO: Implement true camera visibility shape
                if robot.pose.distance(item.pose.position) < 1000:
                    pose_relative = pose.relative_to(robot.pose, item.pose)
                    litter_seen.append(
                        (
                            pose_relative.position,
                            item.type,
                            item.certainty
                        )
                    )

            discoveries[id] = litter_seen

        reports = {}

        for id, action in self._actions.items():
            reports[id] = (action.done, action.failed, action.payload)

        for id in self._robots:
            trash = discoveries[id]

            try: report = reports[id]
            except KeyError: report = None

            planner.sync_robot(id, map, trash, report)

    # Run
    def run(self, planner):
        for id, robot in self._robots.items():
            robot.pose.x = 2000 * id
            robot.pose.a = 90

        for trash in self._trash:
            trash.pose = pose.Pose2D(6000, 5000, 0)

        self._sync_poses(planner)

        while True:
            self._events.process(self)

            dt = self._clock.tick(60)

            if not self._paused:
                #self._carrier.pose.y += 0.89408 * dt

                self._update_plan_timer.tick(dt, planner)

                finished = []
                for id, action in self._actions.items():
                    robot = self._robots[id]
                    action.run(dt, robot)

                    if action.done: finished.append(id)

                for id in finished: del self._actions[id]

                self._sync_poses_timer.tick(dt, planner)
                self._sync_robots_timer.tick(dt, planner)

            self._camera.render(self._carrier, self._robots, self._trash)
            self._camera.debug(planner)

            pygame.display.flip()
