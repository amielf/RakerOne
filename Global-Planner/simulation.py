import actions
import camera
import debug
import entities
import grid
import json
import metrics
import pygame
import random
import time
import types

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
    def __init__(self, filename):
        pygame.init()

        with open(filename, "r") as file:
            config = json.load(file, object_hook = lambda d: types.SimpleNamespace(**d))

        self._run_time_ms = config.simulation.run_time_sec * 1000
        self._dt = config.simulation.step_delta_ms
        self._now = 0

        self._grid = grid.Grid(
            config.terrain.width_mm,
            config.terrain.height_mm,
            config.terrain.resolution_mm
        )

        # Interval Timers
        self._update_plan_timer = _IntervalTimer(
            config.simulation.update_plan_interval_ms,
            self._update_plan
        )

        self._sync_carrier_timer = _IntervalTimer(
            config.simulation.sync_carrier_interval_ms,
            self._sync_carrier
        )

        self._sync_poses_timer = _IntervalTimer(
            config.simulation.sync_poses_interval_ms,
            self._sync_poses
        )

        self._sync_robots_timer = _IntervalTimer(
            config.simulation.sync_robots_interval_ms,
            self._sync_robots
        )

        # Entities
        self._carrier = entities.Carrier(config.carrier.speed_mph * 0.44704)

        random.seed(1000)

        self._huskies = dict()
        for end_effector in config.robots.end_effectors:
            for id in end_effector.robots:
                self._huskies[id] = entities.Husky(
                    config.robots.full_charge_v,
                    config.robots.bin_capacity_mm3,
                    config.robots.lidar_range_m * 1000,
                    config.robots.lidar_arc_deg,
                    config.robots.yolo_range_ft * 304.8,
                    config.robots.yolo_arc_deg,
                    config.robots.linear_speed_mph * 0.44704,
                    config.robots.rotational_speed_dps,
                    end_effector.name
                )

                self._huskies[id].pose.x = 500 + 2000 * (id - 1)
                self._huskies[id].pose.y = 500
                self._huskies[id].pose.a = 90

        self._litter = dict()
        self._collected_litter = set()

        hat = []
        for trash_spec in config.litter.distribution:
            hat.extend(trash_spec for _ in range(trash_spec.percentage))

        assert len(hat) == 100, f"Litter distribution percentages total {len(hat)}."

        padding = config.terrain.resolution_mm

        for i in range(config.litter.total):
            trash_spec = random.choice(hat)
            certainty = random.uniform(*trash_spec.certainty)

            trash = entities.Trash(
                i + 1,
                trash_spec.name,
                trash_spec.volume_mm3,
                certainty,
                trash_spec.end_effectors
            )

            trash.pose.x = random.randint(padding, config.terrain.width_mm - padding)
            trash.pose.y = random.randint(padding, config.terrain.height_mm - padding)

            self._litter[trash.id] = trash

        self._events = _EventPump(self)
        self._events.handle(pygame.QUIT, self._on_quit)
        self._events.handle(pygame.KEYDOWN, self._on_keydown)

        surface = pygame.display.set_mode((700, 1000))
        pygame.display.set_caption("Global Planner Simulator")
        self._camera = camera.Camera(surface, self._events)

        self._paused = True
        self._finished = False

    # Event Handlers
    def _on_quit(self, _):
        metrics.save()
        pygame.quit()
        quit(0)

    def _on_keydown(self, event):
        if event.key == pygame.K_p: self._paused = not self._paused

    # Interval Functions
    @debug.profiled
    def _sync_carrier(self, planner):
        planner.sync_carrier(*self._carrier.get_monitor_data())

    @debug.profiled
    def _sync_poses(self, planner):
        poses = {}
        for id, husky in self._huskies.items():
            pose_relative_to_carrier = husky.pose.relative_to(self._carrier.pose)
            poses[id] = pose_relative_to_carrier

        planner.sync_poses(poses)

    @debug.profiled
    def _sync_robots(self, planner):
        for id, husky in self._huskies.items():
            charge = 100 * (husky.charge / husky.full_charge)
            bin = 100 * (husky.bin / husky.bin_capacity)

            grid = husky.get_partial_grid(self._grid)

            litter = husky.get_visible_litter(self._litter)
            for trash in litter:
                trash_id = trash[0]
                if trash_id not in metrics.discovery_time_by_id:
                    trash = self._litter[trash_id]
                    metrics.discovery_time_by_id[trash_id] = (trash.type, trash.pose.location, self._now)

            next_task_please = len(husky.actions) == 0

            planner.sync_robot(id, charge, bin, husky.end_effector, grid, litter, next_task_please)

    @debug.profiled
    def _update_plan(self, planner):
        plan = planner.get()
        for id, commands in plan.items():
            self._huskies[id].execute(commands)

    # Run
    @debug.profiled
    def _step(self, planner):
        self._carrier.move(self._dt)
        self._sync_carrier_timer.tick(self._dt, planner)

        for id, husky in self._huskies.items():
            husky.update(self._dt)

            for action in husky.finished_actions:
                if isinstance(action, actions._PickAction):
                    trash_id = action.trash_id

                    trash = self._litter[trash_id]
                    metrics.pick_time_by_id[trash_id] = (trash.type, trash.pose.location, self._now)

                    del self._litter[trash_id]
                    self._collected_litter.add(trash_id)

            husky.finished_actions.clear()

        self._sync_poses_timer.tick(self._dt, planner)
        self._sync_robots_timer.tick(self._dt, planner)

        self._update_plan_timer.tick(self._dt, planner)

        self._now += self._dt
        metrics.collected_trash_by_time.append(
            (
                self._now,
                len(self._collected_litter)
            )
        )

    def run(self, planner):
        self._sync_poses(planner)
        self._sync_robots(planner)

        frame_countdown = 0
        prev = time.perf_counter()

        while True:
            self._events.process(self)

            if self._now >= self._run_time_ms:
                if not self._finished: metrics.save()
                self._finished = True

            if not self._paused and not self._finished:
                self._step(planner)

            now = time.perf_counter()
            frame_countdown -= (now - prev) * 1000
            prev = now

            # Visualize
            if frame_countdown <= 0:
                self._camera.render(
                    planner,
                    self._carrier,
                    self._huskies,
                    self._litter,
                    self._paused,
                    self._now
                )
                frame_countdown = 50

            pygame.display.flip()
