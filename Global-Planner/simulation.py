import time

import actions
import camera
import debug
import entities
import grid
import json
import metrics
import pygame
import random
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
    def __init__(self, config_filename):
        pygame.init()

        with open(config_filename, "r") as file:
            config = json.load(file, object_hook = lambda d: types.SimpleNamespace(**d))

        self._desired_run_time = config.simulation.run_time_sec * 1000
        self._sim_time = 0

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

        self._events = _EventPump(self)
        self._events.handle(pygame.QUIT, self._on_quit)
        self._events.handle(pygame.KEYDOWN, self._on_keydown)

        self._clock = pygame.time.Clock()

        # Entities
        self._carrier = entities.Carrier(config.carrier.speed_mph * 0.44704)
        self._carrier_dy = 0

        self._huskies = {
            i + 1: entities.Husky(
                config.robot.full_charge_v,
                config.robot.bin_capacity_mm3,
                config.robot.lidar_range_mm,
                config.robot.lidar_arc_deg,
                config.robot.yolo_range_mm,
                config.robot.yolo_arc_deg,
                config.robot.linear_speed_mph * 0.44704,
                config.robot.rotational_speed_dps
            )

            for i in range(config.simulation.n_robots)
        }

        self._litter = {}
        self._litter_distribution = config.litter_distribution
        self._collected_litter = {}

        # Camera
        surface = pygame.display.set_mode((700, 900))
        pygame.display.set_caption("Global Planner Simulator")
        self._camera = camera.Camera(surface, self._events)

        self._paused = False
        self._finished = False

    # Event Handlers
    def _on_quit(self, _):
        pygame.quit()
        quit(0)

    def _on_keydown(self, event):
        if event.key == pygame.K_p: self._paused = not self._paused

    # Interval Functions
    def _update_plan(self, planner):
        plan = planner.get()
        for id, commands in plan.items():
            self._huskies[id].execute(commands)

    def _sync_carrier(self, planner):
        vy = self._carrier.speed
        planner.sync_carrier(self._carrier_dy, vy)
        self._carrier_dy = 0

    def _sync_poses(self, planner):
        poses = {}
        for id, husky in self._huskies.items():
            pose_relative_to_carrier = husky.pose.relative_to(self._carrier.pose)
            poses[id] = pose_relative_to_carrier

        planner.sync_poses(poses)

    def _sync_robots(self, planner):
        for id, husky in self._huskies.items():
            charge = 100 * (husky.charge / husky.full_charge)
            bin = 100 * (husky.bin / husky.bin_capacity)

            grid = husky.get_partial_grid(self._grid)

            litter = husky.get_visible_litter(self._litter.values())

            for trash in litter:
                trash_id = trash[0]
                if trash_id not in metrics.discovery_time_by_id:
                    trash = self._litter[trash_id]
                    metrics.discovery_time_by_id[trash_id] = (trash.type, trash.pose.location, self._sim_time)

            next_task_please = len(husky.actions) == 0

            planner.sync_robot(id, charge, bin, husky.end_effector, grid, litter, next_task_please)

    # Run
    def _setup(self):
        random.seed(8286)

        self._sim_time = 0

        # Set robot poses and initial values
        end_effectors = ["spike", "gripper", "suction"]

        for id, husky in self._huskies.items():
            husky.pose.x = 1500 + 2000 * (id - 1)
            husky.pose.y = 500
            husky.pose.a = random.randint(60, 120)

            husky.end_effector = end_effectors[(id - 1) % 3]

        hat = []
        for i in range(len(self._litter_distribution.types)):
            hat.extend(i for _ in range(self._litter_distribution.types[i].percentage))

        assert len(hat) == 100, f"Distribution percentages sum to {len(hat)}%"

        for j in range(self._litter_distribution.total):
            i = random.choice(hat)
            type = self._litter_distribution.types[i]

            self._litter[j + 1] = entities.Trash(
                j + 1,
                type.name,
                type.volume_mm3,
                random.uniform(*type.certainty),
                type.end_effectors
            )

        padding = 1000
        for id, trash in self._litter.items():
            trash.pose.x = random.randint(padding, self._grid.width - padding)
            trash.pose.y = random.randint(padding, self._grid.height - padding)

    @debug.profiled
    def _step(self, dt, planner):
        dy = int(self._carrier.speed * dt)
        self._carrier.pose.y += dy
        self._carrier_dy += dy

        self._sync_carrier_timer.tick(dt, planner)

        for id, husky in self._huskies.items():
            husky.update(dt)

        self._sync_poses_timer.tick(dt, planner)
        self._sync_robots_timer.tick(dt, planner)

        self._update_plan_timer.tick(dt, planner)

        self._sim_time += dt
        metrics.collected_trash_by_time.append((self._sim_time, len(planner.tasks.finished_retrieval_tasks)))

    def run(self, planner):
        self._setup()
        self._sync_poses(planner)

        frame_countdown = 50
        prev = time.perf_counter()

        while True:
            self._events.process(self)

            if self._sim_time >= self._desired_run_time:
                if not self._finished: metrics.save()
                self._finished = True

            if not self._paused and not self._finished:
                self._step(500, planner)

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
                    self._sim_time
                )
                frame_countdown = 50

            pygame.display.flip()
