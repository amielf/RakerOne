import pygame
import sim.camera as camera

from . import fleet

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
    def __init__(self, planner, config):
        self._planner = planner
        self._fleet = fleet.FleetSimulator(10)

        surface = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption("Global Planner Simulation")

        self._camera = camera.Camera(surface)

        self._update_plan_timer = _IntervalTimer(config["plan_interval_ms"], self._update_plan)
        self._sync_fleet_timer = _IntervalTimer(config["fleet_sync_interval_ms"], self._sync_fleet)

        self._clock = pygame.time.Clock()

        self._paused = False

        self._events = _EventPump(self)
        self._events.handle(pygame.QUIT, self._on_quit)
        self._events.handle(pygame.MOUSEBUTTONDOWN, self._camera.on_click)
        self._events.handle(pygame.MOUSEMOTION, self._camera.on_drag)
        self._events.handle(pygame.MOUSEBUTTONUP, self._camera.on_release)
        self._events.handle(pygame.MOUSEWHEEL, self._camera.on_scroll)
        self._events.handle(pygame.KEYDOWN, self._on_keydown)

    def _on_quit(self, _):
        pygame.quit()
        quit(0)

    def _on_keydown(self, event):
        if event.key == pygame.K_SPACE:
            self._paused = not self._paused

    def _update_plan(self):
        plan = self._planner.compute()
        self._fleet.receive(plan)

    def _sync_fleet(self):
        poses_readonly = {id: (pose.x, pose.y, pose.theta) for id, pose in self._fleet.poses.items()}
        self._planner.sync(poses_readonly)

    def run(self):
        self._sync_fleet()

        while True:
            self._events.process(self)

            dt = self._clock.tick(60)

            if not self._paused:
                self._update_plan_timer.tick(dt)
                self._sync_fleet_timer.tick(dt)

                self._fleet.update(dt)

            self._camera.render(self._planner, self._fleet)
