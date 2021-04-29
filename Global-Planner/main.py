import debug
import wuc
import simulation

if __name__ == '__main__':
    debug.on()

    planner = wuc.WorkerUnitCoordinator()

    wrapper = simulation.Simulation("config.json")
    # wrapper = ros.RosWrapper(planner)
    wrapper.run(planner)
