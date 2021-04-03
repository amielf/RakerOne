import debug
import planner
import simulation

if __name__ == '__main__':
    debug.on()

    planner = planner.GlobalPlanner()

    wrapper = simulation.Simulation("config.json")
    # wrapper = ros.RosWrapper(planner)
    wrapper.run(planner)
