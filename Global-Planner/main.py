import planner
import sim

if __name__ == '__main__':
    gp = planner.GlobalPlanner()

    config = {
        "plan_interval_ms": 5000,
        "fleet_sync_interval_ms": 200
    }

    wrapper = sim.Simulation(gp, config)
    # wrapper = ros.RosWrapper(gp)
    wrapper.run()
