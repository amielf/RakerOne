discovery_time_by_id = {}
pick_time_by_id = {}

def save():
    print("Saving...")

    with open("task_discovery_times.csv", "w+") as file:
        for id, data in discovery_time_by_id.items():
            type, location, discovery_time = data
            file.write(f"{id};{type};{location};{discovery_time}\n")

    with open("task_wait_times.csv", "w+") as file:
        type, location, pick_time = data
        for id, data in pick_time_by_id.items():
            discovery_time = discovery_time_by_id[id][2]
            wait_time = pick_time - discovery_time
            file.write(f"{id};{type};{location};{wait_time}\n")

    print("... done!")
