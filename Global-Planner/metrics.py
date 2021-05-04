import debug

discovery_time_by_id = {}
pick_time_by_id = {}
collected_trash_by_time = []
visits_by_cell = {}

def save():
    debug.log("Saving metrics")

    with open("data/discovery_times.csv", "w+") as file:
        for id, data in discovery_time_by_id.items():
            type, location, discovery_time = data
            file.write(f"{id};{type};{location};{discovery_time}\n")

    with open("data/wait_times.csv", "w+") as file:
        for id, data in pick_time_by_id.items():
            type, location, pick_time = data
            discovery_time = discovery_time_by_id[id][2]
            wait_time = pick_time - discovery_time
            file.write(f"{id};{type};{location};{wait_time}\n")

    with open("data/collected_trash.csv", "w+") as file:
        for time, n in collected_trash_by_time:
            file.write(f"{time};{n}\n")
