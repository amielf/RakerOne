import csv
import matplotlib.pyplot as plt
import re

plt.figure()

xs = []; ys = []
with open("data/3lanes18robots/collected_trash.csv", "r") as file:
    for line in csv.reader(file, delimiter = ";"):
        xs.append(int(line[0]))
        ys.append(int(line[1]))

ys2 = []
with open("data/18robots/collected_trash.csv", "r") as file:
    for line in csv.reader(file, delimiter=";"):
        ys2.append(int(line[1]))

plt.scatter(xs, ys, label = "Constrained")
plt.scatter(xs, ys2, label = "Unconstrained")

plt.title("Collected Litter By Time")
plt.xlabel("Time (ms)")
plt.ylabel("Litter Collected")
plt.legend(loc = "best")

plt.figure()

ys = []; dts = []
with open("data/3lanes18robots/discovery_times.csv", "r") as file:
    for line in csv.reader(file, delimiter = ";"):
        result = re.search(", (.*?)\)", line[2])
        y = int(result.group(1))
        ys.append(y)
        dts.append(int(line[3]))

plt.scatter(ys, dts, label = "Constrained")

ys2 = []; dts2 = []
with open("data/18robots/discovery_times.csv", "r") as file:
    for line in csv.reader(file, delimiter = ";"):
        result = re.search(", (.*?)\)", line[2])
        y = int(result.group(1))
        ys2.append(y)
        dts2.append(int(line[3]))

plt.scatter(ys2, dts2, label = "Unconstrained")

plt.title("Discovery Time By Position")
plt.xlabel("Y (mm)")
plt.ylabel("Discovery Time (ms)")
plt.legend(loc = "best")

plt.figure()

ys = []; wts = []
with open("data/3lanes18robots/wait_times.csv", "r") as file:
    for line in csv.reader(file, delimiter = ";"):
        result = re.search(", (.*?)\)", line[2])
        y = int(result.group(1))
        ys.append(y)
        wts.append(int(line[3]))

plt.scatter(ys, wts, label = "Constrained")

ys2 = []; wts2 = []
with open("data/18robots/wait_times.csv", "r") as file:
    for line in csv.reader(file, delimiter = ";"):
        result = re.search(", (.*?)\)", line[2])
        y = int(result.group(1))
        ys2.append(y)
        wts2.append(int(line[3]))

plt.scatter(ys2, wts2, label = "Unconstrained")

plt.title("Wait Time By Position")
plt.xlabel("Y (mm)")
plt.ylabel("Wait Time (ms)")
plt.legend(loc = "best")

plt.show()
