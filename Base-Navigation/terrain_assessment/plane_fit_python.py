import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
from scipy.linalg import lstsq
import math
import time

from functions import fit, Assess_Gradients



# import data from excel
print("Starting excel pull ...")
excel_start = time.time()
WS = pd.read_excel('1086_029000000.xlsx', engine='openpyxl')
WS_np = np.array(WS)
excel_stop = time.time()
time_excel_pull = excel_stop - excel_start
print("... Stopped excel pull.")
print("Import time: " + str(time_excel_pull))

# separate the columns from the matrix
xs = WS_np[:,0]
ys = WS_np[:,1]
zs = WS_np[:,2]

### plot raw data
##plt.figure()
##ax = plt.subplot(111, projection='3d')
##ax.scatter(xs, ys, zs, color='b')

xscale = int(np.ceil(max(xs) - min(xs)))  # finds range of x values
print(xscale)
yscale = int(np.ceil(max(ys) - min(ys)))  # finds range of y values
print(yscale)

xbase = int(np.floor(min(xs))) # find minimum of x values
ybase = int(np.floor(min(ys))) # find min of y values
xtop = int(np.ceil(max(xs))) # find max of x values
ytop = int(np.ceil(max(ys))) # find max of y values
print(xbase, xtop) # shows range of x's
print(ybase, ytop) # shows range of x's

count_planefits = 0
start_planefit = time.time()
output_file = open("data.txt","a")

for a in range(xbase, xtop):  # loop from xmin to xmax by 1 increments
    for b in range(ybase, ytop):  # loop from ymin to ymax by 1 increment
        newrow = 1
        xe = []
        ye = []
        ze = []
        for i in range(len(xs)):
            # find any points that are within the current x to x+1 and y to y+1 ranges
            # and make add to the new array; these are points to be used to fit
            if (xs[i] < a +1 and xs[i] >= a and ys[i] < b + 1 and ys[i] > b):
                xe.append(xs[i])
                ye.append(ys[i])
                ze.append(zs[i])
        if (len(xe) > 20):
            coeff = fit(xe, ye, ze)  # returns the coefficients of a plane fit or None based on only
            print(coeff)
            if not coeff is None:
                count_planefits = count_planefits + 1
                ans = Assess_Gradients(a, a+1, b, b+1, float(coeff[0]), float(coeff[1]), float(coeff[2]), 0)
                output_file.writelines(str(a) + "," + str(a+1)+ "," + str(b) + "," + str(b+1) + "," + str(ans[0]) 
                                   + "," + str(ans[1])+ "," + str(ans[2]) + "," + str(ans[3])+ "\n")

output_file.close()
print(count_planefits)
stop_planefit = time.time()
planefit_time = stop_planefit - start_planefit
print(planefit_time)
totaltime = time_excel_pull + planefit_time
print("Total fitting time is " + str(totaltime) + " seconds for " + str(count_planefits) + " plane fits.")

# # plot plane
# xlim = ax.get_xlim()
# ylim = ax.get_ylim()
# X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
#                   np.arange(ylim[0], ylim[1]))
# Z = np.zeros(X.shape)
# for r in range(X.shape[0]):
#     for c in range(X.shape[1]):
#         Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
# ax.plot_wireframe(X,Y,Z, color='k')

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# plt.show()
