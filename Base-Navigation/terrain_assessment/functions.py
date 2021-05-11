
"""
Created on Tue Apr 20 20:33:06 2021
​
@author: Amiel Fernandez
​
Context: Support code for determining the Directional gradients based on the 
Plane fit assessment on a fixed size square of terrain.  In the RakerOne project,
it was a 1 m by 1 m square box which the Clearpath Husky could fit in.
​
"""
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from scipy.linalg import lstsq
import numpy as np
import math

def fit(xe, ye, ze):
    # do fit
    tmp_A = []
    tmp_b = []
    for i in range(len(xe)):
        tmp_A.append([xe[i], ye[i], 1])
        tmp_b.append(ze[i])

    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)


    # Manual solution
    # fit = (A.T * A).I * A.T * b
    # errors = b - A * fit
    # residual = np.linalg.norm(errors)

    # Or use Scipy
    if A.size > 0 and b.size > 0:
        fit, residual, rnk, s = lstsq(A, b)
        print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    #print("errors: \n", errors)
    #print("residual:", residual)
        return fit
    



def Assess_Gradients(x_low, x_high, y_low, y_high, a , b, c, d):
    """ Assess Gradient takes in the min and max x and y values for the 
    terrain bounding box along with the values of the plane fit equation coefficients
    in the form of ax + by + cz + d = 0.
    
    It returns a dict of four values that is scaled to the Max_Climb value of 45 degrees
    for the four directions [0, 1, 2, and 3].  The directions correspond to the following:
        0: _Original_ X direction of the robot when it initiated, and the direction
        forward when the robot initiated
        1: +45 degrees (counter clockwise) in Z from the 0 direction.
        2: +90 degrees in Z from the 0 direction
        3: +135 degrees in Z from the 0 direction
    
    The other facing directions can be determined as the negatives of the values of
    0 to 3.  For example the scaled gradient score in the opposite direction of the 
    original X direction for any given grid (1x1 m) is negative of the 0 value.
    
    """
    
    
    Max_Climb = 45; # Max_climb is the maximum gradient in degrees the robot could drive on;
    # NOTE: Max_climb bounds both + and - maximum.  Robot should not be driving down
    # Max_climb slope
    
    A = a; #-1.5
    B = b; #-1
    C = c; # 2
    D = d; # 0
    
    x = np.arange(x_low, x_high, .01)
    y = np.arange(y_low, y_high, .01)
    
    xx, yy = np.meshgrid(x, y)
    
    if C == 0:  # This implies a flat plane, parallel to the XY plane
        
        grade_degrees_0 = 0;
        grade_degrees_1 = 0;
        grade_degrees_2 = 0;
        grade_degrees_3 = 0;
        
    else:
    
        z = -1/C*(A*xx + B*yy + D);
        
        #plt.show()
        
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.contour3D(xx, yy, z,50)
        ax.set_xlabel('xx')
        ax.set_ylabel('yy')
        ax.set_zlabel('z');
        ax.set_title('3D contour')
        #plt.show()
        
        ## Determine the slopes of the 4 grades
        xlow = x_low
        xmid = (x_high + x_low)/2
        xhigh = x_high
        ylow = y_low
        ymid = (y_high + y_low)/2
        yhigh = y_high
        
        # Grade 0 - Original Forward direction gradient
        z_xm_yl = -1/C*(A*xmid + B*ylow + D);
        z_xm_yh = -1/C*(A*xmid + B*yhigh + D);
        
        z_delta0 = z_xm_yh - z_xm_yl;
        y_delta0 = abs(yhigh - ylow);
        
        grade_degrees_0 = math.atan(z_delta0/y_delta0)*180/3.14159
        
        # Plot the line of grade 0
        x0=np.full((20),xmid)
        y0=np.linspace(ylow,yhigh,num=20)
        z0 =np.linspace(z_xm_yl,z_xm_yh,num=20)
        
        ax.plot3D(x0, y0, z0);
        
        ######
        
        # Grade 1 - 45 degrees counterclockwise from Original Forward
        z_xl_yh = -1/C*(A*xlow + B*yhigh + D);
        z_xh_yl = -1/C*(A*xhigh + B*ylow + D);
        
        z_delta1 = z_xl_yh - z_xh_yl;
        
        y_delta1a = yhigh - ylow;
        x_delta1a = xlow - xhigh;
        
        g1_hyp = np.sqrt(np.power(y_delta1a,2)+np.power(x_delta1a,2));
        
        grade_degrees_1 = math.atan(z_delta1/g1_hyp)*180/3.14159
        
        # Plot the line of grade 1
        x1=np.linspace(xhigh,xlow,num=20)
        y1=np.linspace(ylow,yhigh,num=20)
        z1 =np.linspace(z_xh_yl,z_xl_yh,num=20)
        
        ax.plot3D(x1, y1, z1);
        
        ################
        
        # Grade 2 - 90 degrees counterclockwise from Original Forward
        z_xh_ym = -1/C*(A*xhigh + B*ymid + D);
        z_xl_ym = -1/C*(A*xlow + B*ymid + D);
        
        z_delta2 = z_xl_ym - z_xh_ym;
        x_delta2 = abs(xlow - xhigh);
        
        grade_degrees_2 = math.atan(z_delta2/x_delta2)*180/3.14159
        
        # Plot the line of grade 2
        x2=np.linspace(xhigh,xlow,num=20)
        y2=np.full((20),ymid)
        z2 =np.linspace(z_xh_ym,z_xl_ym,num=20)
        
        ax.plot3D(x2, y2, z2);
        
        ###############
        
        # Grade 3 - 135 degrees counterclockwise from Original Forward
        z_xl_yl = -1/C*(A*xlow + B*ylow + D);
        z_xh_yh = -1/C*(A*xhigh + B*yhigh + D);
        
        z_delta3 = z_xl_yl - z_xh_yh;
        
        y_delta3a = ylow - yhigh;
        x_delta3a = xlow - xhigh;
        
        g3_hyp = np.sqrt(np.power(y_delta3a,2)+np.power(x_delta3a,2));
        
        grade_degrees_3 = math.atan(z_delta3/g3_hyp)*180/3.14159
        
        # Plot the line of grade 3
        x3=np.linspace(xhigh,xlow,num=20)
        y3=np.linspace(yhigh,ylow,num=20)
        z3 =np.linspace(z_xh_yh,z_xl_yl,num=20)
        
        ax.plot3D(x3, y3, z3);
        
        plt.show()
    
    print(grade_degrees_0, grade_degrees_1, grade_degrees_2, grade_degrees_3)
    
#    # return grades dictionary
#    
#    grades = dict();
#    grades[0]=grade_degrees_0
#    grades[1]=grade_degrees_1
#    grades[2]=grade_degrees_2
#    grades[3]=grade_degrees_3
#    
#    return (grades)

    # Return dictionary of grade relative to Max_Climb of 45 degrees (defined above)
    
    grades = dict();
    grades[0]=grade_degrees_0/Max_Climb;
    grades[1]=grade_degrees_1/Max_Climb;
    grades[2]=grade_degrees_2/Max_Climb;
    grades[3]=grade_degrees_3/Max_Climb;
    
    return (grades)


#threshold = 0.9; # Percentage of Max_Climb that the robot should not drive on; 
## provides a buffer before hitting Max_Climb
#warning = 0.8; # Percent of Max_Climb where the robot is warned of steepness