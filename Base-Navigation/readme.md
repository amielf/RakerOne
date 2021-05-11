# Terrain Assessment for Base Navigation
NOTE: Base Navigation essentially was rescope to Terrain Assessment

Several openly available projects were relied upon for this effort:

- https://www.clearpathrobotics.com/assets/guides/kinetic/husky/SimulatingHusky.html -- Reused Roslaunch files but changed them to use a different world environment; reused husky model as well.  EKF_Localization was changed from 2D mode to 3D mode as well.

- https://www.clearpathrobotics.com/assets/guides/kinetic/husky/HuskyMove.html  -- DID NOT reuse the navigation stack.  All movements were performed manually using cmd_vel messages.

- https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/  -- Reused Velodyne URDF/xacros for 3D Lidar, merged with the Husky.urdf.xacro

- https://wiki.ros.org/laser_assembler  -- The Laser Assembler ROS node can indeed stitch pointcloud2 format messages though it is not explicitly stated in the documentation.

- https://gitlab.com/LIRS_Projects/LIRS-WCT -- The LIRS WCT tool was used to create the world environments with elevation changes user in the RakerOne project.  Please see their associated paper for more details.

## Commands to run simulation
The order of commands to perform are the following:

**1.  Start the RakerOne simulation in Gazebo**

`roslaunch raker_base_bring_up raker_one.launch`

This readapts the husky_gazebo ros node in the ros-melodic-husky-simulator package - removing the original planar LIDAR and adding a Velodyne HDL-32E LIDAR


**2. Start RVIZ -uses the husky_viz ROS module set up as is with no changes.**

`roslaunch husky_viz view_robot.launch`

**3. Start the LIDAR assembler node and Pointcloud2 to PCD transform.**

`roslaunch raker_laser_assembler lidar_master.launch`

Note that the merged_pointcloud needs to be added as a view in RVIZ to be visualized

**4. Move the robot around using rostopic pub /cmd_vel messagesp

The robot should move in the simulation and in RVIZ with a time delayed decaying merge of the LIDAR scan as a singe pointcloud.

The Assembler code will drop the created PCD files into .ros folder of the usr based on standard ROS behavior.

Note that this will continue to generate non small MB sized PCD files per second of operation.  The other end of the terrain assessment pipeline does not exist to remove the files over time.

***

The MATLAB and Python plane fitting code does not run live, but only in batch form.  The python version of plane fitting incorporates the gradient asseser code and plane fitting as functions in the function.py file.
