A GitHub repository that implements the Vector Field Histogram (VFH) navigation algorithm on a Turtlebot3. VFH is used to select waypoints in an obstacle-free space and the Dynamic Window Approach (DWA) is used to plan smooth paths to the waypoint and select appropriate robot velocities.

VFH_only.mp4 provides a demo of the VFH in action. VFH provides a (blue) waypoint in obstacle-free space.

![](https://github.com/manavendradesai/vector_field_histogram/blob/master/VFH_turtlebot.gif)

VFH_and_DWA.mp4 provides a demo of the VFH and DWA in action. VFH provides a (blue) waypoint in obstacle-free space.
The Dynamic Window Approach (DWA) samples smooth and collision-free paths to the VFH-selected waypoint. The turtlebot 
follows these paths to eventually get to its goal (green arrow).
