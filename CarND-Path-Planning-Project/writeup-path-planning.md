## 1. Valid trajectories ##

The car is always kept below the max speed of 50 mph (maximum value actually defined at 49.5).

THe car avoids collitions by first checking if it is too close to the car in front of it in the same lane. If so, the car has two options: 
to slow down, or to change lanes. The car will first slow down, and then, if it is actually possible to change lanes will do so, 
favoring the left lane over the right lane.

The car slows down faster than it speeds up again, in spirit similar to the multiplicative decrease / linear increase of the TCP protocol's window size determination algorithm. The idea is to slow down fast to avoid a collision, and then increase speed progressively, 
as it is safe to do so.

The definition of "too close" is 30m, using the value suggested in the walkthrough. The notion of "too close" is used both to check if we are too close to a car in front of us in the same lane, but also to see if it is safe to change lanes. When changing lanes, we consider the distance to cars that could be both in front of and behind our car.

The "linear increase" during speedup also helps with jerk minimisation.

## 2. Reflection ##

I have used the trajectory generation code based on splines described in the project walkthrough. To come up with a new trajectory, I first initialize it with the points coming from the previous trajectory that has not yet been used by the simulator, to ensure a smooth 
transition between cosecutive trajectories sent to the simulator.

For the remaining new points, I first use either the current position and the estimated previous position, or the latest two points of the
previous trajectory. I then add 3 evenly spaced points, 30m apart, within the same lane the car is in. I determine the (x,y) coordinates of these points using the waypoint information and the getXY helper function.

I then change the coordinate system from the global one, to the car's local one, using a shift (assuming the car is at (0,0) in its own 
coordinate system), and a rotation. 

With these 5 points, I can create a spline to interpolate a line to those 5 points, and then extract the remaining number of points N
in order to complete a trajectory of 50 points. N is computed using the average speed, and the assumption that it takes 20 ms for the 
simulator to move the car between consecutive points. 

Finally, I reverse back to the global coordinates system (undoing the rotation and the shift), and I add the new points to complete the
trajectory.

Note that using this method we adapt to lane changes, because the decision to change lanes (using the algorithm described in the previous
section) is done before the generation of the new points in the trajectory, and thus the three points computed using waypoint information
already take the new lane into account. A smooth transition during lange change is achieved because the reused points from the previous
trajectory were computed using the current lane, and thus the resulting trajectory contains points from the current lane and the new
one.

