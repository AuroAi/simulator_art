# Dynamic Obstacle Tracking in RViz

Depends on package sense/auro_dyn_obs

**Run the simulator with** `roslaunch scu_sim_launch sim_test.launch`
`stageros` handles multiple robot models. In our case, the first model
is that of our vehicle, while the remaining models are dynamic obstacles.
These models are set in `applications/scu_date/stage_worlds/scu.world`. In
this file, it is very easy to add or remove obstacles and set their positions
and orientations. Currently, these obstacles are controlled by the package
obstacle_controller. In each obstacle_controller node, you can set the speed and distance
for the simple back and forth motion of the obstacle. 

`stageros` also publishes *DynamicObstacles* messages the topic `/dynamic_obstacles`,
which the dynamic_obstacle_tracking node subscribes to, in `WorldCallback()`.
These message contain the id, current position, average orientation, and
time parametrized path of each obstacle. 

The *dynamic_obstacle_tracking* node subscribes to these messages, and uses
the contained data to draw a bounding box around the obstacle and predict
the future path of the obstacle. Based on this prediction, it will update/extend
the costmap accordingly so that the vehicle will not enter into the obstacle's path.
The node uses the time parameterized path and the average orientation,
and applies a Kalman Filter to it, providing a predicted linear trajectory
for the obstacle. 

The *dynamic_obstacle_tracking* node publishes three messages. It publishes
an array of bounding boxes message, which contain information on the size
and position for each box. It also publishes a *Marker* for each obstacle (in a *MarkerArray*),
that shows the id and current velocity of the object in its bounding box.
Finally, it publishes *People* messages (array of *Persons*), which contain information about the future
path of the obstacle.  

To visualize these objects in RViz, one can add the BoundingBoxArray plugin
subscribing to the topic `/auro_dynamic_obstacle_tracking/dynamic_obstacles_filtered_bb`
and the *MarkerArray* object subscribing to the topic `/auro_dynamic_obstacle_tracking/dynamic_obstacles_filtered_text`.

To see future predictions, make sure the people plugin is added to `scu_sim_launch/config/costmap.yaml`.
