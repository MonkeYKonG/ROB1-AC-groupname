# Project.repos file
`project.repos` file is available on:

`https://raw.githubusercontent.com/MonkeYKonG/ROB1-AC-groupname/master/project.repos`
# Available Launcher
Eight launchers are available with `wall_follower` package.
## Run the map
`ros2 launch wall_follower start_map.launch.py`

Start Gazebo and bringup turtlebot
### Available environment variables
`GAZEBO_MODEL_PATH`: Absolute path to the models directory. 
Default value is the models repository of this package.
### Available launch configurations
`use_sim_time`: Boolean => Use simulation time on turtlebot3. Default: `True`

`world`: String => Absolute path to the world file. Default: `Challenge_maze.burger` from this package.

`model_path`: String => Default path to models repository. Will be use to set `GAZEBO_MODEL_PATH` environment variable.
## Start cartographer
`ros2 launch wall_follower start_cartographer.launch.py`

Start slam package.
### Available launch configurations
`use_sim_time`: Boolean => Use simulation time on slam. Default: `True`

`cartographer_config_dir`: String => Slam configurations directory. Default: configuration files from this package.

`configuration_basename`: String => Slam configuration basename. Default: `turtlebot3_lds_2d.lua`

`resolution`: Float => Map resolution. Default: `0.05`

`publish_period_sec` Float => Map publishing frequency on seconds. Default: `1.0`
## Start navigation
`ros2 launch wall_follower start_navigation.launch.py` Launch nav2 package and wait for publishing on topic `map`.

`ros2 launch wall_follower start_navigation_through_map.launch.py` Launch nav2 package using map file and start rviz2 to navigate on it. You must define initial location on rvzi2 before navigate.
### Available launch configurations
`use_sim_time`: Boolean => Use simulation time on nav2 nodes. Default: `True`

`navigation_param_dir`: String => Nav2 parameters file path. Default: Turtlebot3 param file from `turtlebot3_navigation2` package.

`map_file`: String REQUIRED => Path to the map file. Only used with `start_navigation_through_map.launch.py` launch file.
## Start Rviz2
`ros2 launch wall_follower start_rviz2.launch.py`

Start Rviz2 node.
## Start wall_follower
`ros2 launch wall_follower wall_follower.launch.py`

Start Wall follower custom node.
### How it works
This node publish on `cmd_vel` and subscribe on `scan` topics to move on this environment. 
The node will try to navigate on the environment by following a wall placed on the right.

To localize himself and navigate to the next location node will make compute on received scan.
### Wall detection
The result of scan sensors are buffered and a mean of the buffer is used to give result. This reduce considerably the distance approximations.

Each founded distance is convert to a space point using the angle.
Then points are grouped to packs of points in goal to detect walls.

At this stage, group of points represent different walls but a point group can contain more than one wall.

To split point groups on sub groups containing only one distinct wall, points split on two groups of same size then the direction from first point to last point of the groups are compared.
If directions match (difference is less than maximum angle approximation) this is a single wall else the action is repeated recursively on the two sub groups.

The walls returned are finally compared to the next element of the return list. If the wall directions are matching then the two wall are merged together.

The final result is a list of Wall object containing limits points and direction vector of the wall.
### Next point
First nearest wall is selected from the wall list. Then the destination point is set on the line parallel to the wall on a distance equivalent to the farthest detected point of the wall in front of the bot.

Received next point is analyzed before publishing it as next point. 

If a wall different from previously selected is placed between the bot and the next location point or the wall distance is to smaller, then the bot need to turn left, the next location is recompute using this wall instead.

If the next point is to close from current location then the bot need to turn right. The next point is recomputed, the next point is set to the prolongation of the wall plus a rotation of 30 degrees. 
## Start auto mapping
`ros2 launch wall_follower wall_follower.launch.py`

Start Cartographer, Navigation2 and nav2_wall_follower nodes
### Available launch configuration
Available launch configurations are the same as start_navigation and start_cartographer launch files.
### How does nav2_wall_follower works
The node subscribe on `map` and publish on `goal_pose` topics to works.

Thanks to data contained on `map` topic, the node can interpret his location on the map and anyway the index of his current square on map.

Using this location the bot will search recursively for the nearest 10 points square containing exclusively undefined part of the map.

Then it publish the next location on `goal_pose` topic.
# Available runner
Two runner command are available on this package
## Start wall_follower node
`ros2 run wall_follower wall_follower`
## Start nav2_wall_follower node
`ros2 run wall_follower nav2_wall_follower`