To confirm if everthing is working correctly:

copy cs_merge folder in your catkin_ws/src
goto catkin_ws
run catkin_make

extract bags.tar.gz

open cs_merge.launch with your favorite editor
	adjust the argument "path" to wherever you put the bags folder (don't forget the / at the end)

change ROS_NAMESPACE to /apollon
	to do this, you can run "gedit ~/.bashrc" and add "export ROS_NAMESPACE=/apollon" at the end.
	make sure to relaunch your terminal after this

now you should be set. Go to wherever you put the cs_merge.launch and run "roslaunch cs_merge.launch"


What you should see:

After all the nodes have been started, the Controller node will cycle through the following steps:

1. Update the maps: listen for the map topics of the different agents and save the current map. Via the update_timeout parameter, you can specify how long the node should wait for each map topic to appear before continuing without it.

2. Update transformations: Using the method node specified, the best possible transformations between the agents and own map are calculated.

3. Build world: using the transformations, the maps are merged into one "world" map. This doesn't happen if the own map isn't available.

4. The resulting "world" is published.



Use cs_merge.launch as a template to set parameters and launch the different nodes.

Parameters:

agents: namespaces of other agents. The Controller will look for maps under "/otheragent/map"
merging_method: name of the merging method service to be used. As of now, only cs_merge_icp_svd is available. If you write your own merging method, put the name of the service here.
update_timeout: if a map hasn't been received after X seconds, the node will continue without it (and, if available, use an old map).

Method specific parameters:

ransac_fraction: % of points to be used. 0 = 0%, 1 = 100%.
repetitions: perfom icp multiple times. Every time, new points are picked randomly.
starting_positions_amnt: Since icp depends highly on the starting position, once the point cloud centers are overlayed, one map is rotated in 360°/starting_positions_amnt steps in order to get different starting positions.


