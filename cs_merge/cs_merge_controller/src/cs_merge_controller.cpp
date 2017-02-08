#include "ros/ros.h"
#include <cs_merge_controller/connection_handler.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "cs_merge_controller");

	ROS_INFO("Starting Merge Controller");

	

	ConnectionHandler connectionHandler;

	while(ros::ok()) {

		//update all the maps
		connectionHandler.updateMaps();

		//update transformations to the maps
		connectionHandler.updateTransformations();

		//stitch maps together to world map
		connectionHandler.buildWorld();

		//broadcast the map
		connectionHandler.publishWorld();

		ros::spinOnce();
	}

		return 0;
	}
