#include <cs_merge_controller/connection_handler.h>

ConnectionHandler::ConnectionHandler() {

	worldTopic = ros::this_node::getNamespace() + "/world";
  mapTopic = ros::this_node::getNamespace() + "/map";

	map_available = false;

	//get names of other agents
	node.getParam("agents", agents);

	ROS_INFO("Expecting maps from:");

	//create a Connection for each agent
	for(std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); it++) {
		ROS_INFO("\t%s", (*it).c_str());

		connections.push_back(Connection(*it));
	}

	//get the merging method to be used
	node.getParam("merging_method", merging_method);

	ROS_INFO("Using method: %s", merging_method.c_str());

	//publisher which will publish the resulting map under world
	world_pub = node.advertise<nav_msgs::OccupancyGrid>("world", 10);

	//Define transformation between world and map (which is 0)
	transform.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion q;
	q.setRPY(0,0,0);
	transform.setRotation(q);

	world_locked = true;
	
	node.param("update_timeout", timeout, 1);	

	ROS_INFO("Update timeout: %i", timeout);
}

//get own map
void ConnectionHandler::getMap(const nav_msgs::OccupancyGridConstPtr& new_map) {
	map = *new_map;
	map_updated = true;
	map_available = true;
}

void ConnectionHandler::updateMaps() {

	ROS_INFO("Update Maps");

	ros::Time to;

	map_sub = node.subscribe(mapTopic , 1, &ConnectionHandler::getMap, this);

	to = ros::Time::now() + ros::Duration(timeout);

	map_updated = false;
	while(!map_updated && ros::ok() && (to - ros::Time::now()) > ros::Duration(0)) {
		ros::spinOnce();
		ros::Rate(10).sleep();
	}

	map_sub.shutdown();
	ros::spinOnce();

	


	for(std::vector<Connection>::iterator connection_iterator = connections.begin(); connection_iterator != connections.end(); connection_iterator++) {

    ROS_INFO("Update %s", connection_iterator->agent_name.c_str());

    connection_iterator->map_updated = false;

    map_sub = node.subscribe("/" + connection_iterator->agent_name + "/map" , 1, &Connection::getMap, &(*connection_iterator));

    to = ros::Time::now() + ros::Duration(timeout);

    while(ros::ok() && !connection_iterator->map_updated && (to - ros::Time::now()) > ros::Duration(0)) {
			ros::spinOnce();
			ros::Rate(10).sleep();
		}

		map_sub.shutdown();
		ros::spinOnce();
	}
}

void ConnectionHandler::updateTransformations() {

	ROS_INFO("Update Transformations");

	//only possible, if own map is available
	if(map_available) {

		ros::ServiceClient method_client = node.serviceClient<cs_merge_msgs::getTransformation>(merging_method);
		cs_merge_msgs::getTransformation srv;
		cs_merge_msgs::transformation res;

		srv.request.map_one = map;

		for(vector<Connection>::iterator connection_iterator = connections.begin(); connection_iterator != connections.end(); connection_iterator++) {

			//only possible if other agents is available
			if(connection_iterator->map_available) {
				srv.request.map_two = connection_iterator->map;

				if(method_client.call(srv) && srv.response.evaluation != -1) {
					if(connection_iterator->transformation.evaluation == -1 || srv.response.evaluation < connection_iterator->transformation.evaluation) {
						connection_iterator->transformation.rotation = srv.response.transformation.rotation;
						connection_iterator->transformation.translation.x = srv.response.transformation.translationX;
						connection_iterator->transformation.translation.y = srv.response.transformation.translationY;
						connection_iterator->transformation.evaluation = srv.response.evaluation;
					}
				} else {
					ROS_ERROR("Cannot reach Service Node %s", merging_method.c_str());
				}
			}
		}
	}
}

void ConnectionHandler::buildWorld() {

	ROS_INFO("Build World");

	if(map_available) {

		world_locked = true;

		world = map;

		double rotation;
		Point translation;
		Point transformedPoint;

		for(vector<Connection>::iterator connection_iterator = connections.begin(); connection_iterator != connections.end(); connection_iterator++) {

			ROS_INFO("%s tf evaluation: %f", (connection_iterator->agent_name).c_str(), connection_iterator->transformation.evaluation);

			//only possible if other agents is available
			if(connection_iterator->map_available && connection_iterator->transformation.evaluation != -1) {

				rotation = connection_iterator->transformation.rotation;
				translation = connection_iterator->transformation.translation;

				for(unsigned int y = 0; y < connection_iterator->map.info.height; y++) {
					for(unsigned int x = 0; x < connection_iterator->map.info.width; x++) {

						if(connection_iterator->map.data[x + y * connection_iterator->map.info.width] != -1) {

							//Transform point into own coordinate system
							transformedPoint.x = round(((double) x)*cos(rotation) - ((double) y)*sin(rotation) + translation.x);
							transformedPoint.y = round(((double) x)*sin(rotation) + ((double) y)*cos(rotation) + translation.y);

							if(transformedPoint.x >= 0 && transformedPoint.x < world.info.width && transformedPoint.y >= 0 && transformedPoint.y < world.info.height) {

								//if point is unknown in the world map, assume state of agents map
								if(world.data[transformedPoint.x + transformedPoint.y * world.info.width] == -1) {
									world.data[transformedPoint.x + transformedPoint.y * world.info.width] = connection_iterator->map.data[x + y * connection_iterator->map.info.width];
								} else {
									//else mean the states
									world.data[transformedPoint.x + transformedPoint.y * world.info.width] += connection_iterator->map.data[x + y * connection_iterator->map.info.width];
									world.data[transformedPoint.x + transformedPoint.y * world.info.width] /= 2;
								}
							}
						}
					}
				}
			}
		}

		world_locked = false;
	}
}

void ConnectionHandler::publishWorld() {

	ROS_INFO("Publish World");

	if(!world_locked) {
		tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldTopic, mapTopic));
		world_pub.publish(world);
	}
}
