#include "ros/ros.h"
 include "std_msgs/String.h"

#include "cs_merge_controller.h"

#include <sstream>

#include "nav_msgs/GetMap.h"

#include <cmath>
#include <algorithm>
#include <cstdlib>          //for debug purposes
#include <cstdio>

#include <sstream>

#include <time.h>

#include <cs_merge_msgs/transform.h>
#include <cs_merge_msgs/getTransform.h>
#include <cs_merge_msgs/getWorld.h>

#define DEG2RAD 0.017453293f


struct Point {
	double x;
	double y;

	Point(double x, double y) : x(x), y(y) { }

  Point() { x = 0; y = 0; }
};

struct transformation {
	double rotation;
	Point translation;
	double evaluation;

  transformation(double rotation, Point translation, double evaluation) :
  	rotation(rotation), translation(translation), evaluation(evaluation) { }

  transformation() { rotation = 0; translation = Point(0,0); evaluation = -1; }
};

struct Connection {

	std::string agent_name;

  transformation transform();
  nav_msgs::OccupancyGrid map;

	bool map_updated;
	bool map_available = false;
  //Connection(std::string partner, transformation transform) : agent(agent), transform(transform) {}

  Connection(std::string agent_name) : agent_name(agent_name) {

	}

/*
	Connection(std::string partner, transformation transform) : this(partner) {
		this.transformation = transformation;
	}
*/

	//When a new map is available, save the map and extract the occupied Points
	void getMap(const nav_msgs::OccupancyGridConstPtr& new_map) {
		ROS_INFO("Save map from %s", agent_name.c_str());
    map = *new_map;
		map_updated = true;
		map_available = true;
	}
};

ConnectionHandler::ConnectionHandler() {

	worldTopic = ros::this_node::getNamespace() + "/world";
  mapTopic = ros::this_node::getNamespace() + "/map";

	map_available = false;

	//get names of other agents
	node.getParam("agents", agents);

	//create a Connection for each agent
	for(std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); it++) {
		ROS_INFO("Expecting maps from %s", (*it).c_str());

		connections.push_back(Connection(*it));
	}

	//get the merging method to be used
	node.getParam("merging_method", merging_method);

	//publisher which will publish the resulting map under world
	ros::Publisher world_pub = node.advertise<nav_msgs::OccupancyGrid>("world", 10);

	//Define transformation between world and map (which is 0)
	transform.setOrigin(tf::Vector3(0,0,0));
	tf::Quaternion q;
	q.setRPY(0,0,0);
	transform.setRotation(q);

	world_locked = true;
}

//get own map
void ConnectionHandler::getMap(const nav_msgs::OccupancyGridConstPtr& new_map) {
	map = *new_map;
	map_updated = true;
	map_available = true;
}

void ConnectionHandler::updateMaps(int timeout) {

	map_sub = node.subscribe(mapTopic , 1, &ConnectionHandler::getMap, this);

	map_updated = false;
	while(!map_updated && ros::ok()) {
		ros::spinOnce();
		ros::Rate(10).sleep();
	}

	map_sub.shutdown();
	ros::spinOnce();

	ros::Time to;


	for(std::vector<Connection>::iterator Connection_iterator = connections.begin(); Connection_iterator != connections.end(); Connection_iterator++) {

    ROS_INFO("Update %s", Connection_iterator->partner.c_str());

    Connection_iterator->map_updated = false;

    mapSub = node.subscribe("/" + Connection_iterator->partner + "/map" , 1, &Connection::getMap, &(*Connection_iterator));

    to = ros::Time::now() + ros::Duration(timeout);

    while(ros::ok() && !it->updated && (to - ros::Time::now()) > ros::Duration(0)) {
			ros::spinOnce();
			ros::Rate(10).sleep();
		}

		mapSub.shutdown();
		ros::spinOnce();
	}
}

void ConnectionHandler::updateTransformations() {

	//only possible, if own map is available
	if(map_available) {

		ros::ServiceClient method_client = node->serviceClient<cs_merge_msgs::getTransform>(method);
		cs_merge_msgs::getTransform srv;
		cs_merge_msgs::transform res;

		srv.request.map_one = map;

		for(vector<Connection>::iterator connection_iterator = connections.begin(); connection_iterator != connection_iterator.end(); connection_iterator++) {

			//only possible if other agents is available
			if(connection_iterator->map_available) {
				srv.request.topic_map_two = connection_iterator->map;

				if(method_client.call(srv)) {
					if(srv.response.evaluation < connection_iterator->transform.evaluation) {
						connection_iterator->transform.rotation = srv.response.rotation;
						connection_iterator->transform.translation.x = srv.response.translationX;
						connection_iterator->transform.translation.y = srv.response.translationY;
						connection_iterator->transform.evaluation = srv.response.evaluation;
					}
				} else {
					ROS_ERROR("Cannot reach Service Node %s", method.c_str());
				}
			}
		}
	}
}

void ConnectionHandler::buildWorld() {

	if(map_available) {

		world_locked = true;

		OccupancyGrid world = map;
		world_available = true;

		double rotation;
		Point translation;
		Point transformedPoint;

		for(vector<Connection>::iterator connection_iterator = connections.begin(); connection_iterator != connection_iterator.end(); connection_iterator++) {

			//only possible if other agents is available
			if(connection_iterator->map_available && connection_iterator->transform.evaluation != -1) {

				rotation = connection_iterator->transform.rotation;
				translation = connection_iterator->transform.translation;

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
	if(!world_locked) {
		tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldTopic, mapTopic));
		world_pub.publish(world);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "cs_merge_controller");

	ConnectionHandler ConnectionHandler();

	while(ros::ok()) {

		//update all the maps
		ConnectionHandler.updateMaps(2);

		//update transformations to the maps
		ConnectionHandler.updateTransformations();

		//stitch maps together to world map
		ConnectionHandler.buildWorld();

		//broadcast the map
		ConnectionHandler.publishWorld();

		ros::spinOnce();
	}

		return 0;
	}
