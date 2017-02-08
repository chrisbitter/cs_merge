#ifndef CONNECTION_HANDLER_H_
#define CONNECTION_HANDLER_H_

#include "ros/ros.h"

#include <cs_merge_msgs/structs.h>
#include <cs_merge_msgs/transformation.h>
#include <cs_merge_msgs/getTransformation.h>


#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_broadcaster.h>

#include <time.h>



using namespace std;

class ConnectionHandler
{
public:
  ConnectionHandler();
  
  void updateMaps();
  void updateTransformations();
  void buildWorld();
  void publishWorld();
private:
	void getMap(const nav_msgs::OccupancyGridConstPtr& new_map);

  ros::NodeHandle node;

  //own map and world
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid world;

	std::vector<std::string> agents;

  std::vector<Connection> connections;

  string merging_method;

  ros::Publisher world_pub;
  tf::TransformBroadcaster tf_br;
  tf::Transform transform;

  string worldTopic;
  string mapTopic;

	int timeout;

  ros::Subscriber map_sub;
  bool map_updated;
  bool map_available;
  bool world_locked;
};


#endif //CS_MERGE_CONTROLLER_H_
