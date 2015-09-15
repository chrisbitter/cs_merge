#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "nav_msgs/GetMap.h"

#include <cmath>
#include <algorithm>
#include <cstdlib>          //for debug purposes
#include <cstdio>

#include <sstream>

#include <time.h>

#include <cs_merge_msgs/transform.h>
#include <cs_merge_msgs/getWorld.h>
#include <tf/transform_broadcaster.h>

#define DEG2RAD 0.017453293f


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cs_merge_publisher");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<cs_merge_msgs::getWorld>("cs_merge_getWorld");

  ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("world", 10);

  cs_merge_msgs::getWorld srv;

  nav_msgs::OccupancyGrid world;

  ros::Rate loop_rate(10);

  ros::Time lastUpdate = ros::Time::now();
  ros::Duration updateInterval = ros::Duration(5);

  tf::TransformBroadcaster br;

  tf::Transform transform;

  std::string worldTopic = ros::this_node::getNamespace() + "/world";
  std::string mapTopic = ros::this_node::getNamespace() + "/map";

  transform.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);


  while(ros::ok())
  {
      if(ros::Time::now() > lastUpdate + updateInterval)
      {
          if(client.call(srv))
          {
              world = srv.response.world;
              ROS_INFO("Get World: %i", world.data.size());
              lastUpdate = ros::Time::now();
              br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldTopic, mapTopic));
              pub.publish(world);


          }
      }




      ros::spinOnce();

      loop_rate.sleep();
  }

  return 0;
}
