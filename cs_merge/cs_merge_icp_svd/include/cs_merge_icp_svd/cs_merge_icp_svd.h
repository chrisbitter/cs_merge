#ifndef ICP_SVD_H_
#define ICP_SVD_H_

#include "ros/ros.h"


#include "std_msgs/String.h"



#include <sstream>

#include "nav_msgs/OccupancyGrid.h"

#include <cmath>
#include <algorithm>
#include <cstdlib>          //for debug purposes
#include <cstdio>

#include <sstream>

#include <time.h>

#include <cs_merge_msgs/structs.h>
#include <cs_merge_msgs/transformation.h>
#include <cs_merge_msgs/getTransformation.h>

using namespace std;

class MergingMethod
{
public:
	MergingMethod();
private:
	void initParams();
	bool getTransformation(cs_merge_msgs::getTransformation::Request &req,
                 		cs_merge_msgs::getTransformation::Response &res);
	Transformation calculateTransformation(nav_msgs::OccupancyGrid &map1,
																nav_msgs::OccupancyGrid &map2);

	double ransac_fraction;
  double repetitions;
	double starting_positions_amnt;

	ros::NodeHandle node;

	ros::ServiceServer method_service;

};



#endif //ICP_SVD_H_
