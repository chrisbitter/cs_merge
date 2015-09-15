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
#include <cs_merge_msgs/getTransform.h>
#include <cs_merge_msgs/getWorld.h>


#define DEG2RAD 0.017453293f

void drawMap(nav_msgs::OccupancyGrid map, std::string filename, double res);

struct point {
	double x;
	double y;

	point(double x, double y) : x(x), y(y)
	{

	}

    point()
    {
        x = 0;
        y = 0;
    }
};

struct transformation {
    double rotation;
    point translation;
    double evaluation;

    transformation(double rotation, point translation, double evaluation)
        : rotation(rotation), translation(translation), evaluation(evaluation)
    {

    }

    transformation()
    {
        rotation = 0;
        translation = point(0,0);

        evaluation = -1;
    }
};

struct connection {
    std::string partner;

    transformation current;
    transformation next_best;

    nav_msgs::OccupancyGrid map;

    bool map_available;
    bool updated;
    bool init;

		std::vector<point> occ_points;

    connection(std::string partner, transformation current) : partner(partner), current(current), next_best(current)
    {
        updated = false;
        map_available = false;
        init = true;
    }

    connection(std::string partner) : partner(partner)
    {
        init = false;
    }

    void getMap(const nav_msgs::OccupancyGridConstPtr& new_map)
    {
        map = *new_map;

				occ_points.clear();

        for(unsigned int y = 0; y < map.info.height; y++) {
            for(unsigned int x = 0; x < map.info.width; x++) {
                unsigned int i = x + y * map.info.width;
                if (map.data[i] == +100) {
                    occ_points.push_back(point(x,y));
								}
            }
        }


        map_available = true;
        updated = true;

        ROS_INFO("get map from %s", partner.c_str());
    }
};

class ConnectionHandler
{
public:

    ConnectionHandler()
    {
        ROS_INFO("ConnectionHandler");

        std::vector<std::string> agents;

        n.getParam("agents", agents);

        service = n.advertiseService("cs_merge_getWorld", &ConnectionHandler::sendWorld, this);


        //DEBUG
        agents.push_back("boreas");

        for(std::vector<std::string>::iterator it = agents.begin(); it != agents.end(); it++)
        {
            connections.push_back(connection(*it));
        }


        n.getParam("methods", methods);

        //DEBUG
        methods.push_back("cs_merge_icp_svd");
    }

    bool sendWorld(cs_merge_msgs::getWorld::Request &req,
                   cs_merge_msgs::getWorld::Response &res)
    {
        res.world = world;
        return true;
    }

    double evaluate(std::vector<point> &other_map, transformation &t)
    {
			std::vector<point> other_transformed;

			double xnew;
			double ynew;

			//transform map
			for(std::vector<point>::iterator it = other_map.begin(); it != other_map.end(); it++) {
				xnew = it->x*cos(t.rotation) - it->y*sin(t.rotation) + t.translation.x;
				ynew = it->x*sin(t.rotation) + it->y*cos(t.rotation) + t.translation.y;

				other_transformed.push_back(point(xnew, ynew));
			}

			//calculate MSE
			double error = 0;
			double min_distance;
			double distance;
			double pointMatches = 0;

			//Find pairs
			for(std::vector<point>::iterator it = own_occ.begin(); it != own_occ.end(); it++) //Compare all points from map1...
			{
					min_distance = -1;

					//Find correspondences
					for(std::vector<point>::iterator it2 = other_transformed.begin(); it2 != other_transformed.end(); it2++) //...to all points from map2
					{
							distance = sqrt((it->x - it2->x)*(it->x - it2->x) + (it->y - it2->y)*(it->y - it2->y));
							if(distance < min_distance || min_distance == -1) //When distance between points is the closest so far...
							{
									min_distance = distance;
									pointMatches++;
							}
					}

					if(min_distance != -1)
					{
							error += min_distance;
					}
			}

			if(pointMatches) {
				error = error / ((double) pointMatches);
				return error;
			} else {
				return -1;
			}
    }

    void updateTransformations()
    {
        ROS_INFO("update transformationss");

        std::vector<transformation> solutions;

        ros::ServiceClient client;
        cs_merge_msgs::getTransform srv;
        cs_merge_msgs::transform result;

        transformation resulting_transform;

        for(std::vector<connection>::iterator it = connections.begin(); it != connections.end(); it++)
        {

            solutions.clear();

            srv.request.topic_map_one = "map";
            srv.request.topic_map_two = "/" + it->partner + "/map"; //info.name.substr(1, info.name.size()-1); //remove slash

            //Perform Algorithms
            for(std::vector<std::string>::iterator it2 = methods.begin(); it2 != methods.end(); it2++)
            {
                client = n.serviceClient<cs_merge_msgs::getTransform>(*it2);

                if (client.call(srv))
                {
                    result = srv.response.result;

                    //ROS_INFO("Transform to %s with %s is %.3f", it->partner, *it2, result.evaluation);

                    resulting_transform = transformation(result.rotation, point(result.dx, result.dy), -1);

                    resulting_transform.evaluation = evaluate(it->occ_points, resulting_transform);;

                    solutions.push_back(resulting_transform);
                }
                else
                {
                    ROS_ERROR("Cannot reach ServiceNode %s", (*it2).c_str());
                }
            }

            //Choose best solution
            double best_evaluation = -1;
            int best_transform_index = -1;

            for(std::vector<transformation>::iterator it2 = solutions.begin(); it2 != solutions.end(); it2++)
            {
                if(it2->evaluation != -1 && (best_evaluation == -1 || it2->evaluation > best_evaluation))
                {
                    best_transform_index = it2 - solutions.begin();
                    best_evaluation = it2->evaluation;
                }
            }

            ROS_INFO("Best eval: %.3f", best_evaluation);

            if(best_evaluation != -1)
            {
                if(best_evaluation > it->next_best.evaluation || !it->init) //new transformation is better than existing
                {
                    ROS_INFO("%.3f is better than %.3f", best_evaluation, it->next_best.evaluation);
                    it->next_best = solutions[best_transform_index];
                    it->init = true;
                }
            }
            else
            {
                ROS_INFO("No transform found: %s", it->partner.c_str());
            }
        }
    }



    void addMap(nav_msgs::OccupancyGrid newMap, transformation transform)
    {

        ROS_INFO("add map");

        double rotation = transform.rotation;
        point translation = transform.translation;

        double pointx;
        double pointy;

        for(unsigned int y = 0; y < newMap.info.height; y++) {
            for(unsigned int x = 0; x < newMap.info.width; x++) {

                unsigned int i = x + y * newMap.info.width;

                //transform point
                pointx = round(((double) x)*cos(rotation) - ((double) y)*sin(rotation) + translation.x);
                pointy = round(((double) x)*sin(rotation) + ((double) y)*cos(rotation) + translation.y);


                if(pointx > 0 && pointx < world.info.width && pointy > 0 && pointy < world.info.height)
                {
                    if (newMap.data[i] == +100) { //occ (0.65,1]

                        world.data[pointx + pointy * world.info.width] = 100;
                    }
                    else if(newMap.data[i] == 0)
                    {
                        if(world.data[pointx + pointy * world.info.width] == -1)
                            world.data[pointx + pointy * world.info.width] = 0;


                        if(world.data[pointx+1 + pointy * world.info.width] == -1)
                            world.data[pointx+1 + pointy * world.info.width] = 0;

                        if(world.data[pointx-1 + pointy * world.info.width] == -1)
                            world.data[pointx-1 + pointy * world.info.width] = 0;

                        if(world.data[pointx + (pointy+1) * world.info.width] == -1)
                            world.data[pointx + (pointy+1) * world.info.width] = 0;

                        if(world.data[pointx + (pointy-1) * world.info.width] == -1)
                            world.data[pointx + (pointy-1) * world.info.width] = 0;
                    }
                }
            }
        }
    }


    void refreshOwn(const nav_msgs::OccupancyGridConstPtr& new_map)
    {
        map = *new_map;

				own_occ.clear();

        for(unsigned int y = 0; y < map.info.height; y++) {
            for(unsigned int x = 0; x < map.info.width; x++) {
                unsigned int i = x + y * map.info.width;
                if (map.data[i] == +100) {
                    own_occ.push_back(point(x,y));
								}
            }
        }

        initialized = true;
    }

    void initWorld()
    {
        initialized = false;
        mapSub = n.subscribe("map" , 1, &ConnectionHandler::refreshOwn, this);

        while(!initialized && ros::ok())
        {
            ros::spinOnce();
            ros::Rate(10).sleep();
        }

        mapSub.shutdown();
        ros::spinOnce();

        world = map;
    }

    void updateMap()
    {
        //Update own map
        ROS_INFO("update map");

        initialized = false;
        mapSub = n.subscribe("map" , 1, &ConnectionHandler::refreshOwn, this);

        while(!initialized && ros::ok())
        {
            ros::spinOnce();
            ros::Rate(10).sleep();
        }

        mapSub.shutdown();
        ros::spinOnce();


        double error = 0; //accumulated deviation from next best solution


        for(std::vector<connection>::iterator it = connections.begin(); it != connections.end(); it++)
        {
            if(it->map_available)
                error += abs(it->current.evaluation - it->next_best.evaluation); //should be positive anyways
        }

        if(error > 0) //0 can be a parameter. for now, every time theres a better solution, update
        {
            ROS_INFO("Reset");


            //reset everything
            for(std::vector<connection>::iterator it = connections.begin(); it != connections.end(); it++)
            {
                it->current = it->next_best;
            }

            //reset map to own
            world = map;
        }
        else
        {
            addMap(map, transformation());
        }


        //Add all maps to world

        for(std::vector<connection>::iterator it = connections.begin(); it != connections.end(); it++)
        {
            //First update map

            ROS_INFO("Update %s", it->partner.c_str());

            it->updated = false;

            mapSub = n.subscribe("/" + it->partner + "/map" , 1, &connection::getMap, &(*it));

            timeout = ros::Time::now() + ros::Duration(5); //1 second timeout, if map topic isnt available

            while(ros::ok() && !it->updated && (timeout - ros::Time::now()) > ros::Duration(0))
            {
                ros::spinOnce();
            }

            //Then add map to world
            if(it->map_available)
            {
                ROS_INFO("Add map: %s", it->partner.c_str());
                addMap(it->map, it->current);
            }
        }
    }


    ros::NodeHandle n;
    std::string ownName;
    ros::Subscriber mapSub;

    std::vector<std::string> methods;
    std::vector<connection> connections;

		std::vector<point> own_occ;

    ros::Time timeout;

    bool initialized;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid world;

    ros::ServiceServer service;


};

//void drawMap(std::vector<point> pointsOcc, std::vector<point> pointsFree, std::string filename, double res);


int main(int argc, char **argv)
{

  ros::init(argc, argv, "cs_merge_controller");

  //ros::NodeHandle nPrivateHandle;

  //ConnectionHandler connections(n, ownName);

  ConnectionHandler connectionHandler;

  connectionHandler.initWorld();

  //MapMerger mapMerger(n, ownName);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
      connectionHandler.updateMap();

      connectionHandler.updateTransformations();

      loop_rate.sleep();
  }

  return 0;
}


void drawMap(nav_msgs::OccupancyGrid map, std::string filename, double res)
{

    ROS_INFO("Draw");

    int color;
    unsigned int height = map.info.height;
    unsigned int width = map.info.width;


    std::string file = filename + ".pgm";

    FILE* out = fopen(file.c_str(), "w");
    if(!out)
    {
        ROS_ERROR("couldnt write file");
        return;
    }

    fprintf(out, "P5\n# CREATOR: cs_icp.cpp %.3f m/pix\n%d %d\n255\n", res, width, height);

    for(unsigned int y = 0; y < height; y++)
    {
        for(unsigned int x = 0; x < width; x++)
        {
            if(map.data[x + y*width] == 100)
            {
                color = 0;
            }
            else if(map.data[x + y*width] == 0)
            {
                color = 255;
            }
            else
            {
                color = 205;
            }

            fputc(color, out);

        }
    }

    fclose(out);

    ROS_INFO("map drawn");

}
