#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "nav_msgs/GetMap.h"

#include <cmath>
#include <algorithm>
#include <cstdlib>          //for debug purposes
#include <cstdio>

#include <cs_merge_msgs/transform.h>
#include <cs_merge_msgs/getTransform.h>

#define DEG2RAD 0.017453293f


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

    point(const point& p) : x(p.x), y(p.y)
    {
    }
};

struct transformation {
    double rotation;
    point translation;

    transformation(double rotation, point translation) : rotation(rotation), translation(translation)
    {

    }
};

class occupancyMap
{

public:
    occupancyMap(nav_msgs::OccupancyGrid map)
    {
        map_saved = false;

        width = map->info.width;
        height = map->info.height;
        res = map->info.resolution;

        pointsOccupied.clear();
        pointsFree.clear();

        double center_x = 0;
        double center_y = 0;

        for(unsigned int y = 0; y < map->info.height; y++) {
            for(unsigned int x = 0; x < map->info.width; x++) {
                unsigned int i = x + y * map->info.width;
                if (map->data[i] == +100) {
                    pointsOccupied.push_back(point(x,y));

                    center_x += x;
                    center_y += y;
                }
                else if(map->data[i] == 0)
                {
                   pointsFree.push_back(point(x,y));
                }
            }
        }

        center_x /= pointsOccupied.size();
        center_y /= pointsOccupied.size();

        map_saved = true;
    }

    std::string topic_;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    bool map_saved;
    std::vector<point> pointsOccupied;
    std::vector<point> pointsFree;
    int width;
    int height;
    double res;
};

transformation calculateTransform(occupancyMap map1, occupancyMap map2, double fracPoints = .5, int repetitions = 25, int startPos = 100)
{
    std::vector<point> pointMatch1;
    std::vector<point> pointMatch2;

    double min_distance;
    double distance;
    unsigned int closestPoint_index;

    std::vector<point> points1_working;
    std::vector<point> points2_working;

    unsigned int amtDelete1 = round((double) map1.pointsOccupied.size() * (1-fracPoints));    //Wie viele Punkte sollen vernachlässigt werden?
    unsigned int amtDelete2 = round((double) map2.pointsOccupied.size() * (1-fracPoints));

    point center1;
    point center2;

    double global_smallest_error = -1;

    point reference_point1(0,0);
    point reference_point2(1,0);

    point reference_point1_copy;
    point reference_point2_copy;

    point optimum_p1;
    point optimum_p2;

    double xtemp;
    double ytemp;

    std::vector<point> points2_working_copy;

    int count;

    double delta_error;
    double old_error;
    double new_error;

    for(unsigned int j=0; j<repetitions; j++)
    {
        points1_working.clear();
        points1_working = map1.pointsOccupied;
        points2_working.clear();
        points2_working = map2.pointsOccupied;

        //RANSAC - throw out random points until we are left with desired amount
        for(unsigned int k=0; k<amtDelete1; k++)
        {
            points1_working.erase(points1_working.begin() + (rand() % points1_working.size()));
        }
        for(unsigned int k=0; k<amtDelete2; k++)
        {
            points2_working.erase(points2_working.begin() + (rand() % points2_working.size()));
        }

        //Calculate center
        center1.x = 0;
        center1.y = 0;

        for(std::vector<point>::iterator it = points1_working.begin(); it != points1_working.end(); it++)
        {
            center1.x += it->x;
            center1.y += it->y;
        }

        center1.x /= points1_working.size();
        center1.y /= points1_working.size();

        center2.x = 0;
        center2.y = 0;

        for(std::vector<point>::iterator it = points2_working.begin(); it != points2_working.end(); it++)
        {
            center2.x += it->x;
            center2.y += it->y;
        }

        center2.x /= points2_working.size();
        center2.y /= points2_working.size();

        for(int phi = 0; phi < 360; phi += 360/startPos)
        {
            points2_working_copy.clear();
            points2_working_copy = points2_working;

            //Rotate map2 around center and overlay centers
            for(std::vector<point>::iterator it = points2_working_copy.begin(); it != points2_working_copy.end(); it++)
            {
                xtemp = it->x;
                ytemp = it->y;
                it->x = center1.x + (xtemp - center2.x) * cos(DEG2RAD*phi) - (ytemp - center2.y) * sin(DEG2RAD*phi);
                it->y = center1.y + (xtemp - center2.x) * sin(DEG2RAD*phi) + (ytemp - center2.y) * cos(DEG2RAD*phi);
            }

            //initialize reference points and do initial transform
            reference_point1_copy = reference_point1;
            reference_point2_copy = reference_point2;

            reference_point1_copy.x = center1.x + (reference_point1.x - center2.x) * cos(DEG2RAD*phi) - (reference_point1.y - center2.y) * sin(DEG2RAD*phi);
            reference_point1_copy.y = center1.y + (reference_point1.x - center2.x) * sin(DEG2RAD*phi) + (reference_point1.y - center2.y) * cos(DEG2RAD*phi);

            reference_point2_copy.x = center1.x + (reference_point2.x - center2.x) * cos(DEG2RAD*phi) - (reference_point2.y - center2.y) * sin(DEG2RAD*phi);
            reference_point2_copy.y = center1.y + (reference_point2.x - center2.x) * sin(DEG2RAD*phi) + (reference_point2.y - center2.y) * cos(DEG2RAD*phi);


            //Failsafe, in case error bounces back and forth
            count = 500;

            //ICP Steps
            while(count-- > 0)
            {
                pointMatch1.clear();
                pointMatch2.clear();

                new_error = 0;

                //Find pairs
                for(std::vector<point>::iterator it = points1_working.begin(); it != points1_working.end(); it++) //Compare all points from map1...
                {
                    min_distance = -1;
                    closestPoint_index = -1;

                    //Find correspondences
                    for(std::vector<point>::iterator it2 = points2_working_copy.begin(); it2 != points2_working_copy.end(); it2++) //...to all points from map2
                    {
                        distance = sqrt((it->x - it2->x)*(it->x - it2->x) + (it->y - it2->y)*(it->y - it2->y));
                        if(distance < min_distance || min_distance == -1) //When distance between points is the closest so far...
                        {
                            closestPoint_index = it2 - points2_working_copy.begin(); //...save potential partners index
                            min_distance = distance;
                        }
                    }

                    if(closestPoint_index != -1) //a partner exists, so save the pair and calculate mean squared error
                    {
                        pointMatch1.push_back(*it);
                        pointMatch2.push_back(points2_working_copy[closestPoint_index]);
                        new_error += (it->x - points2_working_copy[closestPoint_index].x)*(it->x - points2_working_copy[closestPoint_index].x) + (it->y - points2_working_copy[closestPoint_index].y)*(it->y - points2_working_copy[closestPoint_index].y);
                    }
                }

                new_error = new_error / ((double) pointMatch1.size());

                if(new_error < global_smallest_error || global_smallest_error == -1)
                {
                    global_smallest_error = new_error;
                    optimum_p1 = reference_point1_copy;
                    optimum_p2 = reference_point2_copy;
                }

                delta_error = abs(old_error - new_error);

                if(!delta_error) //if local error minimum is reached, done
                    break;


                old_error = new_error;

                //Compute parameters
                //center berechnen
                center1.x = 0;
                center1.y = 0;
                center2.x = 0;
                center2.y = 0;

                for(unsigned int i = 0; i < pointMatch1.size(); i++)
                {
                    center1.x += pointMatch1[i].x;
                    center1.y += pointMatch1[i].y;

                    center2.x += pointMatch2[i].x;
                    center2.y += pointMatch2[i].y;
                }

                center1.x /= pointMatch1.size();
                center1.y /= pointMatch1.size();

                center2.x /= pointMatch1.size();
                center2.y /= pointMatch1.size();


                //punkte zentrieren
                for(unsigned int i=0; i< pointMatch1.size(); i++)
                {
                    pointMatch1[i].x -= center1.x;
                    pointMatch1[i].y -= center1.y;

                    pointMatch2[i].x -= center2.x;
                    pointMatch2[i].y -= center2.y;
                }


                double H[2][2] = {{0, 0},{0, 0}};

                for(unsigned int i=0; i< pointMatch1.size(); i++)
                {
                    H[0][0] += pointMatch1[i].x * pointMatch2[i].x;
                    H[0][1] += pointMatch1[i].x * pointMatch2[i].y;
                    H[1][0] += pointMatch1[i].y * pointMatch2[i].x;
                    H[0][0] += pointMatch1[i].y * pointMatch2[i].y;
                }

                //SVD - http://scicomp.stackexchange.com/questions/8899/robust-algorithm-for-2x2-svd

                double q1 = (H[0][0] + H[1][1])/2; //E
                double q2 = (H[0][0] - H[1][1])/2; //F
                double q3 = (H[1][0] + H[0][1])/2; //G
                double q4 = (H[1][0] - H[0][1])/2; //H

                //double q5 = sqrt(q1*q1+q4*q4); //Q
                //double q6 = sqrt(q2*q2+q3*q3); //R

                //double sx = q5+q6;
                //double sy = q5-q6;

                double a1 = atan2(q3,q2);
                double a2 = atan2(q4,q1);

                double angle1 = (a2-a1)/2;
                double angle2 = (a2+a1)/2;

                double U[2][2] = {{cos(angle2), -1*sin(angle2)}, {sin(angle2), cos(angle2)}}; //U
                //double S[2][2] = {{sx, 0}, {0, sy}};
                double V_T[2][2] = {{cos(angle1), -1*sin(angle1)}, {sin(angle1), cos(angle1)}}; //V*

                //H = USV_T

                double U_T[2][2] = {{U[0][0], U[1][0]}, {U[0][1], U[1][1]}};
                double V[2][2] = {{V_T[0][0], V_T[1][0]}, {V_T[0][1], V_T[1][1]}};

                double R[2][2] = {{V[0][0]*U_T[0][0] + V[0][1]*U_T[1][0], V[0][0]*U_T[0][1] + V[0][1]*U_T[1][1]},
                                  {V[1][0]*U_T[0][0] + V[1][1]*U_T[1][0], V[1][0]*U_T[0][1] + V[1][1]*U_T[1][1]}}; //R = VU_T


                double dx = center1.x - (R[0][0]*center2.x + R[0][1]*center2.y);
                double dy = center1.y - (R[1][0]*center2.x + R[1][1]*center2.y);


                //Transform map2 with calculated values
                for(std::vector<point>::iterator it = points2_working_copy.begin(); it != points2_working_copy.end(); it++)
                {
                    xtemp = it->x;
                    ytemp = it->y;

                    it->x = R[0][0]*xtemp - R[0][1]*ytemp + dx;
                    it->y = R[1][0]*xtemp + R[1][1]*ytemp + dy;
                }

                //Transform reference points
                xtemp = reference_point1_copy.x;
                ytemp = reference_point1_copy.y;

                reference_point1_copy.x = R[0][0]*xtemp - R[0][1]*ytemp + dx;
                reference_point1_copy.y = R[1][0]*xtemp + R[1][1]*ytemp + dy;

                xtemp = reference_point2_copy.x;
                ytemp = reference_point2_copy.y;

                reference_point2_copy.x = R[0][0]*xtemp - R[0][1]*ytemp + dx;
                reference_point2_copy.y = R[1][0]*xtemp + R[1][1]*ytemp + dy;
            }
        }
    }

    //Calculate resulting transform with reference points
    double rotation = atan2((optimum_p2.y - optimum_p1.y), (optimum_p2.x - optimum_p1.x));

    ROS_INFO("%.3f", global_smallest_error);

    transformation result(rotation, optimum_p1);
    return result;
}

class Framework
{
public:
    Framework(ros::NodeHandle n) : n(n)
    {

    }

    bool execute(cs_merge_msgs::getTransform::Request &req,
                 cs_merge_msgs::getTransform::Response &res)
    {
        //Get first map
        occupancyMap map1(req.map_one);

        while(!map1.map_saved && ros::ok()) {
            ros::spinOnce();
        }
        map1.sub.shutdown();

        //Get second map
        occupancyMap map2(req.map_two);

        while(!map2.map_saved && ros::ok()) {
            ros::spinOnce();
        }
        map2.sub.shutdown();

        //Kicks in if ros::ok() == false
        if(!map2.map_saved || !map1.map_saved)
        {
            ROS_ERROR("There has been a problem. Shutting down");
            return 0;
        }

        double frac;
        double rep;

        transformation result = calculateTransform(map1, map2);

        res.result.stamp = ros::Time::now();

        res.result.rotation = result.rotation;
        res.result.dx = result.translation.x;
        res.result.dy = result.translation.y;

        //Evaluate
        double agr = 0;
        double dis = 0;

        bool matched;

        for(std::vector<point>::iterator it2 = map2.pointsOccupied.begin(); it2!=map2.pointsOccupied.end(); it2++)
        {
            matched = false;

            for(std::vector<point>::iterator it3 = map1.pointsOccupied.begin(); it3!=map1.pointsOccupied.end(); it3++)
            {
                if(abs(it2->x - it3->x) < 1 && abs(it2->y - it3->y) < 1) //two occupied points overlay --> hit
                {
                    matched = true;
                    agr++;
                    break;
                }
            }

            if(!matched) //no match found, maybe its a miss
            {
                for(std::vector<point>::iterator it3 = map1.pointsFree.begin(); it3!=map1.pointsFree.end(); it3++)
                {
                    if(abs(it2->x - it3->x) < 1 && abs(it2->y - it3->y) < 1) //occupied and free overlay --> miss
                    {
                        dis++;
                        break;
                    }
                }
            }
        }

        for(std::vector<point>::iterator it2 = map1.pointsOccupied.begin(); it2!=map1.pointsOccupied.end(); it2++)
        {
            matched = false;

            for(std::vector<point>::iterator it3 = map2.pointsOccupied.begin(); it3!=map2.pointsOccupied.end(); it3++)
            {
                if(abs(it3->x - it2->x) < 1 && abs(it3->y - it2->y) < 1) //two occupied points overlay --> hit
                {
                    matched = true;
                    //already counted as agr in first check
                    break;
                }
            }

            if(!matched) //no match found, maybe its a miss
            {
                for(std::vector<point>::iterator it3 = map2.pointsFree.begin(); it3!=map2.pointsFree.end(); it3++)
                {
                    if(abs(it3->x - it2->x) < 1 && abs(it3->y - it2->y) < 1) //occupied and free overlay --> miss
                    {
                        dis++;
                        break;
                    }
                }
            }
        }

        return true;
    }

    ros::NodeHandle n;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cs_merge_icp_svd");

    ros::NodeHandle n;

    Framework frame(n);

    ROS_INFO("ICP SVD MERGING");

    ros::ServiceServer service = n.advertiseService("cs_merge_icp_svd", &Framework::execute, &frame);

    ros::spin();

    return 0;
}
