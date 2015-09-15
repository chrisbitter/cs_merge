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
    point rotationCenter;
    double angle;
    point reference;

    transformation(point rotationCenter, double angle, point reference) : rotationCenter(rotationCenter), angle(angle), reference(reference)
    {

    }
};

class occupancyMap
{

public:
    occupancyMap(const std::string& topic, ros::NodeHandle nh) : topic_(topic), nh(nh)
	{
        ROS_INFO("create map");

		map_saved = false;

        sub = nh.subscribe(topic_, 1, &occupancyMap::translateMap, this);
    }

    void translateMap(const nav_msgs::OccupancyGridConstPtr& map)
    {
        ROS_INFO("translate map");

        //Punkte extrahieren
        width = map->info.width;
        height = map->info.height;
        res = map->info.resolution;

        pointsOccupied.clear();
        pointsFree.clear();

        center_x = 0;
        center_y = 0;

        for(unsigned int y = 0; y < map->info.height; y++) {
            for(unsigned int x = 0; x < map->info.width; x++) {
                unsigned int i = x + y * map->info.width;
                if (map->data[i] == +100) { //occ (0.65,1]
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

    double center_x;
    double center_y;

	std::vector<point> pointsOccupied;
    std::vector<point> pointsFree;
    int width;
    int height;
    double res;
};


void drawMap(std::vector<point> pointsOcc, std::vector<point> pointsFree, std::string filename, double res);

transformation calculateTransform(occupancyMap map1, occupancyMap map2, double fracPoints = .8, int repetitions = 25)
{
	std::vector<point> pointMatch1;
	std::vector<point> pointMatch2;

	double min_distance;
	double distance;
	unsigned int closestPoint_index;

    std::vector<point> points1_working; //Punkte die betrachtet werden
    std::vector<point> points2_working;

    unsigned int amtDelete1 = round((double) map1.pointsOccupied.size() * (1-fracPoints));    //Wie viele Punkte sollen vernachlässigt werden?
    unsigned int amtDelete2 = round((double) map2.pointsOccupied.size() * (1-fracPoints));

    point center1;
    point center2;

    double global_smallest_error = -1;

    point reference_point1(0, 0);
    point reference_point2(50, 50);

    point reference_point1_copy;
    point reference_point2_copy;

    point optimum_p1;
    point optimum_p2;

    double x_orig;
    double y_orig;

    std::vector<point> points2_working_copy;


    int count;

    double S_x;
    double S_y;
    double S_x2;
    double S_y2;
    double S_xx;
    double S_xy;
    double S_yx;
    double S_yy;

    double theta;
    double dx;
    double dy;


    double delta_error;
    double old_error;
    double new_error;

    double maxDistance;


    for(unsigned int j=0; j<repetitions; j++)
    {
        //ROS_INFO("rep: %d", j);

        //Always do last try with all points

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


        for(int phi = 0; phi < 360; phi += 5)
        {

            points2_working_copy.clear();
            points2_working_copy = points2_working;

            //Rotate map2 around center and overlay centers
            for(std::vector<point>::iterator it = points2_working_copy.begin(); it != points2_working_copy.end(); it++)
            {
                x_orig = it->x;
                y_orig = it->y;
                it->x = center1.x + (x_orig - center2.x) * cos(DEG2RAD*phi) - (y_orig - center2.y) * sin(DEG2RAD*phi);
                it->y = center1.y + (x_orig - center2.x) * sin(DEG2RAD*phi) + (y_orig - center2.y) * cos(DEG2RAD*phi);
            }

//            std::cout << phi << std::endl;
//            drawMap(points1_working, points2_working_copy, "ausgangsposition", .05);

//            std::cin.ignore();

            //initialize reference points and do initial rotation
            reference_point1_copy = reference_point1;
            reference_point2_copy = reference_point2;

            reference_point1_copy.x = center1.x + (reference_point1.x - center2.x) * cos(DEG2RAD*phi) - (reference_point1.y - center2.y) * sin(DEG2RAD*phi);
            reference_point1_copy.y = center1.y + (reference_point1.x - center2.x) * sin(DEG2RAD*phi) + (reference_point1.y - center2.y) * cos(DEG2RAD*phi);

            reference_point2_copy.x = center1.x + (reference_point2.x - center2.x) * cos(DEG2RAD*phi) - (reference_point2.y - center2.y) * sin(DEG2RAD*phi);
            reference_point2_copy.y = center1.y + (reference_point2.x - center2.x) * sin(DEG2RAD*phi) + (reference_point2.y - center2.y) * cos(DEG2RAD*phi);


            //Find biggest point distance
            old_error = 0;
            maxDistance = 0;

            for(std::vector<point>::iterator it = points1_working.begin(); it != points1_working.end(); it++)
            {
                for(std::vector<point>::iterator it2 = points2_working_copy.begin(); it2 != points2_working_copy.end(); it2++)
                {
                    distance = sqrt(((it->x - it2->x)*(it->x - it2->x)) + ((it->y - it2->y)*(it->y - it2->y)));

                    if(distance > maxDistance)
                        maxDistance = distance;

                }


            }


            count = 500;

            //ICP Steps
            while(1)
            {
                //Failsafe, in case error bounces back and forth
                if(count-- < 0)
                    break;

                pointMatch1.clear();
                pointMatch2.clear();

                new_error = 0;

                //Find pairs
                for(std::vector<point>::iterator it = points1_working.begin(); it != points1_working.end(); it++) //Compare all points from map1...
                {
                    min_distance = maxDistance;
                    closestPoint_index = -1;

                    //Find correspondences
                    for(std::vector<point>::iterator it2 = points2_working_copy.begin(); it2 != points2_working_copy.end(); it2++) //...to all points from map2
                    {
                        distance = sqrt((it->x - it2->x)*(it->x - it2->x) + (it->y - it2->y)*(it->y - it2->y));
                        if(distance < min_distance) //When distance between points is the closest so far...
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

                new_error = new_error / ((double) (pointMatch1.size()*pointMatch1.size())); //penalize low match amount

                if(new_error < global_smallest_error || global_smallest_error == -1)
                {
                    global_smallest_error = new_error;
                    optimum_p1 = reference_point1_copy;
                    optimum_p2 = reference_point2_copy;

                    //drawMap(map1.pointsOccupied,points2_working_copy,"bestinfnk", .05);
                }

                delta_error = abs(old_error - new_error);

                if(!delta_error) //if local error minimum is reached, done
                    break;


                old_error = new_error;

                //Compute parameters
                S_x = 0;
                S_y = 0;
                S_x2 = 0;
                S_y2 = 0;
                S_xx = 0;
                S_xy = 0;
                S_yx = 0;
                S_yy = 0;

                for(unsigned int i=0; i<pointMatch1.size(); i++)
                {
                    S_x += pointMatch1[i].x;
                    S_y += pointMatch1[i].y;
                    S_x2 += pointMatch2[i].x;
                    S_y2 += pointMatch2[i].y;
                    S_xx += pointMatch1[i].x * pointMatch2[i].x;
                    S_xy += pointMatch1[i].x * pointMatch2[i].y;
                    S_yx += pointMatch1[i].y * pointMatch2[i].x;
                    S_yy += pointMatch1[i].y * pointMatch2[i].y;
                }

                theta = atan((S_x*S_y2 - S_y*S_x2 + points1_working.size()*(S_yx - S_xy)) / (pointMatch1.size()*(S_xx + S_yy) - S_x*S_x2 - S_y*S_y2));

                dx = (S_x - S_x2*cos(theta) + S_y2*sin(theta))/pointMatch1.size();
                dy = (S_y - S_x2*sin(theta) - S_y2*cos(theta))/pointMatch1.size();


                //Transform map2 with calculated values
                for(std::vector<point>::iterator it = points2_working_copy.begin(); it != points2_working_copy.end(); it++)
                {
                    x_orig = it->x;
                    y_orig = it->y;

                    it->x = cos(theta)*x_orig - sin(theta)*y_orig + dx;
                    it->y = sin(theta)*x_orig + cos(theta)*y_orig + dy;
                }

                //Transform reference points
                x_orig = reference_point1_copy.x;
                y_orig = reference_point1_copy.y;

                reference_point1_copy.x = cos(theta)*x_orig - sin(theta)*y_orig + dx;
                reference_point1_copy.y = sin(theta)*x_orig + cos(theta)*y_orig + dy;

                x_orig = reference_point2_copy.x;
                y_orig = reference_point2_copy.y;

                reference_point2_copy.x = cos(theta)*x_orig - sin(theta)*y_orig + dx;
                reference_point2_copy.y = sin(theta)*x_orig + cos(theta)*y_orig + dy;
            }
        }
    }

    //Calculate resulting transform with reference points
    double alpha1 = atan2((reference_point2.y - reference_point1.y), (reference_point2.x - reference_point1.x));
    double alpha2 = atan2((optimum_p2.y - optimum_p1.y), (optimum_p2.x - optimum_p1.x));


//    //Map um ref1 um alpha1 - alpha2 drehen, dann um dr = opt1 - ref1 verschieben

//    x = (x-px)*cos(dalpha) - (y-py)*sin(dalpha) + ox;
//    y = (x-px)*sin(dalpha) - (y-py)*cos(dalpha) + ox;

//    x' = (x'-px)*cos(-dalpha) - (y'-py)*sin(-dalpha) - ox;
//    y' = (x'-px)*sin(-dalpha) - (y'-py)*cos(-dalpha) - ox;

    double delta_alpha = alpha2 - alpha1;

    points2_working = map2.pointsOccupied;


    for(std::vector<point>::iterator it = points2_working.begin(); it!= points2_working.end();it++)
    {
        x_orig = it->x;
        y_orig = it->y;

        it->x = x_orig*cos(delta_alpha) - y_orig*sin(delta_alpha) + reference_point1.x;
        it->y = x_orig*sin(delta_alpha) + y_orig*cos(delta_alpha) + reference_point1.y;
    }

    //drawMap(map1.pointsOccupied,points2_working,"infnk", .05);


    transformation result(reference_point1, delta_alpha, optimum_p1);

    ROS_ERROR("MSE: %.3f", global_smallest_error);

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
        //format topic -> cs_map_agent

        ROS_INFO("icp requested: %s, %s", req.topic_map_one.c_str(), req.topic_map_two.c_str());

        occupancyMap map1(req.topic_map_one, n);

        while(!map1.map_saved && ros::ok()) //until map received
        {
            ros::spinOnce();
        }
        map1.sub.shutdown();

        ROS_INFO("Got map1");

        occupancyMap map2(req.topic_map_two, n);

        while(!map2.map_saved && ros::ok()) //until map received
        {
            ros::spinOnce();
        }
        map2.sub.shutdown();

        ROS_INFO("Got map2");

        if(!map2.map_saved || !map1.map_saved) //Check if maps are really fetched
        {
            ROS_ERROR("There has been a problem getting the maps");
            return 0;
        }

        ros::NodeHandle ns("~");

        double frac = .5;
        //ns.getParam("ransac_amount", frac);

        int repetitions = 25;
        //ns.getParam("repetitions", repetitions);


        std::cout << "enter frac: "; std::cin >> frac;
        std::cout << "enter rep: "; std::cin >> repetitions;

        ros::Time begin = ros::Time::now();

        transformation result = calculateTransform(map1, map2, frac, repetitions);

        ROS_INFO("Transform from %s to %s:\nRotation Center: (%.3f, %.3f)\nReference Point: (%.3f, %.3f)\nRotation: %.3fdeg",
                 req.topic_map_two.c_str(), req.topic_map_one.c_str(), result.rotationCenter.x, result.rotationCenter.y, result.reference.x, result.reference.y, result.angle/DEG2RAD);

        ros::Duration dauer = begin - ros::Time::now();
        ROS_INFO("Duration: %.5f", dauer.toSec());

        cs_merge_msgs::transform response;

        response.rotation = result.angle;

        response.dx = result.reference.x;
        response.dy = result.reference.y;

        //Draw original for debug

        std::vector<point> occ;
        occ.reserve(map1.pointsOccupied.size() + map2.pointsOccupied.size());
        occ.insert(occ.end(), map1.pointsOccupied.begin(), map1.pointsOccupied.end());
        occ.insert(occ.end(), map2.pointsOccupied.begin(), map2.pointsOccupied.end());

        std::vector<point> free;
        free.reserve(map1.pointsFree.size() + map2.pointsFree.size());
        free.insert(free.end(), map1.pointsFree.begin(), map1.pointsFree.end());
        free.insert(free.end(), map2.pointsFree.begin(), map2.pointsFree.end());


        drawMap(occ,free,"before", .05);

        //Draw transformed

        double xalt;
        double yalt;

        for(std::vector<point>::iterator it = map2.pointsOccupied.begin(); it!=map2.pointsOccupied.end();it++)
        {
            xalt = it->x;
            yalt = it->y;

            it->x = xalt * cos(response.rotation) - yalt *sin(response.rotation) + response.dx;
            it->y = xalt * sin(response.rotation) + yalt *cos(response.rotation) + response.dy;
        }
        for(std::vector<point>::iterator it = map2.pointsFree.begin(); it!=map2.pointsFree.end();it++)
        {
            xalt = it->x;
            yalt = it->y;

            it->x = xalt * cos(response.rotation) - yalt *sin(response.rotation) + response.dx;
            it->y = xalt * sin(response.rotation) + yalt *cos(response.rotation) + response.dy;
        }

        drawMap(map1.pointsOccupied,map1.pointsFree,"hough1", 10);
        drawMap(map2.pointsOccupied,map2.pointsFree,"hough2", .05);


        std::vector<point> occ2;
        occ2.reserve(map1.pointsOccupied.size() + map2.pointsOccupied.size());
        occ2.insert(occ2.end(), map1.pointsOccupied.begin(), map1.pointsOccupied.end());
        occ2.insert(occ2.end(), map2.pointsOccupied.begin(), map2.pointsOccupied.end());
        std::vector<point> free2;
        free2.reserve(map1.pointsFree.size() + map2.pointsFree.size());
        free2.insert(free2.end(), map1.pointsFree.begin(), map1.pointsFree.end());
        free2.insert(free2.end(), map2.pointsFree.begin(), map2.pointsFree.end());




        drawMap(occ2,free2,"after", .05);

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


       ROS_INFO("Eval: %.3f", agr / (agr+dis));

        response.stamp = ros::Time::now();

        res.result = response;

        return true;

    }

    ros::NodeHandle n;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cs_merge_icp");

    ros::NodeHandle n;

    Framework frame(n);

    ROS_INFO("ICP MERGING");

    ros::ServiceServer service = n.advertiseService("cs_merge_icp", &Framework::execute, &frame);

    ros::spin();

    return 0;
}


void drawMap(std::vector<point> pointsOcc, std::vector<point> pointsFree, std::string filename, double res)
{

    ROS_INFO("Draw");

    int color;

    double smallestX = pointsOcc[0].x;
    double biggestX = pointsOcc[0].x;
    double smallestY = pointsOcc[0].y;
    double biggestY = pointsOcc[0].y;


    for(std::vector<point>::iterator it = pointsOcc.begin(); it != pointsOcc.end(); it++) {
        if(it->x < smallestX)
            smallestX = it->x;
        if(it->x > biggestX)
            biggestX = it->x;
        if(it->y < smallestY)
            smallestY = it->y;
        if(it->y > biggestY)
            biggestY = it->y;
    }

    for(std::vector<point>::iterator it = pointsFree.begin(); it != pointsFree.end(); it++) {
        if(it->x < smallestX)
            smallestX = it->x;
        if(it->x > biggestX)
            biggestX = it->x;
        if(it->y < smallestY)
            smallestY = it->y;
        if(it->y > biggestY)
            biggestY = it->y;
    }

    for(std::vector<point>::iterator it = pointsOcc.begin(); it != pointsOcc.end(); it++) {
        it->x -= smallestX;
        it->y -= smallestY;
    }

    for(std::vector<point>::iterator it = pointsFree.begin(); it != pointsFree.end(); it++) {
        it->x -= smallestX;
        it->y -= smallestY;
    }

    unsigned int height = abs(biggestY - smallestY);
    unsigned int width = abs(biggestX - smallestX);


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
            color = 205;

            for(std::vector<point>::iterator it = pointsOcc.begin(); it != pointsOcc.end(); it++)
            {
                if(((it->x - x)*(it->x - x) + (it->y - y)*(it->y - y)) < 1)
                {
                    color = 0;
                    break;
                }
            }
            if(color == 205)
            {
                for(std::vector<point>::iterator it = pointsFree.begin(); it != pointsFree.end(); it++)
                {
                    if(((it->x - x)*(it->x - x) + (it->y - y)*(it->y - y)) < 1)
                    {
                        color = 255;
                        break;
                    }
                }
            }

            fputc(color, out);

        }
    }

    fclose(out);

    ROS_INFO("map drawn");

}
