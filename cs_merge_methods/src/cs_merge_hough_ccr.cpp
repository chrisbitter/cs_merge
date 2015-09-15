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
};

struct maximum {
    int angle;
    int radius;

    maximum(int angle, int radius) : angle(angle), radius(radius)
    {

    }
};

struct transformation {
    double rotation;
    point translation;

    transformation(double rotation, point translation) : rotation(rotation), translation(translation)
    {

    }

    transformation()
    {
    }
};

struct corel {

    int *data;

    int sizex;
    int sizey;

    void set(int x, int y, int value)
    {
        data[x+y*sizex] = value;
    }

    void increment(int x, int y)
    {
        data[x+y*sizex]++;
    }

    int get(int x, int y)
    {
        return data[x+y*sizex];
    }

    void reset()
    {
        for(int i=0; i<sizex*sizey; i++)
        {
            data[i] = 0;
        }
    }

    corel(int sizex, int sizey) : sizex(sizex), sizey(sizey)
    {
        data = (int*) malloc(sizex*sizey*sizeof(int));
        reset();
    }

};

void drawMap(std::vector<point> pointsOcc, std::vector<point> pointsFree, std::string filename, double res);

std::vector<double> getMaxima(std::vector<double> spectrum, int &globalmax);

class occupancyMap
{

public:
    occupancyMap(const std::string& topic, ros::NodeHandle nh) : topic_(topic), nh(nh)
	{
		map_saved = false;

        sub = nh.subscribe(topic_, 1, &occupancyMap::translateMap, this);
    }

		/**
		* Translates OccupancyGrid in point cloud.
		*/
    void translateMap(const nav_msgs::OccupancyGridConstPtr& map)
    {
        width = map->info.width;
        height = map->info.height;
        res = map->info.resolution;

        pointsOccupied.clear();
        pointsFree.clear();

        center.x = width/2;
        center.y = height/2;

        for(int y = 0; y < map->info.height; y++) {
            for(int x = 0; x < map->info.width; x++) {
                int i = x + y * map->info.width;
                if (map->data[i] == +100) { //occ (0.65,1]
                    pointsOccupied.push_back(point(x - width/2,y - height/2));

                    //std::cout << x << "," << y <<  "   " << width/2 << "," << height/2 <<  "    (" << x-width/2 << ", " << y-height/2 << ")" << std::endl;
                }
                else if(map->data[i] == 0)
                {
                    pointsFree.push_back(point(x - width/2,y - height/2));
                }
            }
        }

		map_saved = true;
	}

		/**
		 * Makes Hough Transformation. Creates parameter space/hough space
		*/
    void makeAccu()
    {
        int r_max = 0;
        int r_act;

        for(std::vector<point>::iterator it = pointsOccupied.begin(); it != pointsOccupied.end(); ++it) {

            r_act = (int) ceil(sqrt((it->x)*(it->x)+(it->y)*(it->y)));

            if(r_act > r_max)
                r_max = r_act;
        }


        accu.resize(360*r_max);

        std::fill(accu.begin(), accu.end(), 0);

        int r;

        for(std::vector<point>::iterator it = pointsOccupied.begin(); it != pointsOccupied.end(); ++it) {
            for(double t=0;t<360;t++)
            {

                r = (int) floor(((it->x) * cos(t * DEG2RAD)) + ((it->y) * sin(t * DEG2RAD)));

                if(r > 0)
                {
                    accu[ (r * 360) + t ]++;
                }
            }
        }

        spectrum.resize(360);
        std::fill(spectrum.begin(), spectrum.end(), 0);

        for(int t = 0; t< 360; t++)
        {
            for(int r=0; r<r_max; r++)
            {
                spectrum[t] += accu[(r*360) + t] * accu[(r*360) + t];
            }
        }
    }


		/**
		*	Rotates map
		*
		* @param rotationangle
		*/
    void rotateMap(double angle)
    {
        double xtemp;
        double ytemp;

        for(std::vector<point>::iterator it = pointsOccupied.begin(); it != pointsOccupied.end(); it++)
        {
            xtemp = it->x;
            ytemp = it->y;

            it->x = (xtemp)*cos(angle) - (ytemp)*sin(angle);
            it->y = (xtemp)*sin(angle) + (ytemp)*cos(angle);
        }

        for(std::vector<point>::iterator it = pointsFree.begin(); it != pointsFree.end(); it++)
        {
            xtemp = it->x;
            ytemp = it->y;

            it->x = (xtemp)*cos(angle) - (ytemp)*sin(angle);
            it->y = (xtemp)*sin(angle) + (ytemp)*cos(angle);
        }
    }

	std::string topic_;
    ros::NodeHandle nh;
    ros::Subscriber sub;
	bool map_saved;
	std::vector<point> pointsOccupied;
    std::vector<point> pointsFree;
    std::vector<unsigned int> accu;
    std::vector<double> spectrum;
    unsigned int _max;

    std::vector<maximum> _maxima;


    point center;

    int width;
    int height;
    double res;
};


transformation calculateTransform(occupancyMap map1, occupancyMap map2, double threshold)
{
    //First rotate both maps, so that map1 is x-y aligned
    map1.makeAccu();

    int initRot = 0;

    getMaxima(map1.spectrum, initRot);

    map1.rotateMap(initRot * DEG2RAD);
    map2.rotateMap(initRot * DEG2RAD);

    //make hough spectrums
    map1.makeAccu();
    map2.makeAccu();

    //now calculate Crosscorrelation between spectrums
    std::vector<double> ccr;

    ccr.resize(360);
    std::fill(ccr.begin(), ccr.end(), 0);

    double xsum = 0;
    double xysum;
    double ysum = 0;

    //calculate crosscorellation

    for(int t=0; t<360; t++)
    {
        xsum += map1.spectrum[t] * map1.spectrum[t];
        ysum += map2.spectrum[t] * map2.spectrum[t];
    }

    for(int k=0; k<360; k++)
    {
        xysum = 0;

        for(int t=0; t<360; t++)
        {
            xysum += map1.spectrum[t]*map2.spectrum[(t+k) % 360];
        }

        ccr[k] = xysum / (xsum*ysum);
    }


    int dummy; //global max not needed, but function expects argument
    std::vector<double> rotations = getMaxima(ccr, dummy);

    std::vector<point> pointsOccupied2_copy;
    std::vector<point> pointsFree2_copy;

    double x_old;
    double y_old;

    double x_max = 0;
    double x_min = map2.width;
    double y_max = 0;
    double y_min = map2.height;


    corel ccr2(map1.width,map1.height);

    transformation best_transform;
    double best_evaluation = -1;

    double evaluation;
    int best_translation;



    for(std::vector<double>::iterator it = rotations.begin(); it != rotations.end(); it++)
    {
        *it = 360 - *it;

        pointsOccupied2_copy = map2.pointsOccupied;
        pointsFree2_copy = map2.pointsFree;

        //First rotate map2

        for(std::vector<point>::iterator it2 = pointsOccupied2_copy.begin(); it2 != pointsOccupied2_copy.end(); it2++)
        {
            x_old = it2->x;
            y_old = it2->y;

            it2->x = x_old *cos(*it * DEG2RAD) - y_old*sin(*it * DEG2RAD);
            it2->y = x_old *sin(*it * DEG2RAD) + y_old*cos(*it * DEG2RAD);

            if(it2->x > x_max)
                x_max = it2->x;
            if(it2->x < x_min)
                x_min = it2->x;
            if(it2->y > y_max)
                y_max = it2->y;
            if(it2->y < y_min)
                y_min = it2->y;
        }

        for(std::vector<point>::iterator it2 = pointsFree2_copy.begin(); it2 != pointsFree2_copy.end(); it2++)
        {
            x_old = it2->x;
            y_old = it2->y;

            it2->x = x_old *cos((*it) * DEG2RAD) - y_old*sin((*it) * DEG2RAD);
            it2->y = x_old *sin((*it) * DEG2RAD) + y_old*cos((*it) * DEG2RAD);
        }

        //calculate crosscorrelation


        ccr2.reset();

        for(std::vector<point>::iterator it3 = map1.pointsOccupied.begin(); it3 != map1.pointsOccupied.end(); it3++)
        {
            for(std::vector<point>::iterator it4 = pointsOccupied2_copy.begin(); it4 != pointsOccupied2_copy.end(); it4++)
            {
                //for every point pair note the distance. h/2 & w/2 needed since origin is center of the map and distances can be negative. But vector index must be positive
                ccr2.increment((((it3->x - it4->x) + (int)map1.width/2) -1), (((it3->y - it4->y) + map1.height/2) -1));

//                //tolerance
//                ccr2[round(it3->x - it4->x + map1.width/2 + 1) + round(it3->y - it4->y + map1.height/2)*map1.width]++;
//                ccr2[round(it3->x - it4->x + map1.width/2 - 1) + round(it3->y - it4->y + map1.height/2)*map1.width]++;
//                ccr2[round(it3->x - it4->x + map1.width/2) + round(it3->y - it4->y + map1.height/2 + 1)*map1.width]++;
//                ccr2[round(it3->x - it4->x + map1.width/2) + round(it3->y - it4->y + map1.height/2 - 1)*map1.width]++;
            }

        }


        //get maximum from crosscorrelation
        int dx = 0;
        int dy = 0;
        best_translation = 0;

        for(int x = 0; x<ccr2.sizex; x++)
        {
            for(int y = 0; y<ccr2.sizey; y++)
            {
                if(ccr2.get(x,y) > best_translation)
                {
                    best_translation = ccr2.get(x,y);
                    dx = x - map1.width/2 + 1;
                    dy = y - map2.height/2 + 1;
                }
            }
        }

        //calculate translation

        for(std::vector<point>::iterator it2 = pointsOccupied2_copy.begin(); it2!=pointsOccupied2_copy.end(); it2++)
        {
            it2->x += dx;
            it2->y += dy;
        }

        for(std::vector<point>::iterator it2 = pointsFree2_copy.begin(); it2!=pointsFree2_copy.end(); it2++)
        {
            it2->x += dx;
            it2->y += dy;
        }


        std::ostringstream strs2;
        strs2 << *it;
        std::string str2 = strs2.str();


        //ROS_INFO("transform: %.1f, %.1f", dx,dy);
        drawMap(map1.pointsOccupied,pointsOccupied2_copy,str2, .05);


        //evaluate Transform
        double agr = 0;
        double dis = 0;

        bool matched;

        for(std::vector<point>::iterator it2 = pointsOccupied2_copy.begin(); it2!=pointsOccupied2_copy.end(); it2++)
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

            for(std::vector<point>::iterator it3 = pointsOccupied2_copy.begin(); it3!=pointsOccupied2_copy.end(); it3++)
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
                for(std::vector<point>::iterator it3 = pointsFree2_copy.begin(); it3!=pointsFree2_copy.end(); it3++)
                {
                    if(abs(it3->x - it2->x) < 1 && abs(it3->y - it2->y) < 1) //occupied and free overlay --> miss
                    {
                        dis++;

                        break;
                    }

                }
            }
        }

        if(agr)
        {
            evaluation = agr / (agr+dis);

            if(best_evaluation == -1 || evaluation > best_evaluation)
            {
                best_evaluation = evaluation;
                best_transform.rotation = *it * DEG2RAD;
                best_transform.translation = point(dx, dy); //translation needs to be adapted, due to initial rotation!!
                ROS_INFO("%.3f", best_evaluation);
            }
        }
    }

    //finally compensate for the inital rotation
    double dx = best_transform.translation.x;
    double dy = best_transform.translation.y;

    best_transform.translation.x = dx*cos(initRot) + dy*sin(initRot);
    best_transform.translation.y = dy*cos(initRot) - dx*sin(initRot);

    return best_transform;
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
        occupancyMap map1(req.topic_map_one, n);

        while(!map1.map_saved && ros::ok())
        {
            ros::spinOnce();
        }
        map1.sub.shutdown();

				//Get second map
        occupancyMap map2(req.topic_map_two, n);

        while(!map2.map_saved && ros::ok())
        {
            ros::spinOnce();
        }
        map2.sub.shutdown();

				//Kicks in if ros::ok() == false
        if(!map2.map_saved || !map1.map_saved)
        {
            ROS_ERROR("There has been a problem. Shutting down");
            return 0;
        }

				//Debugging: runtime
        //ros::Time begin = ros::Time::now();

				std::vector<point> map1occ = map1.pointsOccupied;
				std::vector<point> map1free = map1.pointsFree;
				std::vector<point> map2occ = map2.pointsOccupied;
				std::vector<point> map2free = map2.pointsFree;

        transformation result = calculateTransform(map1, map2, threshold);

        //ros::Duration dur = ros::Time::now() - begin;
        //ROS_INFO("Duration: %.5f", dur.toSec());


				//Decenter maps. Was used for debugging


				//#################### DEBUGGING ZONE #####################//

        for(std::vector<point>::iterator it = map1occ.begin(); it != map1occ.end(); it++)
        {
            it->x += map1.center.x;
            it->y += map1.center.y;
        }

        for(std::vector<point>::iterator it = map1free.begin(); it != map1free.end(); it++)
        {
            it->x += map1.center.x;
            it->y += map1.center.y;
        }

        for(std::vector<point>::iterator it = map2occ.begin(); it != map2occ.end(); it++)
        {
            it->x += map2.center.x;
            it->y += map2.center.y;
        }

        for(std::vector<point>::iterator it = map2free.begin(); it != map2.pointsFree.end(); it++)
        {
            it->x += map2.center.x;
            it->y += map2.center.y;
        }

        cs_merge_msgs::transform response;



        response.rotation = result.rotation;

				//since the transformation is found with centered maps, the translations need to be adjusted.
        response.dx = (map1.center.x + result.translation.x - (map2.center.x * cos(response.rotation) - map2.center.y * sin(response.rotation)));
        response.dy = (map1.center.y + result.translation.y - (map2.center.x * sin(response.rotation) + map2.center.y * cos(response.rotation)));


        double xtemp;
        double ytemp;

        for(std::vector<point>::iterator it = map2occ.begin(); it!=map2occ.end();it++)
        {
            xtemp = it->x;
            ytemp = it->y;

            it->x = xtemp * cos(response.rotation) - ytemp *sin(response.rotation) + response.dx;
            it->y = xtemp * sin(response.rotation) + ytemp *cos(response.rotation) + response.dy;
        }
        for(std::vector<point>::iterator it = map2.pointsFree.begin(); it!=map2.pointsFree.end();it++)
        {
            xtemp = it->x;
            ytemp = it->y;

            it->x = xtemp * cos(response.rotation) - ytemp *sin(response.rotation) + response.dx;
            it->y = xtemp * sin(response.rotation) + ytemp *cos(response.rotation) + response.dy;
        }

        drawMap(map1occ,map1free,"hough1", .05);
        drawMap(map2occ,map2.pointsFree,"hough2", .05);


        std::vector<point> occ2;
        occ2.reserve(map1occ.size() + map2occ.size());
        occ2.insert(occ2.end(), map1occ.begin(), map1occ.end());
        occ2.insert(occ2.end(), map2occ.begin(), map2occ.end());
        std::vector<point> free2;
        free2.reserve(map1free.size() + map2.pointsFree.size());
        free2.insert(free2.end(), map1free.begin(), map1free.end());
        free2.insert(free2.end(), map2.pointsFree.begin(), map2.pointsFree.end());


        drawMap(occ2,free2,"after", .05);

				//################## DEBUNNGING END ####################//





        response.stamp = ros::Time::now();

        res.result = response;

        return true;
    }

    ros::NodeHandle n;
    double threshold;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cs_merge_hough_ccr");

    ros::NodeHandle n;

    Framework frame(n);

    ROS_INFO("HOUGH MERGING");

    ros::ServiceServer service = n.advertiseService("cs_merge_hough_ccr", &Framework::execute, &frame);

    ros::spin();

    return 0;
}


/**
	Finds all local maxima in spectrum and the index of the global maximum.

	@param spectrum
	@param variable, with which index of global maximum is returned.
	@return vector with local maxima indices
	@return index of global maximum (via variable reference)
*/
std::vector<double> getMaxima(std::vector<double> spectrum, int &maxIndex)
{
    int globalmax = 0;
    std::vector<double> result;

    for(int i = 0; i<spectrum.size(); i++)
    {
        if(spectrum[i] > spectrum[(i+1) % spectrum.size()] && spectrum[i] > spectrum[(i+spectrum.size()-1) % spectrum.size()] )
        {
            result.push_back(i);
            if(spectrum[i] > globalmax)
                globalmax = spectrum[i];
                maxIndex = i;
        }
    }
    return result;
}


/**
	draws map into a .pgm. Mainly used for debugging
*/

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
}
