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

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


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
    double angle;
    double radius;
    double value;

    maximum(double angle, double radius, double value) : angle(angle), radius(radius), value(value)
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


std::vector<maximum> getMaxima(std::vector<int> accu, double &max);
void drawMap(std::vector<point> pointsOcc, std::vector<point> pointsFree, std::string filename, double res);

class occupancyMap
{

public:
    occupancyMap(const std::string& topic, ros::NodeHandle nh) : topic_(topic), nh(nh)
	{
		map_saved = false;

        sub = nh.subscribe(topic_, 1, &occupancyMap::translateMap, this);
    }

    void translateMap(const nav_msgs::OccupancyGridConstPtr& received_map)
    {
        //Punkte extrahieren
//        ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

        map = *received_map;

        width = map.info.width;
        height = map.info.height;
        res = map.info.resolution;

        pointsOccupied.clear();
        pointsFree.clear();

        center.x = width/2;
        center.y = height/2;

//        double turnamnt = 90*DEG2RAD;

        for(int y = 0; y < map.info.height; y++) {
            for(int x = 0; x < map.info.width; x++) {
                int i = x + y * map.info.width;
                if (map.data[i] == +100) { //occ (0.65,1]
                    pointsOccupied.push_back(point(x - width/2,y - height/2));


                    //std::cout << x << "," << y <<  "   " << width/2 << "," << height/2 <<  "    (" << x-width/2 << ", " << y-height/2 << ")" << std::endl;
                }
                else if(map.data[i] == 0)
                {
                    pointsFree.push_back(point(x - width/2,y - height/2));
                }
            }
        }



        //ROTATE FOR DEBUGGING

//        double x_orig;
//        double y_orig;

//        //rotate
//        if(turn)
//        {
//            for(std::vector<point>::iterator it = pointsOccupied.begin(); it != pointsOccupied.end(); it++)
//            {
//                x_orig = it->x;
//                y_orig = it->y;

//                it->x = (x_orig-center.x)*cos(turnamnt) - (y_orig-center.y)*sin(turnamnt) + center.x;// + (rand() % 20) - 10;
//                it->y = (x_orig-center.x)*sin(turnamnt) - (y_orig-center.y)*cos(turnamnt) + center.y;// + (rand() % 5) - 1;
//            }
//        }

//        if(turn)
//        {
//            for(std::vector<point>::iterator it = pointsFree.begin(); it != pointsFree.end(); it++)
//            {
//                x_orig = it->x;
//                y_orig = it->y;

//                it->x = (x_orig-center.x)*cos(turnamnt) - (y_orig-center.y)*sin(turnamnt) + center.x;
//                it->y = (x_orig-center.x)*sin(turnamnt) - (y_orig-center.y)*cos(turnamnt) + center.y;
//            }
//        }

        //DEBUG END

        //Filling the Accumulator. Important: Origin is center

        //maximalen Radius rausfinden, gleichzeitig zentrieren
        int r_max = 0;
        int r_act;

        for(std::vector<point>::iterator it = pointsOccupied.begin(); it != pointsOccupied.end(); ++it) {

            r_act = (int) ceil(sqrt((it->x)*(it->x)+(it->y)*(it->y)));

            //std::cout << count++ << "   " << r_act << std::endl;

            if(r_act > r_max)
                r_max = r_act;
        }


        accu.resize(360*r_max);
        std::fill(accu.begin(), accu.end(), 0);

        int r;

        for(std::vector<point>::iterator it = pointsOccupied.begin(); it != pointsOccupied.end(); ++it) {
            for(double t=0;t<360;t++) //Winkel
            {

                r = (int) floor(((it->x) * cos(t * DEG2RAD)) + ((it->y) * sin(t * DEG2RAD)));

                if(r > 0)
                {
                    accu[ (r * 360) + t ]++;
                }
            }
        }

		map_saved = true;
	}

	std::string topic_;
    ros::NodeHandle nh;
    ros::Subscriber sub;
	bool map_saved;
    nav_msgs::OccupancyGrid map;
	std::vector<point> pointsOccupied;
    std::vector<point> pointsFree;
    std::vector<int> accu;
    int _max;
    std::vector<maximum> _maxima;


    point center;

    int width;
    int height;
    double res;
};


double evaluate(occupancyMap &map1, occupancyMap &map2, double rot, double dx, double dy, double &resagr)
{
    std::vector<point> pointsOccupied2;
    std::vector<point> pointsFree2;

    pointsOccupied2.clear();
    pointsFree2.clear();

    for(std::vector<point>::iterator it = map2.pointsOccupied.begin(); it != map2.pointsOccupied.end(); it++)
    {
        pointsOccupied2.push_back(point(it->x * cos(rot*DEG2RAD) - it->y * sin(rot*DEG2RAD) + dx,
                                        it->x * sin(rot*DEG2RAD) + it->y * cos(rot*DEG2RAD) + dy));

        //std::cout << newx << "," << newy << std::endl;
    }

    for(std::vector<point>::iterator it = map2.pointsFree.begin(); it != map2.pointsFree.end(); it++)
    {
        pointsFree2.push_back(point(it->x * cos(rot*DEG2RAD) - it->y * sin(rot*DEG2RAD) + dx,
                                    it->x * sin(rot*DEG2RAD) + it->y * cos(rot*DEG2RAD) + dy));

        //std::cout << newx << "," << newy << std::endl;
    }

    //drawMap(map1.pointsOccupied,pointsOccupied2_copy,"raw", .05);


    //evaluate Transform

    double agr = 0;
    double dis = 0;
    double nomatch = 0;
    bool matched;


    for(std::vector<point>::iterator it = pointsOccupied2.begin(); it!=pointsOccupied2.end(); it++)
    {
        matched = false;

        for(std::vector<point>::iterator it2 = map1.pointsOccupied.begin(); it2!=map1.pointsOccupied.end(); it2++)
        {

            if(abs(it->x - it2->x) < 1 && abs(it->y - it2->y) < 1) //two occupied points overlay --> hit
            {
                matched = true;
                agr++;

                break;
            }

        }

        if(!matched) //no match found, maybe its a miss
        {
            for(std::vector<point>::iterator it2 = map1.pointsFree.begin(); it2!=map1.pointsFree.end(); it2++)
            {

                if(abs(it->x - it2->x) < 1 && abs(it->y - it2->y) < 1) //occupied and free overlay --> miss
                {
                    dis++;
                    matched = true;
                    break;
                }

            }
        }
        if(!matched)
            nomatch++;
    }

    for(std::vector<point>::iterator it = map1.pointsOccupied.begin(); it != map1.pointsOccupied.end(); it++)
    {
        matched = false;

        for(std::vector<point>::iterator it2 = pointsOccupied2.begin(); it2!=pointsOccupied2.end(); it2++)
        {

            if(abs(it->x - it2->x) < 1 && abs(it->y - it2->y) < 1) //two occupied points overlay --> hit
            {
                matched = true;
                //already counted as agr, so do nothing

                break;
            }

        }

        if(!matched) //no match found, maybe its a miss
        {
            for(std::vector<point>::iterator it2 = pointsFree2.begin(); it2!=pointsFree2.end(); it2++)
            {
                if(abs(it->x - it2->x) < 1 && abs(it->y - it2->y) < 1) //occupied and free overlay --> miss
                {
                    dis++;
                    matched = true;
                    break;
                }

            }
        }
        if(!matched)
            nomatch++;
    }

    if(agr && dis)
    {
        ROS_ERROR("agr: %.3f\ndis: %.3f\nnom: %.3f\neval: %.7f\n",agr,dis,nomatch, agr / (((agr+dis)*nomatch)));
        return agr / (agr+dis);
    }
    else
        return 0;
}


transformation calculateTransform(occupancyMap map1, occupancyMap map2)
{
    ROS_INFO("perform hough2");

    double r11;
    double r12;
    double r21;
    double r22;

    double x1;
    double y1;
    double x2;
    double y2;

    double theta11;
    double theta12;
    double theta21;
    double theta22;

    double dx;
    double dy;

    transformation best_transform;
    double best_evaluation;

    double evaluation;
    double agr;
    double best_agr;

//    int counter = 1;

    double thresholdmax1;
    double thresholdmax2;


    std::vector<maximum> allmax1 = getMaxima(map1.accu, thresholdmax1);
    std::vector<maximum> allmax2 = getMaxima(map2.accu, thresholdmax2);


//    std::vector<maximum> debugmax1 = getMaxima(map1.accu, 45, 135, threshold);
//    std::vector<maximum> debugmax2 = getMaxima(map2.accu, 45, 135, threshold);


//Debug
//    for(int i = 0; i<max1.size(); i++)
//    {
//        std::cout << max1[i].angle << "  " << max1[i].radius << std::endl;
//        std::cout << max2[i].angle << "  " << max2[i].radius  << std::endl;
//    }

//    std::cout << "debugamount" << debugmax1.size() << "  " << debugmax2.size() << std::endl;

//    for(int i = 0; i<debugmax1.size(); i++)
//    {
//        std::cout << debugmax1[i].angle << "  " << debugmax1[i].radius << std::endl;
//        std::cout << debugmax2[i].angle << "  " << debugmax2[i].radius  << std::endl;
//    }



    double threshold;

    ROS_INFO("Enter thres: ");
    std::cin >> threshold;

    thresholdmax1 *= threshold;
    thresholdmax2 *= threshold;

    std::vector<maximum> max1;
    std::vector<maximum> max2;

    for(std::vector<maximum>::iterator it = allmax1.begin(); it != allmax1.end(); it++)
    {
        if(it->value > thresholdmax1)
            max1.push_back(*it);
    }

    for(std::vector<maximum>::iterator it = allmax2.begin(); it != allmax2.end(); it++)
    {
        if(it->value > thresholdmax2)
            max2.push_back(*it);
    }

    std::vector<maximum> tempmax1;
    std::vector<maximum> tempmax2;


    ROS_INFO("max1: %d, max2: %d", max1.size(), max2.size());


//    //debug ende


    for(std::vector<maximum>::iterator it = max1.begin(); it != max1.end(); it++) //vergleiche alle maxima1...
    {
        for(std::vector<maximum>::iterator it2 = max2.begin(); it2 != max2.end(); it2++) //...mit allen maxima2
        {
            double delta_theta = fmod((it->angle - it2->angle + 360),360);


            //get lines that are not parallel and meet a certain threshold


            tempmax1.clear();
            tempmax2.clear();

            for(std::vector<maximum>::iterator it3 = max1.begin(); it3 != max1.end(); it3++)
            {
                if(fmod(((it->angle - it3->angle) + 630), 360) < 45 || fmod(((it->angle - it3->angle) + 450), 360) < 45)
                {
                    tempmax1.push_back(*it3);
                }
            }

            for(std::vector<maximum>::iterator it3 = max2.begin(); it3 != max2.end(); it3++)
            {
                if(fmod(((it2->angle - it3->angle) + 630), 360) < 45 || fmod(((it2->angle - it3->angle) + 450), 360) < 45)
                {
                    tempmax2.push_back(*it3);
                }
            }


            //look if two other lines are parallel
            for(std::vector<maximum>::iterator it4 = tempmax1.begin(); it4 != tempmax1.end(); it4++) //vergleiche alle maxima1...
            {
                for(std::vector<maximum>::iterator it5 = tempmax2.begin(); it5 != tempmax2.end(); it5++) //...mit allen maxima2
                {
                    if((abs(fmod((it4->angle - (it5->angle + delta_theta) + 722), 360) - 2) < 2) || (abs(fmod((it4->angle - (it5->angle + delta_theta) + 902), 360) - 2) < 2)) //wenn noch zwei maxima (fast) übereinstimmen können die Maps gemerged werden
                    {
                        //counter++;

                        //Hole die Winkel der 4 Linien || WICHTIG: Orginalwinkel, nicht angepasst
                        theta11 = it->angle * DEG2RAD;
                        theta12 = it4->angle * DEG2RAD;

                        theta21 = it2->angle * DEG2RAD;
                        theta22 = it5->angle * DEG2RAD;

                        //Hole die Radien der 4 Linien
                        r11 = it->radius;
                        r12 = it4->radius;
                        r21 = it2->radius;
                        r22 = it5->radius;

                        //Berechne den Punkt (x1,y1), an dem sich die beiden Linien der ersten Map schneiden
                        x1 = (r11*sin(theta12) - r12*sin(theta11))/(cos(theta11)*sin(theta12) - cos(theta12)*sin(theta11));
                        y1 = (r12*cos(theta11) - r11*cos(theta12))/(cos(theta11)*sin(theta12) - cos(theta12)*sin(theta11));


                        //Für Normalfall

                        //Berechne den Punkt (x2,y2), an dem sich die beiden Linien der zweiten Map schneiden
                        x2 = (r21*sin(theta22) - r22*sin(theta21))/(cos(theta21)*sin(theta22) - cos(theta22)*sin(theta21));
                        y2 = (r22*cos(theta21) - r21*cos(theta22))/(cos(theta21)*sin(theta22) - cos(theta22)*sin(theta21));

                        //std::cout << "x-Koordinate1: "<< (r11*sin(theta12) - r12*sin(theta11))/(cos(theta11)*sin(theta12) - cos(theta12)*sin(theta11)) << std::endl;

                        // - (x2*cos(delta_theta) - y2*sin(delta_theta))
                        // - (x2*sin(delta_theta) + y2*cos(delta_theta))

                        //Berechne die Translation der zweiten Map
                        dx = x1 - (x2*cos(delta_theta*DEG2RAD) - y2*sin(delta_theta*DEG2RAD));
                        dy = y1 - (x2*sin(delta_theta*DEG2RAD) + y2*cos(delta_theta*DEG2RAD));




                        //Alle Punkte drehen und verschieben

//                        points2_copy.clear();

//                        for(std::vector<point>::iterator it6 = map2.pointsOccupied.begin(); it6 != map2.pointsOccupied.end(); it6++)
//                        {
//                            newx = it6->x * cos(delta_theta*DEG2RAD) - it6->y * sin(delta_theta*DEG2RAD) + dx;
//                            newy = it6->x * sin(delta_theta*DEG2RAD) + it6->y * cos(delta_theta*DEG2RAD) + dy;

//                            points2_copy.push_back(point(newx, newy));

//                            //std::cout << newx << "," << newy << std::endl;
//                        }


//                        //drawMap(map1.pointsOccupied,points2_copy,"raw", .05);

                        //evaluate Transform

                        //evaluate Transform
                        evaluation = evaluate(map1, map2, delta_theta, dx, dy, agr);


                        if(best_evaluation == -1 || evaluation > best_evaluation || (evaluation == best_evaluation && agr > best_agr))
                        {
                            best_evaluation = evaluation;
                            best_agr = agr;
                            best_transform.rotation = delta_theta * DEG2RAD;
                            best_transform.translation = point(dx, dy); //translation needs to be adapted, due to initial rotation!!
                            ROS_INFO("Best eval: %.3f", best_evaluation);
                        }

                        //Für 180deg gedreht

                        dx = x1 + (x2*cos(delta_theta*DEG2RAD) - y2*sin(delta_theta*DEG2RAD));
                        dy = y1 + (x2*sin(delta_theta*DEG2RAD) + y2*cos(delta_theta*DEG2RAD));

                        delta_theta = fmod((delta_theta + 180), 360);


                        //evaluate

                        evaluation = evaluate(map1, map2, delta_theta, dx, dy, agr);


                        if(best_evaluation == -1 || evaluation > best_evaluation || (evaluation == best_evaluation && agr > best_agr))
                        {
                            best_evaluation = evaluation;
                            best_agr = agr;
                            best_transform.rotation = delta_theta * DEG2RAD;
                            best_transform.translation = point(dx, dy); //translation needs to be adapted, due to initial rotation!!
                            ROS_INFO("Best eval: %.3f", best_evaluation);
                        }

                    }
                }
            }

        }
    }

//    std::cout << counter << " Vergleiche"<< std::endl;

    ROS_ERROR("Best eval: %.3f", best_evaluation);

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

        //format topic -> cs_map_agent

        occupancyMap map1(req.topic_map_one, n);

        while(!map1.map_saved && ros::ok()) //so lange, bis map erfasst
        {
            ros::spinOnce();
        }

        map1.sub.shutdown();

        occupancyMap map2(req.topic_map_two, n);

        while(!map2.map_saved && ros::ok()) //so lange, bis map erfasst
        {
            ros::spinOnce();
        }
        map2.sub.shutdown();

        if(!map2.map_saved || !map1.map_saved) //Check if maps are really fetched
        {
            ROS_ERROR("There has been a problem. ros::ok() returned false. Shutting down");
            return 0;
        }

        ros::Time begin = ros::Time::now();

        transformation result = calculateTransform(map1, map2);

        ros::Duration dauer = ros::Time::now() - begin;

        ROS_INFO("Duration: %.5f", dauer.toSec());

        ROS_INFO("Transform from %s to %s:\nrotation: %.3f\ntranslation: %.3f, %.3f\n",
                 req.topic_map_two.c_str(), req.topic_map_one.c_str(), result.rotation, result.translation.x, result.translation.y);


        //



        cs_merge_msgs::transform response;
        response.rotation = -1*result.rotation;
        response.dx = 0;
        response.dy = 0;

        //DEBUG

        drawMap(map1.pointsOccupied, map2.pointsOccupied, "h2_before_tf", 0.05);

        double oldx;
        double oldy;

        for(std::vector<point>::iterator it = map2.pointsOccupied.begin(); it != map2.pointsOccupied.end(); it++)
        {
            oldx = it->x;
            oldy = it->y;

            it->x = oldx*cos(result.rotation) - oldy*sin(result.rotation);
            it->y = oldx*sin(result.rotation) + oldy*cos(result.rotation);
        }


        drawMap(map1.pointsOccupied, map2.pointsOccupied, "h2_after_rot", 0.05);



        for(std::vector<point>::iterator it = map2.pointsOccupied.begin(); it != map2.pointsOccupied.end(); it++)
        {
            oldx = it->x;
            oldy = it->y;

            it->x += result.translation.x;
            it->y += result.translation.y;
        }

        drawMap(map1.pointsOccupied, map2.pointsOccupied, "h2_after_tf", 0.05);




        response.stamp = ros::Time::now();

        res.result = response;


        return true;

    }

    ros::NodeHandle n;
    double threshold;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cs_merge_hough");

    ros::NodeHandle n;

    Framework frame(n);

    ROS_INFO("HOUGH MERGING");

    ros::ServiceServer service = n.advertiseService("cs_merge_hough_two", &Framework::execute, &frame);

    ros::spin();

    return 0;
}


std::vector<maximum> getMaxima(std::vector<int> accu, double &max)
{
    std::vector<maximum> maxima;
    maxima.clear();
    bool ismax;
    int value;
    int rmax = accu.size()/360;

    for(int t=0; t<360; t++)
    {
        for(int r=0; r<rmax; r++)
        {
            ismax = true;
            value = accu[r*360 + t];

            if(value < accu[r*360 + ((t+1)%360)])
                ismax = false;

            if(value < accu[r*360 + ((t+359)%360)])
                ismax = false;

            if(r > 0)
            {
                if(value < accu[(r-1)*360 + t])
                    ismax = false;
            }
            if(r<(rmax - 1))
            {
                if(value < accu[(r+1)*360 + t])
                    ismax = false;
            }



            if(ismax)
            {
                maxima.push_back(maximum(t,r,value));

                if((double) value > max)
                    max = (double) value;
            }
        }
    }

    //ROS_INFO("Got %i Maxima", maxima.size());

    return maxima;
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

    int height = abs(biggestY - smallestY);
    int width = abs(biggestX - smallestX);


    std::string file = filename + ".pgm";

    FILE* out = fopen(file.c_str(), "w");
    if(!out)
    {
        ROS_ERROR("couldnt write file");
        return;
    }

    fprintf(out, "P5\n# CREATOR: cs_icp.cpp %.3f m/pix\n%d %d\n255\n", res, width, height);

    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
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
