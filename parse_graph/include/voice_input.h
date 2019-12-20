#ifndef VOICE_INPUT_H
#define VOICE_INPUT_H

#include<stdio.h>  
#include <iostream>  
#include <map>
#include <string.h>

#include "move_base_msgs/MoveBaseActionGoal.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>


#include <stdio.h>  
#include <stdlib.h>  
#include <unistd.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <sys/signal.h>  
#include <fcntl.h>  
#include <termios.h>  
#include <errno.h>  
#include <iostream>  
#include <string.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "jsoncpp/json/json.h"
// Eigen 部分
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

using namespace std;
using namespace Eigen;

namespace INPUT
{
    
    /** Load in the json pose files into a boost::ptree **/
    inline boost::property_tree::ptree
    loadPoseFile(const std::string &json_filename)
    {
        // Read in view_dict
        boost::property_tree::ptree pt;
        std::ifstream in(json_filename);
        std::stringstream buffer;
        buffer << in.rdbuf();
        read_json(buffer, pt);
        return pt;
    }


    
    boost::property_tree::ptree PG,PG_object;

    ros::Subscriber costmap_sub;
    ros::Publisher pub_nav;



}








#endif