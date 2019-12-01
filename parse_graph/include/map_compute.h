#ifndef MAP_COMPUTE_H
#define MAP_COMPUTE_H

#include<stdio.h>  
#include <iostream>  
#include <map>
#include <string.h>
#include <tuple> 

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "jsoncpp/json/json.h"

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

// Eigen 部分
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double,9,1> Vector9d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace Map_Compute
{
    
    void Compute_Object_Map(boost::property_tree::ptree office_object,
                            const darknet_ros_msgs::BoundingBoxes& Bound_msg,
                            std::map<string,Vector4d>& after_map);

    void Compute_Relationships_Map(boost::property_tree::ptree office_relationships,
                                            std::map<string,Vector9d> Support_box,
                                            std::map<string,Vector3d> On_box,
                                            std::map<string,Vector3d> object_V,
                                            std::vector<std::tuple<string,string,string,float>>& rela_after_map);

    





}

#endif

