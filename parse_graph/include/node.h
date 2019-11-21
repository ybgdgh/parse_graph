#ifndef NODE_H
#define NODE_H

#include<stdio.h>  
#include<stdlib.h>  
#include<unistd.h>  
#include<sys/types.h>  
#include<sys/stat.h>  
#include<sys/signal.h>  
#include<fcntl.h>  
#include<termios.h>  
#include<errno.h>  
#include <iostream>  
#include "ros/ros.h"
#include <map>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "jsoncpp/json/json.h"

#include <visualization_msgs/Marker.h> 
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>


// Eigen 部分
#include <Eigen/Core>
#include <Eigen/Geometry>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_geometry/pinhole_camera_model.h"

#include "draw.h"

using namespace std;
using namespace Eigen;

class PARSE
{
    public:
    PARSE(ros::NodeHandle nh,ros::NodeHandle np);

    ~PARSE();

    void orb_pose(const geometry_msgs::PoseStamped& pose_msg);

    void tag_detections_mark(const apriltag_ros::AprilTagDetectionArray& msg);

    void darknet_Bbox(const darknet_ros_msgs::BoundingBoxes& Bound_msg);

    void CameraInfo(const sensor_msgs::CameraInfo& camera);

    void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg);

    void color_Callback(const sensor_msgs::ImageConstPtr& depth_msg);

    void init_marker();

    bool get_ar_trans(Eigen::Vector3d& trans_ar,Eigen::Quaterniond quat_ar,string ar_name);

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

    typedef Eigen::Matrix<double,9,1> Vector9d;



    //定义机器人本体到相机之间的变换
    Eigen::Isometry3d T_pari_to_other = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d T_base_to_camera = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d T_base_to_apri = Eigen::Isometry3d::Identity();

    double cx,cy,fx,fy;
    cv::Mat pg_graph,pg_image_show;
    std::map<string,Vector2d> object_2d_pose;
    std::map<string,Vector2d> object_2d_ar_pose;

    bool support_flag = false, on_flag = false;
    std::map<string,Vector9d> Support_box;
    std::map<string,Vector3d> On_box;
    std::map<string,Vector3d> object_xyz;
    std::map<string,Vector2d> object_pg_pose;
    std::map<string,string> relationships;

    bool TV=false,desk=false,computer=false,chair=false,air_conditioner=false,floor_=false,desk1=false,desk2=false,desk3=false;

    boost::property_tree::ptree knowledgegraph ;
    boost::property_tree::ptree knowledgegraph_object ;

 
    tf::TransformListener listener;

    ros::Subscriber sub_ar_track;
    ros::Subscriber sub_orb_pose;

    ros::Subscriber sub_darknet,sub_CameraInfo,sub_depth_camera,sub_color_camera;

    ros::Publisher pub_pg_show,pub_pic;
    tf::TransformBroadcaster tf_pub_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle np_;


};




#endif