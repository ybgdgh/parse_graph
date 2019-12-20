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
#include <math.h>
#include <tuple> 

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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


// Eigen 部分
#include <Eigen/Core>
#include <Eigen/Geometry>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_geometry/pinhole_camera_model.h"

#include "draw.h"
#include "map_compute.h"


using namespace std;
using namespace Eigen;

class Parse_Node
{
    public:
    Parse_Node(ros::NodeHandle nh,ros::NodeHandle np);

    ~Parse_Node();

    void tag_detections_mark(const apriltag_ros::AprilTagDetectionArray& msg);

    void darknet_Bbox(const darknet_ros_msgs::BoundingBoxes& Bound_msg);

    void CameraInfo(const sensor_msgs::CameraInfo& camera);

    void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg);

    void color_Callback(const sensor_msgs::ImageConstPtr& depth_msg);

    void init_marker();

    bool get_ar_trans(Eigen::Vector3d& trans_ar,Eigen::Quaterniond quat_ar,string ar_name);

    void Publishtf(Vector3d point,std::string tf1, std::string tf2);

    void Listentf(std::string tf1, std::string tf2,Eigen::Isometry3d& T);

    void PublishMarker(std::map<string,Vector3d> object_V);


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
    typedef Eigen::Matrix<double,6,1> Vector6d;

    // camera info
    double cx,cy,fx,fy;

    // publish the pg and image
    cv::Mat pg_graph,pg_image_show;

    // darknet detect 2d pose 
    std::map<string,Vector2d> object_2d_pose;
    // ar_tag 2d pose
    std::map<string,Vector2d> object_2d_ar_pose;

    // depth image receive
    cv::Mat depth_pic;
    cv_bridge::CvImagePtr depth_ptr;

    // record support and on object's Bbox
    std::map<string,Vector9d> Support_box;
    std::map<string,Vector3d> On_box;
    std::map<string,Vector3d> On_box_local;

    // record object's 3d pose
    std::map<string,Vector3d> object_xyz;
    std::map<string,Vector3d> object_xyz_local;

    // record the V of the object
    std::map<string,Vector3d> object_V;
    std::map<string,Vector3d> object_V_local;

    // record the probability of the object
    std::map<string,float> object_P;
    

    // record the relationships of object
    std::map<string,string> support_relationships;
    std::map<string,string> contian_relationships;
    std::map<string,std::vector<string>> adjoin_relationships;
  
    // bool TV=false,desk=false,computer=false,chair=false,air_conditioner=false,floor_=false,desk1=false,desk2=false,desk3=false;

    boost::property_tree::ptree vg_AOG;
    boost::property_tree::ptree kitchen,conference,bathroom,office,dining,living,bedroom;
    boost::property_tree::ptree office_object,office_relationships;
    boost::property_tree::ptree knowledgegraph ;
    boost::property_tree::ptree knowledgegraph_object ;

    std::vector<std::tuple<string,Vector4d,float>> id_object_p_only;

    // save the id for every class . calss1,2,3...
    std::map<string,std::vector<string>> class_id;
    

    // keyframe group active
    std::vector<std::vector<std::tuple<string,Vector3d,Vector3d>>> KFG_active;
    // keyframe group optimize   
    std::vector<std::vector<std::tuple<string,Vector3d,Vector3d>>> KFG_optimize;
    //Flag of active is full!
    bool KFG_Active_Finish;
    // KFG number
    int KFG_Number;

    

       

 
    tf::TransformListener listener;

    ros::Subscriber sub_ar_track;

    ros::Subscriber sub_darknet,sub_CameraInfo,sub_depth_camera,sub_color_camera;

    ros::Publisher pub_pg_show,pub_pic;
    ros::Publisher marker_pub;

    tf::TransformBroadcaster tf_pub_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle np_;


};




#endif