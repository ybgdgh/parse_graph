#include "node.h"


cv::Mat depth_pic;
cv_bridge::CvImagePtr depth_ptr;

PARSE::PARSE(ros::NodeHandle nh,ros::NodeHandle np)
:nh_(nh),
 np_(np)
{
    //订阅odom和激光的话题
    sub_ar_track = nh_.subscribe("tag_detections", 10, &PARSE::tag_detections_mark,this);
    sub_orb_pose = nh_.subscribe("orb_slam2_rgbd/pose", 30, &PARSE::orb_pose,this);
    sub_darknet = nh_.subscribe("darknet_ros/bounding_boxes", 30, &PARSE::darknet_Bbox,this);
    sub_CameraInfo = nh_.subscribe("camera/depth/camera_info", 30, &PARSE::CameraInfo,this);
    sub_depth_camera = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 30, &PARSE::depth_Callback,this);

    // ros::Publisher pub_ar_pose = nh_.advertise<geometry_msgs::PoseStamped>("ar_pose", 10);
    // ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    std::cout << "init finish" << std::endl;
   
}

PARSE::~PARSE()
{
    if(true)
    {
        Json::Value root;
        root["name"] = "meeting_room";

        boost::property_tree::ptree knowledgegraph = PARSE::loadPoseFile("/home/ybg/knowledgegraph.json");
        boost::property_tree::ptree knowledgegraph_object = knowledgegraph.get_child("object");

        Json::Value attribute;

        // support object
        for(auto iter = Support_box.begin();iter != Support_box.end(); iter++)
        {
            string name_support_object = iter->first;
            Vector9d Sbox_support_object = iter->second;

            Json::Value object;
            object["name"] = Json::Value(name_support_object);
            object.removeMember("XYZ");
            object["XYZ"].append(Sbox_support_object[0]);
            object["XYZ"].append(Sbox_support_object[1]);
            object["XYZ"].append(Sbox_support_object[8]);

            BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, knowledgegraph_object)
            {
                boost::property_tree::ptree vt = vtt.second;
                string name_kg = vt.get<string>("name");
                if(name_support_object == name_kg )
                {
                    BOOST_FOREACH (boost::property_tree::ptree::value_type &v, vt)
                    {
                        if(v.second.get_value<std::string>() != name_kg)
                        {
                            attribute[v.first]=Json::Value(v.second.get_value<std::string>());
                        }
                    }
                    object["attribute"] = attribute;
                }
            }
            root["object"].append(object);
        }

        // on object
        for(auto iter = On_box.begin();iter != On_box.end(); iter++)
        {
            string name_on_object = iter->first;
            Vector3d Sbox_on_object = iter->second;

            Json::Value object;
            object["name"] = Json::Value(name_on_object);
            object.removeMember("XYZ");
            object["XYZ"].append(Sbox_on_object[0]);
            object["XYZ"].append(Sbox_on_object[1]);
            object["XYZ"].append(Sbox_on_object[2]);

            for(auto iter_ = Support_box.begin();iter_ != Support_box.end(); iter_++)
            {
                string name_support_object_ = iter_->first;
                if(name_support_object_.compare(name_on_object))
                {
                    Vector9d Sbox_support_object_ = iter_->second;
                    if(Sbox_on_object[2] > Sbox_support_object_[8]     // Z > Z_
                    && Sbox_on_object[0] > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                    && Sbox_on_object[0] < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                    && Sbox_on_object[1] > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                    && Sbox_on_object[1] < max(Sbox_support_object_[1],Sbox_support_object_[7]))    //Ymax > Ymin
                    {
                        object["on"] = Json::Value(name_support_object_);
                    }
                }
                BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, knowledgegraph_object)
                {
                    boost::property_tree::ptree vt = vtt.second;
                    string name_kg = vt.get<string>("name");
                    if(name_on_object == name_kg )
                    {
                        BOOST_FOREACH (boost::property_tree::ptree::value_type &v, vt)
                        {
                            if(v.second.get_value<std::string>() != name_kg)
                            {
                                attribute[v.first]=Json::Value(v.second.get_value<std::string>());
                            }
                        }
                        object["attribute"] = attribute;
                    }
                }
            }

            root["object"].append(object);
        }


        cout << "StyledWriter:" << endl;
        Json::StyledWriter sw;
        cout << sw.write(root) << endl << endl;

        // 输出到文件
        ofstream dataFile;//记录数据
        dataFile.open("/home/ybg/kg.json",ios::out);
        dataFile << sw.write(root);
        dataFile.close();

    }
}



void PARSE::darknet_Bbox(const darknet_ros_msgs::BoundingBoxes& Bound_msg)
{
    Eigen::Vector3d trans_Bbox;
    Eigen::Quaterniond quat_Bbox;

    // cx = 617.8583984375;
    // cy = 618.0875244140625;
    // fx = 327.5780944824219;
    // fy = 242.83938598632812;

    // cout << "Bound_msg : " << Bound_msg.header.frame_id << endl;
    tf::StampedTransform transform_Bbox;
    try
    {
        listener.lookupTransform("/map", "/camera",ros::Time(0), transform_Bbox);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute ar pose, skipping scan (%s)", e.what());
        ros::Duration(1.0).sleep();
        return ;
    }
    trans_Bbox << transform_Bbox.getOrigin().x(),transform_Bbox.getOrigin().y(),transform_Bbox.getOrigin().z();

    // cout << "trans : " << trans_Bbox << endl;

    quat_Bbox.w() = transform_Bbox.getRotation().getW();
    quat_Bbox.x() = transform_Bbox.getRotation().getX();
    quat_Bbox.y() = transform_Bbox.getRotation().getY();
    quat_Bbox.z() = transform_Bbox.getRotation().getZ();

    T_base_to_camera = Eigen::Isometry3d::Identity();
    T_base_to_camera.rotate ( quat_Bbox );
    T_base_to_camera.pretranslate ( trans_Bbox );

    std::stringstream sss;
    for(const darknet_ros_msgs::BoundingBox& Bbox : Bound_msg.bounding_boxes)
    {
        sss.str("");
        sss << Bbox.Class << '_' << Bbox.id;

        string name_darknet = sss.str();
        Vector3d S_darknet_object;

        std::vector<float> depth;
        // cout << "name : " << Bbox.Class << endl;  
        // for(int i =Bbox.xmin; i < Bbox.xmax; i=i+int((Bbox.xmax - Bbox.xmin)/5))
        //     for(int j =Bbox.ymin; j < Bbox.ymax; j=j+int((Bbox.ymax - Bbox.ymin)/5))
        //     {
        //         depth.push_back(depth_pic.ptr<float>(j)[i]/1000);
        //         if (depth_pic.ptr<float>(j)[i] == 0)
        //             continue;
        //     }
        // double sum = std::accumulate(std::begin(depth), std::end(depth), 0.0);
	    // double mean_depth =  sum / depth.size(); //均值

        Vector2d mean_point((Bbox.xmin+Bbox.xmax)/2,(Bbox.ymin+Bbox.ymax)/2);
        double mean_depth = depth_pic.ptr<float>(int(mean_point[1]))[int(mean_point[0])]/1000;

        double x = (mean_point[0] - cx) * mean_depth / fx;
        double y = (mean_point[1] - cy) * mean_depth / fy;

        Vector3d point(x,y,mean_depth);

        S_darknet_object = T_base_to_camera * point;

        if(Bbox.Class == "cup" || Bbox.Class == "mouse")
        {
            
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = mean_depth;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            pose.header = Bound_msg.header;
            tf::Stamped<tf::Transform> dark_transform;
            tf::poseStampedMsgToTF(pose, dark_transform);
            tf_pub_.sendTransform(tf::StampedTransform(dark_transform,
                                                        dark_transform.stamp_,
                                                        "camera",
                                                        name_darknet));
        }

        for(auto iter = On_box.begin();iter != On_box.end(); iter++)
        {
            string iter_name = iter->first;
            if(name_darknet == iter_name)
            {
                return;
            }
            
        }
        cout << "name : " << name_darknet << endl;
        cout << "size of On_box : " << On_box.size() << endl;
        On_box.insert(std::map<string,Vector3d>::value_type(name_darknet,S_darknet_object));
  
    }



}

void PARSE::CameraInfo(const sensor_msgs::CameraInfo& camera)
{
    double K[9] ;
    for( int t = 0;t<9;t++)
    {
        K[t] = camera.K[t]; 
    }
    

    cx = K[0];
    cy = K[4];
    fx = K[2];
    fy = K[5];
}


void PARSE::depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    //cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
    //depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); 
    // cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); 

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
  }

  depth_pic = depth_ptr->image;
}



void PARSE::tag_detections_mark(const apriltag_ros::AprilTagDetectionArray& msg)
{
    int t=0;
    int sum=0;
    // tf::TransformListener listener;

    std::stringstream ss;

    for(const apriltag_ros::AprilTagDetection& ar_marker : msg.detections)
    {
        ss.str("");
        ss << "tag_"<< ar_marker.id[0];

        // cout << ss.str() << endl;


        Eigen::Vector3d trans_object;
        Eigen::Quaterniond quat_object;
  

        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/map", ss.str(),ros::Time(0), transform);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute ar pose, skipping scan (%s)", e.what());
            ros::Duration(1.0).sleep();
            return ;
        }
        trans_object << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();

        quat_object.w() = transform.getRotation().getW();
        quat_object.x() = transform.getRotation().getX();
        quat_object.y() = transform.getRotation().getY();
        quat_object.z() = transform.getRotation().getZ();

        string name;
        Vector9d S_support_object;
        Vector3d S_on_object;

        // ROS_INFO("%d",  ar_marker.id[0]);

        // std::cout << "trans_object:" << ar_marker.id[0] << std::endl << trans_object << std::endl;
        // cout << "quat_object : " << quat_object.toRotationMatrix() << endl;

        T_base_to_apri = Eigen::Isometry3d::Identity();
        T_base_to_apri.rotate(quat_object.toRotationMatrix());
        T_base_to_apri.pretranslate(trans_object);
        // cout << "quat_object : " << T_base_to_apri.matrix() << endl;

        // TV
        if(ar_marker.id[0] == 0 && TV == false)
        {
            TV = true;
            name = "TV";

            Eigen::Vector3d trans(trans_object[0],trans_object[1],trans_object[2]);    

            S_on_object << trans(0),trans(1),trans_object[2];     

            on_flag = true;
        }
        // desk support function
        else if(ar_marker.id[0] == 1 && desk == false)
        {
            desk = true;
            name = "desk";

            Eigen::Vector3d t2(1,0,0);
            Eigen::Vector3d t3(0,-2,0);
            Eigen::Vector3d t4(1,-2,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            support_flag = true;
            // cout << "T_base_to_apri :" << T_base_to_apri.matrix() << endl;
            cout << "desk : " << S_support_object << endl;
        }
        // computer support_function
        else if(ar_marker.id[0] == 2 && computer == false)
        {
            computer = true;
            name = "computer";

            Eigen::Vector3d t2(0.1,0,0);
            Eigen::Vector3d t3(0,-0.3,0);
            Eigen::Vector3d t4(0.1,-0.3,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            support_flag = true;
            
        }
        // chair support_function
        else if(ar_marker.id[0] == 3 && chair == false)
        {
            chair = true;
            name = "chair";

            Eigen::Vector3d t2(0.2,0,0);
            Eigen::Vector3d t3(0,-0.2,0);
            Eigen::Vector3d t4(0.2,-0.2,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            support_flag = true;            

            // cout << "chair : " << S_object << endl; 
        }
        // air_conditioner
        else if(ar_marker.id[0] == 4 && air_conditioner == false)
        {
            air_conditioner = true;
            name = "air_conditioner";

            Eigen::Vector3d trans(trans_object[0],trans_object[1],trans_object[2]);    

            S_on_object << trans(0),trans(1),trans_object[2];

            on_flag = true;
            
        }
        
        else 
        {
            break;
        }

        if(support_flag = true)
        {
            support_flag = false;
            Support_box.insert(std::map<string,Vector9d>::value_type(name,S_support_object));
        }
        else if(on_flag = true)
        {
            on_flag = false;
            On_box.insert(std::map<string,Vector3d>::value_type(name,S_on_object));
        }
 
        
    }
   
    //ROS_INFO("%d", t);
    
}

void PARSE::orb_pose(const geometry_msgs::PoseStamped& pose_msg)
{
    // T_base_to_camera = Eigen::Isometry3d::Identity();

    Eigen::Vector3d trans;
    trans[0] = pose_msg.pose.position.x;
    trans[1] = pose_msg.pose.position.y;
    trans[2] = pose_msg.pose.position.z;

    Eigen::Quaterniond quat;
    quat.x() = pose_msg.pose.orientation.x; 
    quat.y() = pose_msg.pose.orientation.y; 
    quat.z() = pose_msg.pose.orientation.z; 
    quat.w() = pose_msg.pose.orientation.w; 

    // T_base_to_camera.rotate ( quat );
    // T_base_to_camera.pretranslate ( trans );
    // std::cout << "T_base_to_camera:" << std::endl << T_base_to_camera.matrix() << std::endl;
}

