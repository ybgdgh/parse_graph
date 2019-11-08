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
    sub_darknet = nh_.subscribe("darknet_ros/bounding_boxes", 10, &PARSE::darknet_Bbox,this);
    sub_CameraInfo = nh_.subscribe("camera/color/camera_info", 10, &PARSE::CameraInfo,this);
    sub_depth_camera = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 10, &PARSE::depth_Callback,this);

    sub_color_camera = nh_.subscribe("/camera/color/image_raw", 30, &PARSE::color_Callback,this);

    pub_pic = nh_.advertise<sensor_msgs::Image>("kg_camera", 10);
    // ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    knowledgegraph = PARSE::loadPoseFile("/home/ybg/knowledgegraph.json");
    knowledgegraph_object = knowledgegraph.get_child("object");

    std::cout << "init finish" << std::endl;
   
}

PARSE::~PARSE()
{
    if(true)
    {
        Json::Value root;
        root["name"] = "meeting_room";

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
    object_2d_pose.clear();
    for(const darknet_ros_msgs::BoundingBox& Bbox : Bound_msg.bounding_boxes)
    {
        sss.str("");
        sss << Bbox.Class << '_' << Bbox.id;

        string name_darknet = sss.str();
        Vector3d S_darknet_object;


        Vector2d mean_point((Bbox.xmin+Bbox.xmax)/2,(Bbox.ymin+Bbox.ymax)/2);
        double mean_depth = depth_pic.ptr<float>(int(mean_point[1]))[int(mean_point[0])]/1000;

        double x = (mean_point[0] - cy) * mean_depth / fy;
        double y = (mean_point[1] - cx) * mean_depth / fx;

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

            Vector2d d_pose((Bbox.xmin+Bbox.xmax)/2,(Bbox.ymin+Bbox.ymax)/2);

            if(object_2d_pose.count(Bbox.Class) == 0)
                object_2d_pose.insert(std::map<string,Vector2d>::value_type(Bbox.Class,d_pose));


            if(On_box.count(name_darknet)>0)
            {
                continue;
            }
                
            
            // cout << "name : " << name_darknet << endl;
            // cout << "size of On_box : " << On_box.size() << endl;
            On_box.insert(std::map<string,Vector3d>::value_type(Bbox.Class,S_darknet_object));

        }

       
  
    }



}

void PARSE::CameraInfo(const sensor_msgs::CameraInfo& camera_info)
{
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info);
      // Get camera intrinsic properties for rectified image.
    fx = camera_model.fx(); // focal length in camera x-direction [px]
    fy = camera_model.fy(); // focal length in camera y-direction [px]
    cx = camera_model.cx(); // optical center x-coordinate [px]
    cy = camera_model.cy(); // optical center y-coordinate [px]

}


void PARSE::depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); 

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
  }

  depth_pic = depth_ptr->image;
}


void PARSE::color_Callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;



    // 画箭头
    // on object
    for(auto iter = Support_box.begin();iter != Support_box.end(); iter++)
    {
        string name_support_object_ = iter->first;
        if(object_2d_ar_pose.count(name_support_object_)>0)
        {
            Vector9d Sbox_support_object_ = iter->second;

            for(auto iter_ = On_box.begin();iter_ != On_box.end(); iter_++)
            {
                string name_on_object_ = iter_->first;
                if(object_2d_ar_pose.count(name_on_object_)>0 || object_2d_pose.count(name_on_object_)>0)
                {
            
                    Vector3d Sbox_on_object = iter_->second;
                    if(Sbox_on_object[2] > Sbox_support_object_[8]     // Z > Z_
                    && Sbox_on_object[0] > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                    && Sbox_on_object[0] < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                    && Sbox_on_object[1] > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                    && Sbox_on_object[1] < max(Sbox_support_object_[1],Sbox_support_object_[7]))    //Ymax > Ymin
                    {
                        if(object_2d_ar_pose.count(name_on_object_)>0)
                            cv::arrowedLine(image, cv::Point(object_2d_ar_pose[name_support_object_][0], object_2d_ar_pose[name_support_object_][1]),
                            cv::Point(object_2d_ar_pose[name_on_object_][0], object_2d_ar_pose[name_on_object_][1]), cv::Scalar(0, 255, 0), 2, 4,0,0.1);
                        else if(object_2d_pose.count(name_on_object_)>0)
                            cv::arrowedLine(image, cv::Point(object_2d_ar_pose[name_support_object_][0], object_2d_ar_pose[name_support_object_][1]),
                            cv::Point(object_2d_pose[name_on_object_][0], object_2d_pose[name_on_object_][1]), cv::Scalar(0, 255, 0), 2, 4,0,0.1);
                    }
                
                }
                
        
            }
        }
        

    }


    // 画圈、写字
    int add=15;
    int r=0,g=255,b=255;

    //yolo
    for(auto iter = object_2d_pose.begin();iter != object_2d_pose.end(); iter++)
    {
        string pose_name = iter->first;
        Vector2d pose_d = iter->second;

        cv::circle(image,cv::Point(pose_d[0],pose_d[1]),10,cv::Scalar(0, 255, 0),4,8);
        cv::putText(image, pose_name, cv::Point(pose_d[0]+15,pose_d[1]), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

        BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, knowledgegraph_object)
        {
            boost::property_tree::ptree vt = vtt.second;
            string name_kg = vt.get<string>("name");
            if(pose_name == name_kg )
            {
                add = 15;
                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, vt)
                {
                    if(v.second.get_value<std::string>() != name_kg)
                    {
                        cv::putText(image, v.first, cv::Point(pose_d[0]+10,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
                        cv::putText(image, ":", cv::Point(pose_d[0]+70,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
                        cv::putText(image, v.second.get_value<std::string>(), cv::Point(pose_d[0]+80,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
                        
                        add = add+15;
                        if(add%45 == 0)
                        {
                            r=255*abs(r-255);
                        }
                        if((add+15)%45 == 0)
                        {
                            g=255*abs(g-255);
                        }
                        if((add+30)%45 == 0)
                        {
                            b=255*abs(b-255);
                        }

                    }
                }
                break;  
            }
        }

        
    }

    // 二维码
    for(auto iter = object_2d_ar_pose.begin();iter != object_2d_ar_pose.end(); iter++)
    {
        string pose_name = iter->first;
        Vector2d pose_d = iter->second;

        cv::circle(image,cv::Point(pose_d[0],pose_d[1]),10,cv::Scalar(0, 255, 0),4,8);
        cv::putText(image, pose_name, cv::Point(pose_d[0]+15,pose_d[1]), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

        BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, knowledgegraph_object)
        {
            boost::property_tree::ptree vt = vtt.second;
            string name_kg = vt.get<string>("name");
            if(pose_name == name_kg )
            {
                add = 15;
                r=0,g=255,b=255;
                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, vt)
                {
                    if(v.second.get_value<std::string>() != name_kg)
                    {
                        cv::putText(image, v.first, cv::Point(pose_d[0]+15,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
                        cv::putText(image, ":", cv::Point(pose_d[0]+70,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
                        cv::putText(image, v.second.get_value<std::string>(), cv::Point(pose_d[0]+80,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
                        
                        add = add+15;
                        if(add%45 == 0)
                        {
                            r=255*abs(r-255);
                        }
                        if((add+15)%45 == 0)
                        {
                            g=255*abs(g-255);
                        }
                        if((add+30)%45 == 0)
                        {
                            b=255*abs(b-255);
                        }

                    }
                }
                break;
            }
        }

        
    }




    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub_pic.publish(msg);


}


void PARSE::tag_detections_mark(const apriltag_ros::AprilTagDetectionArray& msg)
{
    int t=0;
    int sum=0;
    // tf::TransformListener listener;

    std::stringstream ss;

    object_2d_ar_pose.clear();
    for(const apriltag_ros::AprilTagDetection& ar_marker : msg.detections)
    {
        ss.str("");
        ss << "tag_"<< ar_marker.id[0];

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

        string ar_pose_name;
        switch(ar_marker.id[0])
        {
            case 0 :
                ar_pose_name = "TV";
                break;
            case 1 :
                ar_pose_name = "desk";
                break;
            case 2 :
                ar_pose_name = "computer";
                break;
            case 3 :
                ar_pose_name = "chair";
                break;
            case 4 :
                ar_pose_name = "air_conditioner";
                break;
        }


        // if(object_2d_ar_pose.count(ar_pose_name) == 0)
        object_2d_ar_pose.insert(std::map<string,Vector2d>::value_type(ar_pose_name,Vector2d(ar_marker.tdposex[0],ar_marker.tdposey[0])));



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

