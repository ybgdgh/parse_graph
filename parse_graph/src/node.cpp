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
    pub_pg_show = nh_.advertise<sensor_msgs::Image>("pg_show", 10);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    // read AOG scene
    vg_AOG = PARSE::loadPoseFile("/home/dm/AOG_all.json");
    kitchen = vg_AOG.get_child("kitchen");
    conference = vg_AOG.get_child("conference");
    bathroom = vg_AOG.get_child("bathroom");
    office = vg_AOG.get_child("office");
    dining = vg_AOG.get_child("dining");
    living = vg_AOG.get_child("living");
    bedroom = vg_AOG.get_child("bedroom");

    office_object = office.get_child("objects");
    

    knowledgegraph = PARSE::loadPoseFile("/home/dm/knowledgegraph.json");
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
        dataFile.open("/home/dm/kg.json",ios::out);
        dataFile << sw.write(root);
        dataFile.close();

        cv::imwrite("/home/dm/pg_graph.png",pg_graph);
        cv::imwrite("/home/dm/pg_image_show.png",pg_image_show);


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

    // 统计同一物体出现多个概率的情况
    int count_object=0;
    bool happen_flag=false;
    id_object_p.clear();
    std::map<string,float> object_p;
    object_Bbox.clear();
    for(const darknet_ros_msgs::BoundingBox& Bbox : Bound_msg.bounding_boxes)
    {
        happen_flag=false;
        object_p.clear();
        if(!object_Bbox.empty())
        {
            for(int i=0;i<object_Bbox.size();i++)
            {
                if(Bbox.xmin == object_Bbox[i](0) && Bbox.xmax == object_Bbox[i](1) && Bbox.ymin == object_Bbox[i](2) && Bbox.ymax == object_Bbox[i](3))
                {
                    id_object_p[i].insert(std::map<string,float>::value_type(Bbox.Class,Bbox.probability));
                    cout << "happend!" << endl;
                    happen_flag=true;
                    break;
                }
               
            }
            
        }
        if(happen_flag==true) 
            continue;
        
        // 保存Bbox
        object_Bbox.push_back(Vector4d(Bbox.xmin,Bbox.xmax,Bbox.ymin,Bbox.ymax));
        // 保存类别对应的p
        object_p.insert(std::map<string,float>::value_type(Bbox.Class,Bbox.probability));
        // 保存该类别到object
        id_object_p.push_back(object_p);
    }

    // std::cout << "!!!!!!!!!!!!!!" << id_object_p.size() << endl;
    id_object_p_only.clear();
    for(int i=0;i<id_object_p.size();i++)
    {
        string map_name;
        float MAP=0;
        if(id_object_p[i].size()>1)
        {
            for(auto iter_ = id_object_p[i].begin();iter_ != id_object_p[i].end(); iter_++)
            {
                // cout << iter_->first << " : " << iter_->second;
                BOOST_FOREACH (boost::property_tree::ptree::value_type &v, office_object)
                {
                    if(iter_->first == v.first)
                    {
                        float office_object_p = office_object.get<float>(iter_->first);
                        
                        if(MAP < iter_->second*office_object_p)
                        {
                            MAP = iter_->second*office_object_p;
                            map_name = iter_->first;                        
                        }
                        cout << iter_->first << " : " << iter_->second << " * " << office_object_p << " = " << iter_->second*office_object_p << endl;                      
                        break;
                    }
                }   
            }
        }
        else
        {
            map_name = id_object_p[i].begin()->first;
        }

        id_object_p_only.push_back(map_name);
        // cout << i ;

        // cout << map_name << " : " << MAP;
        
        // cout << endl;
    }


    std::stringstream sss;
    object_2d_pose.clear();
    for(const darknet_ros_msgs::BoundingBox& Bbox : Bound_msg.bounding_boxes)
    {
        sss.str("");
        sss << Bbox.Class << '_' << Bbox.id;

        string name_darknet = sss.str();
        Vector3d S_darknet_object;

        Vector2d mean_point((Bbox.xmin+Bbox.xmax)/2,(Bbox.ymin+Bbox.ymax)/2);        
        double mean_depth = depth_pic.ptr<float>(int(mean_point[1]))[int(mean_point[0])]/1000.0;
        
        double x = (mean_point[0] - cy) * mean_depth / fy;
        double y = (mean_point[1] - cx) * mean_depth / fx;

        double x1 = (Bbox.ymin - cy) * mean_depth / fy;
        double y1 = (Bbox.xmin - cx) * mean_depth / fx;

        double x2 = (Bbox.ymax - cy) * mean_depth / fy;
        double y2 = (Bbox.xmax - cx) * mean_depth / fx;

        Vector3d point(x,y,mean_depth);

        S_darknet_object = T_base_to_camera * point;

        S_darknet_object[0] = float(round(S_darknet_object[0]*1000+0.5f)/1000.0);
        S_darknet_object[1] = float(round(S_darknet_object[1]*1000+0.5f)/1000.0);
        S_darknet_object[2] = float(round(S_darknet_object[2]*1000+0.5f)/1000.0);

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

            // the scale of boundingbox    
            Vector3d marker_scale;
            marker_scale << abs(x2-x1),max(abs(x2-x1),abs(y2-y1)),abs(y2-y1);
            
            // cout << "name : " << name_darknet << endl;
            // cout << "size of On_box : " << On_box.size() << endl;
            On_box.insert(std::map<string,Vector3d>::value_type(Bbox.Class,S_darknet_object));
            object_xyz.insert(std::map<string,Vector3d>::value_type(Bbox.Class,S_darknet_object));
            object_V.insert(std::map<string,Vector3d>::value_type(Bbox.Class,marker_scale));

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
                cout << name_on_object_ << endl;
                if(object_2d_ar_pose.count(name_on_object_)>0 || object_2d_pose.count(name_on_object_)>0)
                {
            
                    Vector3d Sbox_on_object = iter_->second;

                    // support                     
                    if(Sbox_on_object[2] > Sbox_support_object_[8]     // Z > Z_
                    && Sbox_on_object[0] > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                    && Sbox_on_object[0] < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                    && Sbox_on_object[1] > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                    && Sbox_on_object[1] < max(Sbox_support_object_[1],Sbox_support_object_[7]))    //Ymax > Ymin
                    {
                        // 连接两端的箭头线，有的可能存在ar里有的不是，因为实时清空所以实时显示更新
                        if(object_2d_ar_pose.count(name_on_object_)>0)
                            cv::arrowedLine(image, cv::Point(object_2d_ar_pose[name_support_object_][0], object_2d_ar_pose[name_support_object_][1]),
                            cv::Point(object_2d_ar_pose[name_on_object_][0], object_2d_ar_pose[name_on_object_][1]), cv::Scalar(0, 255, 0), 2, 4,0,0.1);
                        else if(object_2d_pose.count(name_on_object_)>0)
                            cv::arrowedLine(image, cv::Point(object_2d_ar_pose[name_support_object_][0], object_2d_ar_pose[name_support_object_][1]),
                            cv::Point(object_2d_pose[name_on_object_][0], object_2d_pose[name_on_object_][1]), cv::Scalar(0, 255, 0), 2, 4,0,0.1);

                        if(support_relationships.count(name_on_object_) == 0)
                            support_relationships.insert(std::map<string,string>::value_type(name_on_object_,name_support_object_));
                        

                        // 在on的面积上均匀100个点进行采样，判断多少位于support的面积之内
                        int support_sample_count=0;
                        int kkk=0;
                        for(float i=Sbox_on_object[0]-object_V[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V[name_on_object_][0]/2.0;i=i+(object_V[name_on_object_][0]/10.0))
                            for(float j=Sbox_on_object[1]-object_V[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V[name_on_object_][1]/2.0;j=j+(object_V[name_on_object_][1]/10.0))
                            {
                                kkk++;
                                if(i > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                                && i < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                                && j > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                                && j < max(Sbox_support_object_[1],Sbox_support_object_[7]))    //Ymax > Ymin
                                {
                                    support_sample_count++;
                                }
                            }
                            cout << "kkk : " << kkk << endl;
                        cout << "support_sample_count : " << support_sample_count << endl;
                        if(intersection_S.count(name_on_object_) == 0)
                            intersection_S.insert(map<string, float>::value_type(name_on_object_, float(support_sample_count/100.0)));
                   
                    }                
                

                    // contain
                    // 在on的体积上均匀1000个点进行采样，判断多少位于support的体积之内                    
                    int contain_sample_count=0;
                    int jjj =0;
                    for(float i=Sbox_on_object[0]-object_V[name_on_object_][0]/2.0;i<Sbox_on_object[0]+object_V[name_on_object_][0]/2.0;i=i+(object_V[name_on_object_][0]/10.0))
                        for(float j=Sbox_on_object[1]-object_V[name_on_object_][1]/2.0;j<Sbox_on_object[1]+object_V[name_on_object_][1]/2.0;j=j+(object_V[name_on_object_][1]/10.0))
                            for(float k = Sbox_on_object[2]-object_V[name_on_object_][2]/2 ; k < Sbox_on_object[2]+object_V[name_on_object_][2]/2 ; k = k+object_V[name_on_object_][2]/10.0)                    
                            {
                                jjj++;
                                if(i > min(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                                && i < max(Sbox_support_object_[0],Sbox_support_object_[6])     //Xmax > Xmin
                                && j > min(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                                && j < max(Sbox_support_object_[1],Sbox_support_object_[7])     //Ymax > Ymin
                                && k > Sbox_support_object_[8]-object_V[name_on_object_][2]   //z > Zmin
                                && k < Sbox_support_object_[8])  //z < Zmax
                                {
                                    contain_sample_count++;
                                }
                            }
                    cout << "jjj : " << jjj << endl;                    
                    cout << "contain_sample_count : " << contain_sample_count << endl;
                    if(contain_sample_count>500)
                    {
                        // 连接两端的箭头线，有的可能存在ar里有的不是，因为实时清空所以实时显示更新
                        if(object_2d_ar_pose.count(name_on_object_)>0)
                            cv::arrowedLine(image, cv::Point(object_2d_ar_pose[name_support_object_][0], object_2d_ar_pose[name_support_object_][1]),
                            cv::Point(object_2d_ar_pose[name_on_object_][0], object_2d_ar_pose[name_on_object_][1]), cv::Scalar(0, 0, 255), 2, 4,0,0.1);
                        else if(object_2d_pose.count(name_on_object_)>0)
                            cv::arrowedLine(image, cv::Point(object_2d_ar_pose[name_support_object_][0], object_2d_ar_pose[name_support_object_][1]),
                            cv::Point(object_2d_pose[name_on_object_][0], object_2d_pose[name_on_object_][1]), cv::Scalar(0, 0, 255), 2, 4,0,0.1);

                        if(contian_relationships.count(name_on_object_) == 0)
                            contian_relationships.insert(std::map<string,string>::value_type(name_on_object_,name_support_object_));
                        
                        if(intersection_V.count(name_on_object_) == 0)
                            intersection_V.insert(map<string, float>::value_type(name_on_object_, float(contain_sample_count/1000.0)));

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

    pg_image_show = image;


    int rows = 600;
    int cols = 200+(Support_box.size() + On_box.size())*200;
    int x_ = cols/2;
    int y_ = 50;
    int count = 0;
    std::stringstream ss;
    object_pg_pose.clear();
    if(cols > 0)
    {
        cv::Mat image_pg(rows, cols, CV_8UC3, Scalar(255,255,255));

        string pg_name = knowledgegraph.get<string>("name");
        
        Draw_PG::draw_node(image_pg,cols/2,y_,"pg_name");

        for(auto iter = object_xyz.begin();iter != object_xyz.end(); iter++)
        {
            string object_name = iter->first;
            Vector3d object_xyz_ = iter->second;
            x_ = x_ + count*180*pow(-1,count);
            Draw_PG::draw_node(image_pg,x_,y_+150,object_name);
            Draw_PG::draw_arrow(image_pg,cols/2,y_,x_,y_+150);

            // 属性节点
            cv::Rect rect(x_-10-30, y_+300, 20,20);//左上坐标（x,y）和矩形的长(x)宽(y)
            cv::rectangle(image_pg, rect, cv::Scalar(0, 160, 255),-1, cv::LINE_8,0);//绘制填充矩形
            Draw_PG::draw_attribute_arrow(image_pg,x_,y_+150+15,x_-30, y_+300);

            // 地址节点
            Draw_PG::draw_triangle(image_pg, x_+30, y_+300);
            Draw_PG::draw_attribute_arrow(image_pg,x_,y_+150+15,x_+30, y_+300);
            object_pg_pose.insert(std::map<string,Vector2d>::value_type(object_name,Vector2d(x_+30,y_+300+20)));
            

            // xyz
            cv::Rect rect1(x_-60-30, y_+420, 120,20);//左上坐标（x,y）和矩形的长(x)宽(y)
            cv::rectangle(image_pg, rect1, cv::Scalar(0, 255, 0), 1, cv::LINE_8,0);//绘制矩形
            Draw_PG::draw_attribute_arrow(image_pg,x_-30, y_+320,x_-30, y_+420);

            ss.str("");
            ss << object_xyz_[0] << "," << object_xyz_[1] << "," << object_xyz_[2];
            cv::putText(image_pg, ss.str(), cv::Point(x_-60-27,y_+435), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, 8);


            // BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, knowledgegraph_object)
            // {
            //     boost::property_tree::ptree vt = vtt.second;
            //     string name_kg = vt.get<string>("name");
            //     if(object_name == name_kg)
            //     {
            //         // Draw_PG::draw_attribute_node(image_pg, int x, int y, std::string node_name)
            //     }
     
            // }

            count++;
            
        }


        // 连接节点间的关系
        for(auto iter = support_relationships.begin();iter != support_relationships.end(); iter++)
        {
            
            std::string name_on = iter->first;
            std::string name_support = iter->second;
           
            Vector2d start = object_pg_pose[name_on];
            Vector2d end = object_pg_pose[name_support];
            Vector2d meddle = Vector2d((start[0]+end[0])/2,start[1]+50);
            Draw_PG::draw_Arc(image_pg,cv::Point(start[0],start[1]),cv::Point(meddle[0],meddle[1]),cv::Point(end[0],end[1]),2);

        }
        for(auto iter = contian_relationships.begin();iter != contian_relationships.end(); iter++)
        {
            
            std::string name_in = iter->first;
            std::string name_out = iter->second;
           
            Vector2d start = object_pg_pose[name_in];
            Vector2d end = object_pg_pose[name_out];
            Vector2d meddle = Vector2d((start[0]+end[0])/2,start[1]+50);
            Draw_PG::draw_Arc(image_pg,cv::Point(start[0],start[1]),cv::Point(meddle[0],meddle[1]),cv::Point(end[0],end[1]),2);

        }

        
        
        

        // 发布pg图
        pg_graph = image_pg;
        sensor_msgs::ImagePtr pg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_pg).toImageMsg();
        pub_pg_show.publish(pg_msg);

    }

    // 发布带节点关系的图像
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub_pic.publish(msg);

    // visualization_msgs::MarkerArray markerarray;

    // 发布object的scale
    std::stringstream sss;
    int count_marker=0;
    // cout << object_V.size() << endl;
    for(auto iter = object_V.begin();iter != object_V.end(); iter++)
    {
        Vector3d marker_scale_ = iter->second;
        Vector3d marker_pose = object_xyz[iter->first];
        // cout << "marker_pose: " << marker_pose << endl;
        sss.str("");
        sss << "/" << iter->first;
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        sss.str("");
        sss << "basic_shapes" << count_marker;
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = sss.str();
        marker.id = count_marker;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

        if(iter->first == "cup" || iter->first == "mouse")
        {
            marker.pose.position.x = marker_pose[0];
            marker.pose.position.y = marker_pose[1];
            marker.pose.position.z = marker_pose[2];
        }
        else
        {
            marker.pose.position.x = marker_pose[0];
            marker.pose.position.y = marker_pose[1];
            marker.pose.position.z = marker_pose[2]-marker_scale_[2]/2;

        }
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = marker_scale_[0];
        marker.scale.y = marker_scale_[0];
        marker.scale.z = marker_scale_[2];

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.5f;//1.0f;
        marker.color.g = 0.5f;//1.0f;
        marker.color.b = 0.5f;//1.0f;
        marker.color.a = 0.5f;//1.0;

        marker.lifetime = ros::Duration();

        // markerarray.markers.push_back(marker);
        marker_pub.publish(marker);
        count_marker++;
    }
    
    // marker_pub.publish(markerarray);

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

        Vector3d marker_scale;

        // TV
        if(ar_marker.id[0] == 0 && TV == false)
        {
            TV = true;
            name = "TV";

            Eigen::Vector3d trans(trans_object[0],trans_object[1],trans_object[2]);    

            S_on_object << trans(0),trans(1),trans_object[2];     
            
            marker_scale << 0.5,0.3,0.1;

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

            marker_scale << 1,2,1;
            
            support_flag = true;
            // cout << "T_base_to_apri :" << T_base_to_apri.matrix() << endl;
            // cout << "desk : " << S_support_object << endl;
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

            marker_scale << 0.1,0.3,0.3;
            
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

            marker_scale << 0.2,0.2,-0.5;
            
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

            marker_scale << 0.1,1,0.1;
            
            on_flag = true;
            
        }
        else if(ar_marker.id[0] == 5 && desk1 == false)
        {
            desk1 = true;
            name = "desk1";

            Eigen::Vector3d t2(0,0,0.5);
            Eigen::Vector3d trans = T_base_to_apri * t2;    

            S_on_object << trans(0),trans(1),trans_object[2];
            // cout << "S_on_object" << S_on_object << endl;
            // cout << "trans" << trans << endl;
            marker_scale << 0.1,0.3,0.3;
            
            on_flag = true;
        }
        else if(ar_marker.id[0] == 6 && desk2 == false)
        {
            desk2 = true;
            name = "desk2";

            Eigen::Vector3d t2(0,0,0.5);
            Eigen::Vector3d trans = T_base_to_apri * t2;    

            S_on_object << trans(0),trans(1),trans_object[2];
            marker_scale << 0.1,0.3,0.3;

            on_flag = true;
        }
        else if(ar_marker.id[0] == 7 && desk3 == false)
        {
            desk1 = true;
            name = "desk3";

            Eigen::Vector3d t2(0,0,0.5);
            Eigen::Vector3d trans = T_base_to_apri * t2;    

            S_on_object << trans(0),trans(1),trans_object[2];
            marker_scale << 0.1,0.3,0.3;

            on_flag = true;
        }
        
        else 
        {
            break;
        }

        if(support_flag == true)
        {

            support_flag = false;
            Support_box.insert(std::map<string,Vector9d>::value_type(name,S_support_object));

            S_support_object[0] = double((int)(S_support_object[0]*1000+0.5f)/1000.0);
            S_support_object[1] = double((int)(S_support_object[1]*1000+0.5f)/1000.0);
            S_support_object[8] = double((int)(S_support_object[8]*1000+0.5f)/1000.0);

            object_xyz.insert(std::map<string,Vector3d>::value_type(name,Vector3d(S_support_object[0],S_support_object[1],S_support_object[8])));
            object_V.insert(std::map<string,Vector3d>::value_type(name,marker_scale));
            
        }
        else if(on_flag == true)
        {
            on_flag = false;

            On_box.insert(std::map<string,Vector3d>::value_type(name,S_on_object));
            S_on_object[0] = double((int)(S_on_object[0]*1000+0.5f)/1000.0);
            S_on_object[1] = double((int)(S_on_object[1]*1000+0.5f)/1000.0);
            S_on_object[2] = double((int)(S_on_object[2]*1000+0.5f)/1000.0);

            object_xyz.insert(std::map<string,Vector3d>::value_type(name,S_on_object));
            object_V.insert(std::map<string,Vector3d>::value_type(name,marker_scale));
            
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

