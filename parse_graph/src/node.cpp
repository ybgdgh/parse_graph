/**
 * Copyright (c) 2019, The Parse_graph author.
 * All rights reserved.
 *
 * Author:dm
 * This is the node file for subscribe and publish the ros node.
 */

#include "node.h"

bool Base_Flag,Scannet_Flag,Save_Image_Flag;
string Yolo_Base[10] = { 
                        "cup",
                        "mouse",
                        "chair",
                        "tvmonitor",
                        "keyboard",
                        "bottle",
                        "book",
                        "bowl",
                        "phone",
                        "remote"
                    };

int output_image_count=0;


// 计算坐标对应的color
string compute_color(cv::Mat img_pg, int x, int y)
{
    Scalar p = img_pg.at<Vec3b>(x, y);
    string color;
    bool b_color = p[0]>127 ? true:false;
    bool g_color = p[1]>127 ? true:false;
    bool r_color = p[2]>127 ? true:false;

    if(!b_color && !g_color && !r_color)
    {
        color = "black";
    }
    else if(!b_color && !g_color && r_color)
    {
        color = "red";
    }
    else if(!b_color && g_color && !r_color)
    {
        color = "blue";
    }
    else if(b_color && !g_color && !r_color)
    {
        color = "green";
    }
    else if(!b_color && g_color && r_color)
    {
        color = "yellow";
    }
    else if(b_color && !g_color && r_color)
    {
        color = "purple";
    }
    else if(b_color && g_color && !r_color)
    {
        color = "Indigo";
    }
    else if(b_color && g_color && r_color)
    {
        color = "white";
    }

    return color;

}


Parse_Node::Parse_Node(ros::NodeHandle nh,ros::NodeHandle np)
:nh_(nh),
 np_(np)
{
    nh_.param<bool>("Base_Flag",Base_Flag,false);
    nh_.param<bool>("Scannet_Flag",Scannet_Flag,false);
    nh_.param<bool>("Save_Image_Flag",Save_Image_Flag,false);
    
    //订阅odom和激光的话题
    sub_ar_track = nh_.subscribe("tag_detections", 10, &Parse_Node::tag_detections_mark,this);
    sub_darknet = nh_.subscribe("darknet_ros/bounding_boxes", 10, &Parse_Node::darknet_Bbox,this);
    
    sub_CameraInfo = nh_.subscribe("camera/aligned_depth_to_color/camera_info", 10, &Parse_Node::CameraInfo,this);
    sub_depth_camera = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 10, &Parse_Node::depth_Callback,this);

    sub_color_camera = nh_.subscribe("/camera/color/image_raw", 10, &Parse_Node::color_Callback,this);
    sub_dark_detect_camera = nh_.subscribe("/darknet_ros/detection_image", 10, &Parse_Node::dark_detect_Callback,this);

    pub_pic = nh_.advertise<sensor_msgs::Image>("kg_camera", 10);
    pub_pg_show = nh_.advertise<sensor_msgs::Image>("pg_show", 10);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    // read AOG scene
    vg_AOG = Parse_Node::loadPoseFile("/home/ybg/AOG_all.json");
    // kitchen = vg_AOG.get_child("kitchen");
    // conference = vg_AOG.get_child("conference");
    // bathroom = vg_AOG.get_child("bathroom");
    // office = vg_AOG.get_child("office");
    // dining = vg_AOG.get_child("dining");
    // living = vg_AOG.get_child("living");
    // bedroom = vg_AOG.get_child("bedroom");

    // office_object = office.get_child("objects");
    // office_relationships = office.get_child("relationships");

    // init current scene
    current_scene = "conference";
    current_scene_tree = vg_AOG.get_child(current_scene);
    
    
    // attribute
    knowledgegraph = Parse_Node::loadPoseFile("/home/ybg/knowledgegraph.json");
    knowledgegraph_object = knowledgegraph.get_child("object");

    //inite the Flag of active is full!
    KFG_Active_Finish = false;
    KFG_AR_Active_Finish = false;
    KFG_Number = 50;


    //调用python接口
    cout << "python init!" << endl;
    Py_Initialize();
    PyRun_SimpleString("import sys");
    cout << "python initing" << endl;
    
    std::string dirs = std::string("sys.path.append('/home/ybg/ROS_code/catkin_vision/src/parse_graph/src/')");
    PyRun_SimpleString(dirs.c_str());

    PyObject *module = PyString_FromString("vis_pg");
    pModule = PyImport_Import(module);      //Python文件名
    if(!pModule) //failed
    {
        std::cout << "Get python module failed." << std::endl;
        return ;
    }
    pFunc_scene = PyObject_GetAttrString(pModule, "add_scene"); //Python文件中的函数名
    pFunc_node = PyObject_GetAttrString(pModule, "add_node"); //Python文件中的函数名
    pFunc_rela = PyObject_GetAttrString(pModule, "add_rela"); //Python文件中的函数名
    pFunc_viz = PyObject_GetAttrString(pModule, "viz_pg"); //Python文件中的函数名
    cout << "python init finish!" << endl;


    std::cout << "python init finish" << std::endl;
   
}

Parse_Node::~Parse_Node()
{
    Json::Value root;
    
    for(auto iter_ = Scene_Relation.begin();iter_ != Scene_Relation.end(); iter_++)
    {
        Json::Value Scene;
        Scene["name"] = iter_->first;
        std::vector<string> simple_scene = iter_->second;
        Json::Value attribute;

        // support object
        for(auto iter = Support_box.begin();iter != Support_box.end(); iter++)
        {
            string name_support_object = iter->first;

            if(std::find(simple_scene.begin(), simple_scene.end(), name_support_object) == simple_scene.end()) 
                continue;

            Vector9d Sbox_support_object = iter->second;

            Json::Value object;
            object["name"] = Json::Value(name_support_object);
            object.removeMember("XYZ");
            object["XYZ"].append(Sbox_support_object[0]);
            object["XYZ"].append(Sbox_support_object[1]);
            object["XYZ"].append(Sbox_support_object[8]);

    
            attribute["color"]=Json::Value(Rgb_color[name_support_object]);
            attribute.removeMember("size");
            attribute["size"].append(float((int)(object_V[name_support_object][0]*1000+0.5f)/1000.0));
            attribute["size"].append(float((int)(object_V[name_support_object][1]*1000+0.5f)/1000.0));
            attribute["size"].append(float((int)(object_V[name_support_object][2]*1000+0.5f)/1000.0));
  
            object["attribute"] = attribute;
     
            Scene["object"].append(object);

        }

        // on object
        if(Base_Flag)
        {
            for(auto iter = On_box_local.begin();iter != On_box_local.end(); iter++)
            {
                string name_on_object = iter->first;

                if(std::find(simple_scene.begin(), simple_scene.end(), name_on_object) == simple_scene.end()) 
                    continue;

                Vector3d Sbox_on_object = iter->second;

                Json::Value object;
                object["name"] = Json::Value(name_on_object);
                object.removeMember("XYZ");
                object["XYZ"].append(Sbox_on_object[0]);
                object["XYZ"].append(Sbox_on_object[1]);
                object["XYZ"].append(Sbox_on_object[2]);

                if(support_relationships_local.count(name_on_object) > 0)
                {
                    object["on"] = Json::Value(support_relationships_local[name_on_object]);
                }

                if(contian_relationships_local.count(name_on_object) > 0)
                {
                    object["contain"] = Json::Value(contian_relationships_local[name_on_object]);
                }
                
                attribute["color"]=Json::Value(Rgb_color[name_on_object]);
                attribute.removeMember("size");
                attribute["size"].append(float((int)(object_V[name_on_object][0]*1000+0.5f)/1000.0));
                attribute["size"].append(float((int)(object_V[name_on_object][1]*1000+0.5f)/1000.0));
                attribute["size"].append(float((int)(object_V[name_on_object][2]*1000+0.5f)/1000.0));

                object["attribute"] = attribute;

                Scene["object"].append(object);
            }

        }
        else
        {
            for(auto iter = On_box.begin();iter != On_box.end(); iter++)
            {
                string name_on_object = iter->first;

                if(std::find(simple_scene.begin(), simple_scene.end(), name_on_object) == simple_scene.end()) 
                    continue;

                Vector3d Sbox_on_object = iter->second;

                Json::Value object;
                object["name"] = Json::Value(name_on_object);
                object.removeMember("XYZ");
                object["XYZ"].append(Sbox_on_object[0]);
                object["XYZ"].append(Sbox_on_object[1]);
                object["XYZ"].append(Sbox_on_object[2]);

                if(support_relationships.count(name_on_object) > 0)
                {
                    object["on"] = Json::Value(support_relationships[name_on_object]);
                }

                if(contian_relationships.count(name_on_object) > 0)
                {
                    object["contain"] = Json::Value(contian_relationships[name_on_object]);
                }
                
                attribute["color"]=Json::Value(Rgb_color[name_on_object]);
                attribute.removeMember("size");
                attribute["size"].append(float((int)(object_V[name_on_object][0]*1000+0.5f)/1000.0));
                attribute["size"].append(float((int)(object_V[name_on_object][1]*1000+0.5f)/1000.0));
                attribute["size"].append(float((int)(object_V[name_on_object][2]*1000+0.5f)/1000.0));

                object["attribute"] = attribute;

                Scene["object"].append(object);
            }

        }

        root.append(Scene);
    }


    // cout << "StyledWriter:" << endl;
    Json::StyledWriter sw;
    // cout << sw.write(root) << endl << endl;

    // 输出到文件
    ofstream dataFile;//记录数据
    dataFile.open("/home/ybg/kg.json",ios::out);
    dataFile << sw.write(root);
    dataFile.close();

    cv::imwrite("/home/ybg/pg_graph.png",pg_graph);
    cv::imwrite("/home/ybg/pg_image_show.png",pg_image_show);

    // python 调用结束
    Py_Finalize();


}

void Parse_Node::Publishtf(Vector3d point,std::string tf1, std::string tf2)
{
    // pub the tf 
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z = point[2];
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/tf";
    tf::Stamped<tf::Transform> transform;
    tf::poseStampedMsgToTF(pose, transform);
    tf_pub_.sendTransform(tf::StampedTransform(transform,
                                                transform.stamp_,
                                                tf1,
                                                tf2));

}

void Parse_Node::Listentf(std::string tf1, std::string tf2,Eigen::Isometry3d& T)
{
    Eigen::Vector3d trans;
    Eigen::Quaterniond quat;

    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(tf1, tf2,ros::Time(0), transform);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute ar pose, skipping scan (%s)", e.what());
        ros::Duration(1.0).sleep();
        return ;
    }
    trans << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();

    // cout << "trans : " << trans << endl;

    quat.w() = transform.getRotation().getW();
    quat.x() = transform.getRotation().getX();
    quat.y() = transform.getRotation().getY();
    quat.z() = transform.getRotation().getZ();

    T = Eigen::Isometry3d::Identity();
    T.rotate ( quat );
    T.pretranslate ( trans );

}




void Parse_Node::darknet_Bbox(const darknet_ros_msgs::BoundingBoxes& Bound_msg)
{

    //定义机器人本体到相机之间的变换
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
        ROS_WARN("Failed to compute dark pose, skipping scan (%s)", e.what());
        ros::Duration(1.0).sleep();
        return ;
    }
    trans_Bbox << transform_Bbox.getOrigin().x(),transform_Bbox.getOrigin().y(),transform_Bbox.getOrigin().z();

    // cout << "trans : " << trans_Bbox << endl;

    quat_Bbox.w() = transform_Bbox.getRotation().getW();
    quat_Bbox.x() = transform_Bbox.getRotation().getX();
    quat_Bbox.y() = transform_Bbox.getRotation().getY();
    quat_Bbox.z() = transform_Bbox.getRotation().getZ();

    Eigen::Isometry3d T_base_to_camera = Eigen::Isometry3d::Identity();
    T_base_to_camera = Eigen::Isometry3d::Identity();
    T_base_to_camera.rotate ( quat_Bbox );
    T_base_to_camera.pretranslate ( trans_Bbox );
    

    // compute the MAP of object
    id_object_p_only.clear();
    current_object = current_scene_tree.get_child("objects");
    Map_Compute::Compute_Object_Map(current_object,Bound_msg,id_object_p_only);

    // cout << "id_object_p_only size : " << id_object_p_only.size() << endl;

    std::stringstream sss;
    object_2d_pose.clear();
    object_P.clear();
    // keyframe group in one image
    std::vector<std::tuple<string,Vector3d,Vector3d>> KFG_active_oneimage;
    // 判断同一帧图像中是否与同一物体检测重复
    vector<string> same_detect;
    for(int i=0;i<id_object_p_only.size();i++)
    {
        // keyframe group in one object
        std::tuple<string,Vector3d,Vector3d> KFG_active_oneobject;

        string ob_name;
        float ob_sum_p;
        Vector4d Bbox;
        std::tie(ob_name,Bbox,ob_sum_p)=id_object_p_only[i];
        // cout << ob_name << " : " << ob_sum_p << endl;

        Vector2d mean_point((Bbox[0]+Bbox[1])/2,(Bbox[2]+Bbox[3])/2);        
        double mean_depth = depth_pic.ptr<float>(int(mean_point[1]))[int(mean_point[0])]/1000.0;
        // 不知咋搞的有时候会返回0，导致坐标跑到相机上
        if(mean_depth == 0) continue;
        // cout << "Bbox : " << Bbox.matrix() << endl;
        // cout << "mean_depth : " << mean_depth << endl;

        double x = (mean_point[0] - cy) * mean_depth / fy;
        double y = (mean_point[1] - cx) * mean_depth / fx;

        double x1 = (Bbox[2] - cy) * mean_depth / fy;
        double y1 = (Bbox[0] - cx) * mean_depth / fx;

        double x2 = (Bbox[3] - cy) * mean_depth / fy;
        double y2 = (Bbox[1] - cx) * mean_depth / fx;

        Vector3d point(x,y,mean_depth);

        Vector3d S_darknet_object;
        S_darknet_object = T_base_to_camera * point;

        // cout << "point : " << point.matrix() << endl;
        // cout << "T_base_to_camera :" << endl << T_base_to_camera.matrix() << endl;

        string name_darknet;
        bool Same_flag=false;
        std::vector<string> vecArr(Yolo_Base, Yolo_Base+10); 
	    std::vector<string>::iterator ite = find(vecArr.begin(),vecArr.end(),ob_name);
	    if(ite!=vecArr.end())
        {
            // 之前无此class，则创建class类别，添加id 0
            if(class_id.count(ob_name) == 0)
            {
                sss.str("");
                sss << ob_name << '_' << 0;
                name_darknet = sss.str();
                cout << "first : " << name_darknet << " : " << ob_sum_p << endl;
                std::vector<string> class_id_first;
                class_id_first.push_back(name_darknet);
                class_id.insert(std::map<string,std::vector<string>>::value_type(ob_name,class_id_first));
            }
            // 再有相同class的object时，首先判断是否为之前的id，若不是则push新id
            else if(class_id.count(ob_name) > 0)
            {
                float distance_xyz;
                float min_distance = 100000;
                for(int k=0;k<class_id[ob_name].size();k++)
                {
                    // 判断三维坐标是否一致(之间距离是否大于最大边长)
                    string other_class_name = class_id[ob_name][k];
                    if(object_V_local.count(other_class_name)>0)
                    {
                        float a=object_V_local[other_class_name][0];
                        float b=object_V_local[other_class_name][1];
                        float c=object_V_local[other_class_name][2];
                        float max_distance = (a>=b&&a>=c)?a:(b>=a&&b>=c)?b:c;
                        
                        // 0.25为阈值，调整max_distance参数
                        if(max_distance < 0.20) max_distance=max_distance*2;
                        
                        if(abs(S_darknet_object[0]-object_xyz_local[other_class_name][0])<max_distance &&
                            abs(S_darknet_object[1]-object_xyz_local[other_class_name][1])<max_distance &&
                            abs(S_darknet_object[2]-object_xyz_local[other_class_name][2])<max_distance &&
                            std::find(same_detect.begin(), same_detect.end(), other_class_name) == same_detect.end())
                        {               
                            distance_xyz = abs(S_darknet_object[0]-object_xyz_local[other_class_name][0])+
                                                    abs(S_darknet_object[1]-object_xyz_local[other_class_name][1]);
                            if(distance_xyz < min_distance)
                            {               
                                min_distance = distance_xyz;
                                name_darknet = other_class_name;  
                                Same_flag=true;
                                
                            }
                        }
                    }
                    
                }
                if(Same_flag)
                {
                    same_detect.push_back(name_darknet);  
                }

                if(!Same_flag)
                {
                    sss.str("");
                    sss << ob_name << '_' << class_id[ob_name].size();
                    name_darknet = sss.str();
                    class_id[ob_name].push_back(name_darknet);
                    cout << "second : " << name_darknet << " : " << ob_sum_p << endl;
                    
                }
                
            }

            // publish the tf of cmaera and darktag
            Parse_Node::Publishtf(S_darknet_object,"map",name_darknet);

            Vector2d d_pose((Bbox[0]+Bbox[1])/2,(Bbox[2]+Bbox[3])/2);

            if(object_2d_pose.count(name_darknet) == 0)
                object_2d_pose.insert(std::map<string,Vector2d>::value_type(name_darknet,d_pose));

          
            // the scale of boundingbox    
            Vector3d marker_scale = Vector3d::Zero();
            marker_scale[0] = (abs(x2-x1)>0?abs(x2-x1):0.01);
            marker_scale[1] = (min(abs(x2-x1),abs(y2-y1))>0?min(abs(x2-x1),abs(y2-y1)):0.01);
            marker_scale[2] = (abs(y2-y1)>0?abs(y2-y1):0.01);

            // cout << name_darknet << ": " << marker_scale.matrix() << endl;

            if(On_box_local.count(name_darknet) == 0)
            {           
                On_box_local.insert(std::map<string,Vector3d>::value_type(name_darknet,S_darknet_object));
                object_xyz_local.insert(std::map<string,Vector3d>::value_type(name_darknet,S_darknet_object));
                object_V_local.insert(std::map<string,Vector3d>::value_type(name_darknet,marker_scale));       
                object_P.insert(std::map<string,float>::value_type(name_darknet,ob_sum_p));     
                if(Base_Flag) Scene_Relation[current_scene].push_back(name_darknet);            
                  
            }
            else if(On_box_local.count(name_darknet) > 0)
            {    

                On_box_local[name_darknet]=S_darknet_object;
                object_xyz_local[name_darknet]=S_darknet_object;
                object_V_local[name_darknet]=marker_scale;
                object_P[name_darknet]=ob_sum_p;                       
            }

            // cout << "marker_scale : " << name_darknet << " " << marker_scale << endl;
            // cout << "size of On_box : " << On_box.size() << endl;

            if(!Base_Flag)
            {
                KFG_active_oneobject=make_tuple(name_darknet,S_darknet_object,marker_scale);
                KFG_active_oneimage.push_back(KFG_active_oneobject);
            }
            
        }

       
  
    }
    if(!Base_Flag) KFG_active.push_back(KFG_active_oneimage);

    // transform active to optimize
    if(KFG_active.size() >= 50)
    {
        KFG_Active_Finish = true;
        // clear KFG_optimize and insert KFG_active
        KFG_optimize.assign(KFG_active.begin(),KFG_active.end());
        // clear KFG_active and insert the after half of KFG_optimize
        KFG_active.assign(KFG_optimize.begin()+KFG_Number/2,KFG_optimize.end());
    }
}



void Parse_Node::tag_detections_mark(const apriltag_ros::AprilTagDetectionArray& msg)
{
    int t=0;
    int sum=0;
    bool support_flag = false, on_flag = false, scene_flag = false;

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
            return ;
        }
        trans_object << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();

        quat_object.w() = transform.getRotation().getW();
        quat_object.x() = transform.getRotation().getX();
        quat_object.y() = transform.getRotation().getY();
        quat_object.z() = transform.getRotation().getZ();

        // ROS_INFO("%d",  ar_marker.id[0]);

        Eigen::Isometry3d T_base_to_apri = Eigen::Isometry3d::Identity();
        T_base_to_apri = Eigen::Isometry3d::Identity();
        T_base_to_apri.rotate(quat_object.toRotationMatrix());
        T_base_to_apri.pretranslate(trans_object);

       
        // add the 2d pose to map
        // if(object_2d_ar_pose.count(ar_pose_name) == 0)


        string name;
        Vector9d S_support_object;
        Vector3d S_on_object;
        Vector3d marker_scale;
        string Ar_color;

        // TV
        if(ar_marker.id[0] == 0)
        {
            name = "desk_0";

            Eigen::Vector3d t2(0,-2,0);
            Eigen::Vector3d t3(-1,0,0);
            Eigen::Vector3d t4(-1,-2,0);

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

            Ar_color = "white";

            support_flag = true;
        }
        // desk support function && desk == false
        else if(ar_marker.id[0] == 1 )
        {
            name = "desk_1";

            Eigen::Vector3d t2(1,0,0);
            Eigen::Vector3d t3(0,-2,0);
            Eigen::Vector3d t4(1,-2,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            // Parse_Node::Publishtf(trans2,"map","desk_2");
            // Parse_Node::Publishtf(trans3,"map","desk_3");
            // Parse_Node::Publishtf(trans4,"map","desk_4");

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 1,2,1;

            Ar_color = "yellow";            
            
            support_flag = true;
            // cout << "T_base_to_apri :" << T_base_to_apri.matrix() << endl;
            // cout << "desk : " << S_support_object << endl;
        }
        // computer support_function
        else if(ar_marker.id[0] == 2)
        {
            name = "desk_2";

            Eigen::Vector3d t1(-0.2,0,0);
            Eigen::Vector3d t2(1,0,0);
            Eigen::Vector3d t3(-0.2,-2,0);
            Eigen::Vector3d t4(1,-2,0);

            Eigen::Vector3d trans(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans1 = T_base_to_apri * t1; 

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 1,2,1;
            
            Ar_color = "white";
            
            support_flag = true;
            
        }
        // chair support_function
        else if(ar_marker.id[0] == 3)
        {
            name = "office";

            Eigen::Vector3d t2(0,0,-4);
            Eigen::Vector3d t3(5,0,0);
            Eigen::Vector3d t4(5,0,-4);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            Parse_Node::Publishtf(trans2,"map","office_2");
            Parse_Node::Publishtf(trans3,"map","office_3");
            Parse_Node::Publishtf(trans4,"map","office_4");

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 5,4,2;
            
            scene_flag = true;            

            // cout << "chair : " << S_object << endl; 
        }
        // air_conditioner
        else if(ar_marker.id[0] == 4)
        {
            name = "desk_4";

            Eigen::Vector3d t2(0.5,0,0);
            Eigen::Vector3d t3(0,-1,0);
            Eigen::Vector3d t4(0.5,-1,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 0.5,1,1;
            
            Ar_color = "black";
            
            support_flag = true;
            
        }
        else if(ar_marker.id[0] == 5)
        {//chugai
            name = "cabinet";

            Eigen::Vector3d t1(0.5,0,0);
            Eigen::Vector3d t2(-4,0,0);
            Eigen::Vector3d t3(0.5,0,-1);
            Eigen::Vector3d t4(-4,0,-1);

            Eigen::Vector3d trans(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans1 = T_base_to_apri * t1; 

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            Parse_Node::Publishtf(trans1,"map","cabinet_1");
            Parse_Node::Publishtf(trans2,"map","cabinet_2");
            Parse_Node::Publishtf(trans3,"map","cabinet_3");
            Parse_Node::Publishtf(trans4,"map","cabinet_4");

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 4,0.5,3;

            Ar_color = "white";            
            
            support_flag = true;
        }
        else if(ar_marker.id[0] == 6)
        {
            name = "conference";

            Eigen::Vector3d t2(0,0,-4);
            Eigen::Vector3d t3(4,0,0);
            Eigen::Vector3d t4(4,0,-4);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            Parse_Node::Publishtf(trans2,"map","conference_2");
            Parse_Node::Publishtf(trans3,"map","conference_3");
            Parse_Node::Publishtf(trans4,"map","conference_4");

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 4,4,2;

            scene_flag = true;
        }
        else if(ar_marker.id[0] == 7)
        {
            name = "sofa";

            Eigen::Vector3d t2(0,-0.5,0);
            Eigen::Vector3d t3(-2,0,0);
            Eigen::Vector3d t4(-2,-0.5,0);

            Eigen::Vector3d trans1(trans_object[0],trans_object[1],trans_object[2]);    

            Eigen::Vector3d trans2 = T_base_to_apri * t2; 

            Eigen::Vector3d trans3 = T_base_to_apri * t3;

            Eigen::Vector3d trans4 = T_base_to_apri * t4;

            S_support_object << trans1(0),trans1(1),
                        trans2(0),trans2(1),
                        trans3(0),trans3(1),
                        trans4(0),trans4(1),
                        trans_object[2];

            marker_scale << 2,0.5,1;

            Ar_color = "white";
            
            support_flag = true;
        }
        
        else 
        {
            continue;
        }

        object_2d_ar_pose.insert(std::map<string,Vector2d>::value_type(name,Vector2d(ar_marker.tdposex[0],ar_marker.tdposey[0])));



        if(scene_flag == true)
        {
            scene_flag = false;

            // record the current scene calss
            current_scene_tree = vg_AOG.get_child(name);
            current_scene = name;

            if(Scene_box.count(name) > 0)
            {
                Scene_box[name]=S_support_object;
                object_xyz[name]=Vector3d(S_support_object[0],S_support_object[1],S_support_object[8]);
                object_V[name]=marker_scale;
                continue;
            }
            Scene_box.insert(std::map<string,Vector9d>::value_type(name,S_support_object));
            object_xyz.insert(std::map<string,Vector3d>::value_type(name,Vector3d(S_support_object[0],S_support_object[1],S_support_object[8])));
            object_V.insert(std::map<string,Vector3d>::value_type(name,marker_scale));
            std::vector<string> scene_objects;            
            Scene_Relation.insert(make_pair(current_scene,scene_objects));
            if(!Base_Flag)KFG_AR_Active_Finish = true;
            
        }
        else if(support_flag == true)
        {
            support_flag = false;

            if(Support_box.count(name) > 0)
            {
                Support_box[name]=S_support_object;
                object_xyz[name]=Vector3d(S_support_object[0],S_support_object[1],S_support_object[8]);
                object_V[name]=marker_scale;
                if(Base_Flag)object_V_local[name]=marker_scale;                
                continue;
            }
            Support_box.insert(std::map<string,Vector9d>::value_type(name,S_support_object));
            object_xyz.insert(std::map<string,Vector3d>::value_type(name,Vector3d(S_support_object[0],S_support_object[1],S_support_object[8])));
            object_V.insert(std::map<string,Vector3d>::value_type(name,marker_scale));
            Scene_Relation[current_scene].push_back(name);
            if(!Base_Flag)KFG_AR_Active_Finish = true;
            if(Base_Flag)object_V_local.insert(std::map<string,Vector3d>::value_type(name,marker_scale));                   
            Rgb_color.insert(std::map<string,string>::value_type(name,Ar_color));
            
        }
        else if(on_flag == true)
        {
            on_flag = false;

            if(On_box.count(name)>0)
            {
                On_box[name]=S_on_object;
                object_xyz[name]=S_on_object;
                object_V[name]=marker_scale;
                continue;
            }
            On_box.insert(std::map<string,Vector3d>::value_type(name,S_on_object));
            object_xyz.insert(std::map<string,Vector3d>::value_type(name,S_on_object));
            object_V.insert(std::map<string,Vector3d>::value_type(name,marker_scale));
            Scene_Relation[current_scene].push_back(name);            
            if(!Base_Flag)KFG_AR_Active_Finish = true;
            Rgb_color.insert(std::map<string,string>::value_type(name,Ar_color));
            
        }
 
        
    }
   
    //ROS_INFO("%d", t);
    
}


int scannet_count=0;
int frame_count=0;
int local_show_count=0;
double time_sum=0;
int time_count=0;
void Parse_Node::color_Callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    // record time
    double t1 = cv::getTickCount();

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


    // 生成scannet格式数据集
    if(Scannet_Flag)
    {    
        if(frame_count == 5)
        {
            // color
            std::stringstream scan_count;
            scan_count << "/home/ybg/projects/3D-Scene-Graph/data/mo_scannet/color/" << scannet_count << ".jpg";
            cv::imwrite(scan_count.str(),image);
            scan_count.str("");
            // depth
            scan_count << "/home/ybg/projects/3D-Scene-Graph/data/mo_scannet/depth/" << scannet_count << ".png";
            cv::imwrite(scan_count.str(),depth_pic);
            // pose
            //定义机器人本体到相机之间的变换
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
                ROS_WARN("Failed to compute dark pose, skipping scan (%s)", e.what());
                ros::Duration(1.0).sleep();
                return ;
            }
            trans_Bbox << transform_Bbox.getOrigin().x(),transform_Bbox.getOrigin().y(),transform_Bbox.getOrigin().z();

            quat_Bbox.w() = transform_Bbox.getRotation().getW();
            quat_Bbox.x() = transform_Bbox.getRotation().getX();
            quat_Bbox.y() = transform_Bbox.getRotation().getY();
            quat_Bbox.z() = transform_Bbox.getRotation().getZ();

            Eigen::Isometry3d T_base_to_camera = Eigen::Isometry3d::Identity();
            T_base_to_camera = Eigen::Isometry3d::Identity();
            T_base_to_camera.rotate ( quat_Bbox );
            T_base_to_camera.pretranslate ( trans_Bbox );
            
            scan_count.str("");
            scan_count << "/home/ybg/projects/3D-Scene-Graph/data/mo_scannet/pose/" << scannet_count << ".txt";    
            ofstream ofile(scan_count.str());
            
            // ofile<< T_base_to_camera.matrix();
            for(int i=0;i<4;i++)
            {
                for(int j=0;j<4;j++)
                {
                    ofile << T_base_to_camera.matrix()(i,j)<< " ";
                }
                ofile << endl;
            }
            ofile.close();//④
            scannet_count++;
            frame_count=0;
            
        }
        frame_count++;
    
    }

    // 节点注释
    // 画圈、写字
    int add=15;
    int r=0,g=255,b=255;

    //yolo
    for(auto iter = object_2d_pose.begin();iter != object_2d_pose.end(); iter++)
    {
        add = 15;
        r=0,g=255,b=255;
        string pose_name = iter->first;
        Vector2d pose_d = iter->second;

        std::stringstream ssss; 
        float pp=0;
        if(object_P.count(pose_name))
            pp = object_P[pose_name];
        ssss << pose_name << ":" << pp ;

        cv::circle(image,cv::Point(pose_d[0],pose_d[1]),10,cv::Scalar(0, 255, 0),4,8);
        cv::putText(image, ssss.str(), cv::Point(pose_d[0]+15,pose_d[1]), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);

        // 计算颜色
        string pose_color = compute_color(image, pose_d[0], pose_d[1]);
        if(Rgb_color.count(pose_name) == 0)
            Rgb_color.insert(std::map<string,string>::value_type(pose_name,pose_color));
        else if(Rgb_color.count(pose_name) > 0)
            Rgb_color[pose_name]=pose_color;

        cv::putText(image, "color", cv::Point(pose_d[0]+10,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
        cv::putText(image, ":", cv::Point(pose_d[0]+70,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
        cv::putText(image, pose_color, cv::Point(pose_d[0]+80,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);

        add = add+15; 
        r=255*abs(r-255);
        g=255*abs(g-255);
        b=255*abs(b-255);
        ssss.str("");
        ssss << float((int)(object_V_local[pose_name][0]*1000+0.5f)/1000.0) << "," 
            << float((int)(object_V_local[pose_name][1]*1000+0.5f)/1000.0) << "," 
            << float((int)(object_V_local[pose_name][2]*1000+0.5f)/1000.0);
        cv::putText(image, "size", cv::Point(pose_d[0]+10,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
        cv::putText(image, ":", cv::Point(pose_d[0]+70,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
        cv::putText(image, ssss.str(), cv::Point(pose_d[0]+80,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
           
        add = add+15; 
        r=255*abs(r-255);
        g=255*abs(g-255);
        b=255*abs(b-255);
        ssss.str("");
        ssss << float((int)(object_xyz_local[pose_name][0]*1000+0.5f)/1000.0) << "," 
            << float((int)(object_xyz_local[pose_name][1]*1000+0.5f)/1000.0) << "," 
            << float((int)(object_xyz_local[pose_name][2]*1000+0.5f)/1000.0);
        cv::putText(image, "position", cv::Point(pose_d[0]+10,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
        cv::putText(image, ":", cv::Point(pose_d[0]+70,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
        cv::putText(image, ssss.str(), cv::Point(pose_d[0]+80,pose_d[1]+add), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(r, g, b), 0.5, 8, 0);
           

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


    
    // compute local map relationships
    std::vector<std::tuple<string,string,string,float>> Local_rela_after_map;
    current_relationships = current_scene_tree.get_child("relationships");
    Map_Compute::Compute_Local_Relationships_Map(
        current_relationships,
        Support_box,
        On_box_local,
        object_2d_ar_pose,
        object_2d_pose,
        object_V_local,
        Local_rela_after_map);

   
    for(int i=0;i<Local_rela_after_map.size();i++)
    {
        string Ob_name,Sub_name,Relation;
        float Relation_P;
        std::tie(Ob_name,Sub_name,Relation,Relation_P) = Local_rela_after_map[i];
        // cout << Ob_name << " , " << Sub_name << " " << Relation << " is " << Relation_P << endl;

        if(Relation == "ON")
        {
            if(support_relationships_local.count(Ob_name) == 0)
                support_relationships_local.insert(std::map<string,string>::value_type(Ob_name,Sub_name));
            else if(support_relationships_local.count(Ob_name) > 0)
                support_relationships_local[Ob_name]=Sub_name;
            
        }

        if(Relation == "IN")
        {
            if(contian_relationships_local.count(Ob_name) == 0)
                contian_relationships_local.insert(std::map<string,string>::value_type(Ob_name,Sub_name));
            else if(contian_relationships_local.count(Ob_name) > 0)
                contian_relationships_local[Ob_name]=Sub_name;
        }     
                
    }
    


    for(auto iter = support_relationships_local.begin();iter != support_relationships_local.end(); iter++)
    {
        string Ob_name = iter->first;
        string Sub_name = iter->second;
        if(object_2d_ar_pose.count(Sub_name)>0)
        {
            std::stringstream sss;
            // sss << "on:" << Relation_P << endl;
            // 连接两端的箭头线，有的可能存在ar里有的不是，因为实时清空所以实时显示更新
            if(object_2d_ar_pose.count(Ob_name)>0)
            {
                cv::arrowedLine(image, cv::Point(object_2d_ar_pose[Sub_name][0], object_2d_ar_pose[Sub_name][1]),
                cv::Point(object_2d_ar_pose[Ob_name][0], object_2d_ar_pose[Ob_name][1]), cv::Scalar(0, 255, 0), 2, 4,0,0.1);

                // cv::putText(image, sss.str(), cv::Point((object_2d_ar_pose[Sub_name][0]+object_2d_ar_pose[Ob_name][0])/2,
                // (object_2d_ar_pose[Sub_name][1]+object_2d_ar_pose[Ob_name][1])/2+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 8);

            }
            else if(object_2d_pose.count(Ob_name)>0)
            {
                cv::arrowedLine(image, cv::Point(object_2d_ar_pose[Sub_name][0], object_2d_ar_pose[Sub_name][1]),
                cv::Point(object_2d_pose[Ob_name][0], object_2d_pose[Ob_name][1]), cv::Scalar(0, 255, 0), 2, 4,0,0.1);

                // cv::putText(image, sss.str(), cv::Point((object_2d_ar_pose[Sub_name][0]+object_2d_pose[Ob_name][0])/2,
                // (object_2d_ar_pose[Sub_name][1]+object_2d_pose[Ob_name][1])/2+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 8);

            }
        }
        
    }

    for(auto iter = contian_relationships_local.begin();iter != contian_relationships_local.end(); iter++)
    {
        string Ob_name = iter->first;
        string Sub_name = iter->second;
        if(object_2d_ar_pose.count(Sub_name)>0)
        {
            // sss << "in:" << Relation_P << endl;
            // 连接两端的箭头线，有的可能存在ar里有的不是，因为实时清空所以实时显示更新
            if(object_2d_ar_pose.count(Ob_name)>0)
            {
                cv::arrowedLine(image, cv::Point(object_2d_ar_pose[Sub_name][0], object_2d_ar_pose[Sub_name][1]),
                cv::Point(object_2d_ar_pose[Ob_name][0], object_2d_ar_pose[Ob_name][1]), cv::Scalar(0, 0, 255), 2, 4,0,0.1);

                // cv::putText(image, sss.str(), cv::Point((object_2d_ar_pose[Sub_name][0]+object_2d_ar_pose[Ob_name][0])/2,
                // (object_2d_ar_pose[Sub_name][1]+object_2d_ar_pose[Ob_name][1])/2+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2, 8);

            }
            else if(object_2d_pose.count(Ob_name)>0)
            {
                cv::arrowedLine(image, cv::Point(object_2d_ar_pose[Sub_name][0], object_2d_ar_pose[Sub_name][1]),
                cv::Point(object_2d_pose[Ob_name][0], object_2d_pose[Ob_name][1]), cv::Scalar(0, 0, 255), 2, 4,0,0.1);

                
            }
        }

    }


    if(Base_Flag && local_show_count>100)
    {
        local_show_count=0;
        // graphviz 
        // 添加场景
        for(auto iter = Scene_Relation.begin();iter != Scene_Relation.end(); iter++)
        {
            string scene_name = iter->first;
            std::vector<string> simple_scene = iter->second;

            PyObject *pArgs_scene = PyTuple_New(1);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
            PyTuple_SetItem(pArgs_scene, 0, PyString_FromString(scene_name.c_str())); //0--序号,i表示创建int型变量

            PyEval_CallObject(pFunc_scene, pArgs_scene); //调用函数
            

            // 添加节点
            for(auto iter = object_V_local.begin();iter != object_V_local.end(); iter++)
            {
                string object_name = iter->first;
                Vector3d object_xyz_ = object_xyz_local[object_name];
                Vector3d object_V_ = iter->second;

                if(std::find(simple_scene.begin(), simple_scene.end(), object_name) == simple_scene.end()) 
                    continue;

                //创建参数:
                PyObject *pArgs_node = PyTuple_New(9);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
                PyTuple_SetItem(pArgs_node, 0, Py_BuildValue("s", scene_name.c_str())); //0--序号,i表示创建int型变量
                PyTuple_SetItem(pArgs_node, 1, Py_BuildValue("s", object_name.c_str())); //0--序号,i表示创建int型变量
                PyTuple_SetItem(pArgs_node, 2, Py_BuildValue("f", double((int)(object_xyz_[0]*1000+0.5f)/1000.0))); //1--序号
                PyTuple_SetItem(pArgs_node, 3, Py_BuildValue("f", double((int)(object_xyz_[1]*1000+0.5f)/1000.0))); //2--序号
                PyTuple_SetItem(pArgs_node, 4, Py_BuildValue("f", double((int)(object_xyz_[2]*1000+0.5f)/1000.0))); //3--序号
                PyTuple_SetItem(pArgs_node, 5, Py_BuildValue("f", double((int)(object_V_[0]*1000+0.5f)/1000.0))); //1--序号
                PyTuple_SetItem(pArgs_node, 6, Py_BuildValue("f", double((int)(object_V_[1]*1000+0.5f)/1000.0))); //2--序号
                PyTuple_SetItem(pArgs_node, 7, Py_BuildValue("f", double((int)(object_V_[2]*1000+0.5f)/1000.0))); //3--序号
                PyTuple_SetItem(pArgs_node, 8, Py_BuildValue("s", Rgb_color[object_name].c_str())); //4--序号
                // 无返回值
                PyEval_CallObject(pFunc_node, pArgs_node); //调用函数

            }

        }

        // 添加关系
        for(auto iter = support_relationships_local.begin();iter != support_relationships_local.end(); iter++)
        {
            
            std::string name_on = iter->first;
            std::string name_support = iter->second;
            
            // 添加关系
            //创建参数:
            PyObject *pArgs_rela = PyTuple_New(3);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
            PyTuple_SetItem(pArgs_rela, 0, Py_BuildValue("s", name_on.c_str())); //0--序号,i表示创建int型变量
            PyTuple_SetItem(pArgs_rela, 1, Py_BuildValue("s", name_support.c_str())); //1--序号
            PyTuple_SetItem(pArgs_rela, 2, Py_BuildValue("s", "on")); //2--序号
            // 无返回值
            PyEval_CallObject(pFunc_rela, pArgs_rela); //调用函数

        }

        for(auto iter = contian_relationships_local.begin();iter != contian_relationships_local.end(); iter++)
        {
            
            std::string name_in = iter->first;
            std::string name_out = iter->second;
            
            // 添加关系
            //创建参数:
            PyObject *pArgs_rela = PyTuple_New(3);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
            PyTuple_SetItem(pArgs_rela, 0, Py_BuildValue("s", name_in.c_str())); //0--序号,i表示创建int型变量
            PyTuple_SetItem(pArgs_rela, 1, Py_BuildValue("s", name_out.c_str())); //1--序号
            PyTuple_SetItem(pArgs_rela, 2, Py_BuildValue("s", "in")); //2--序号
            // 无返回值
            PyEval_CallObject(pFunc_rela, pArgs_rela); //调用函数

        }

        // update date
        PyEval_CallObject(pFunc_viz, NULL); 

    }
    local_show_count++;

        
        
    std::stringstream sss;
    sss << current_scene;            
    cv::putText(image, sss.str(), cv::Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2, 8);



    pg_image_show = image;



    // 关系注释
    // 50times
    if(KFG_Active_Finish || KFG_AR_Active_Finish)
    {
        // dark part
        if(KFG_Active_Finish)
        {
            KFG_AR_Active_Finish=false;
            KFG_Active_Finish=false;
            // the sum of xyz V 将同一类的xyz和V放到同一容器
            std::map<string,std::vector<std::tuple<Vector3d,Vector3d>>> KFG_sum;
            for(int i=0;i<KFG_optimize.size();i++)
            {
                for(int j=0;j<KFG_optimize[i].size();j++)
                {
                    string KFG_name ;
                    Vector3d KFG_xyz;
                    Vector3d KFG_V;
                    std::tie(KFG_name,KFG_xyz,KFG_V) = KFG_optimize[i][j];

                    if(KFG_sum.count(KFG_name) == 0)
                    {
                        std::vector<std::tuple<Vector3d,Vector3d>> xyz_V ;
                        xyz_V.push_back(make_tuple(KFG_xyz,KFG_V)) ;
                        KFG_sum.insert(make_pair(KFG_name,xyz_V));

                    }
                    else 
                        KFG_sum[KFG_name].push_back(make_tuple(KFG_xyz,KFG_V));

                }
            }

            // 遍历每一种classid
            for(auto iter = KFG_sum.begin();iter != KFG_sum.end(); iter++)
            {
                string KDF_name_oneclass = iter->first;
                std::vector<std::tuple<Vector3d,Vector3d>> KDF_sum_oneclassid = iter->second;
                if(KDF_sum_oneclassid.size()>15)
                {
                    // xyz V 求和取均值
                    Vector3d xyz_sum = Vector3d::Zero();
                    Vector3d V_sum = Vector3d::Zero();
                    for(int i=0;i<KDF_sum_oneclassid.size();i++)
                    {
                        // xyz_sum[0]=xyz_sum[0]+KDF_xyz_sum_oneclass[i][0];
                        // xyz_sum[1]=xyz_sum[1]+KDF_xyz_sum_oneclass[i][1];
                        // xyz_sum[2]=xyz_sum[2]+KDF_xyz_sum_oneclass[i][2];
                        Vector3d xyz_sum_one,V_sum_one;

                        std::tie(xyz_sum_one,V_sum_one) = KDF_sum_oneclassid[i];

                        xyz_sum=xyz_sum+xyz_sum_one;
                        V_sum=V_sum+V_sum_one;
                    }
                    xyz_sum = xyz_sum/KDF_sum_oneclassid.size();
                    V_sum = V_sum/KDF_sum_oneclassid.size();
                    // cout << "V_sum : " << KDF_name_oneclass << " " << V_sum << endl;
                    
                    // update the globle on_box
                    cout << "record______: " << KDF_name_oneclass << endl;
                    if(On_box.count(KDF_name_oneclass) == 0)
                    {
                        On_box.insert(std::map<string,Vector3d>::value_type(KDF_name_oneclass,xyz_sum));
                        object_xyz.insert(std::map<string,Vector3d>::value_type(KDF_name_oneclass,xyz_sum));
                        object_V.insert(std::map<string,Vector3d>::value_type(KDF_name_oneclass,V_sum));
                        Scene_Relation[current_scene].push_back(KDF_name_oneclass);            
                        
                    }
                    else
                    {
                        On_box[KDF_name_oneclass]=xyz_sum;
                        object_xyz[KDF_name_oneclass]=xyz_sum;
                        if(V_sum[0]>0.1 && V_sum[1]>0.1 && V_sum[2]>0.1)
                            object_V[KDF_name_oneclass]=V_sum;
                    }

                    
                }

            }

            // update local date
            On_box_local.clear();
            On_box_local.insert(On_box.begin(),On_box.end());
            object_xyz_local.clear();
            object_xyz_local.insert(object_xyz.begin(),object_xyz.end());
            object_V_local.clear();
            object_V_local.insert(object_V.begin(),object_V.end());

        }

        // ar part
        if(KFG_AR_Active_Finish)
        {
            KFG_AR_Active_Finish=false;
        }
        // cout << "KFG_AR_Active_Finish " << KFG_AR_Active_Finish << endl;
        
        // 判断在哪个房间
        // for(auto iter = Scene_box.begin();iter != Scene_box.end(); iter++)
        // {
        //     string scene_name = iter->first;
        //     Vector9d Sbox_scene_object_ = iter->second;

        //     // scene面积   
        //     Vector2d a(Sbox_scene_object_[0],Sbox_scene_object_[1]);           
        //     Vector2d b(Sbox_scene_object_[2],Sbox_scene_object_[3]);           
        //     Vector2d c(Sbox_scene_object_[4],Sbox_scene_object_[5]);           
        //     Vector2d d(Sbox_scene_object_[6],Sbox_scene_object_[7]); 

        //     std::vector<string> scene_objects;
        //     for(auto iter_ = object_xyz.begin();iter_ != object_xyz.end(); iter_++)
        //     {
        //         string scene_object_name = iter_->first;
        //         Vector3d scene_object_xyz = iter_->second;

        //         if(scene_name != scene_object_name)
        //         {
        //             Vector2d pp(scene_object_xyz[0],scene_object_xyz[1]);     
        //             if((pp-a).transpose()*(b-a) > 0
        //             && (pp-a).transpose()*(c-a) > 0
        //             && (pp-d).transpose()*(b-d) > 0
        //             && (pp-d).transpose()*(c-d) > 0)
        //             {
        //                 scene_objects.push_back(scene_object_name);
        //             }     
                    
        //         }
        //     }
            
        //     if(Scene_Relation.count(current_scene) == 0)
        //         Scene_Relation.insert(make_pair(current_scene,scene_objects));
        //     else 
        //         Scene_Relation[current_scene] = scene_objects;

        // }
    
        // compute map relationships
        std::vector<std::tuple<string,string,string,float>> Globe_rela_after_map;
        current_relationships = current_scene_tree.get_child("relationships");
        Map_Compute::Compute_Globe_Relationships_Map(
            current_relationships,
            Support_box,
            On_box,
            object_V,
            Scene_Relation[current_scene],
            Globe_rela_after_map);
     
        for(int i=0;i<Globe_rela_after_map.size();i++)
        {
            string Ob_name,Sub_name,Relation;
            float Relation_P;
            std::tie(Ob_name,Sub_name,Relation,Relation_P) = Globe_rela_after_map[i];
            // cout << Ob_name << " , " << Sub_name << " " << Relation << " is " << Relation_P << endl;
 
            if(Relation == "ON")
            {
                if(support_relationships.count(Ob_name) == 0)
                    support_relationships.insert(std::map<string,string>::value_type(Ob_name,Sub_name));
                else if(support_relationships.count(Ob_name) > 0)
                    support_relationships[Ob_name]=Sub_name;
                
            }

            if(Relation == "IN")
            {
                if(contian_relationships.count(Ob_name) == 0)
                    contian_relationships.insert(std::map<string,string>::value_type(Ob_name,Sub_name));
                else if(contian_relationships.count(Ob_name) > 0)
                    contian_relationships[Ob_name]=Sub_name;
            }     
                    
        }

        //将全局关系更新到local关系中
        support_relationships_local.clear();
        support_relationships_local.insert(support_relationships.begin(),support_relationships.end());
        contian_relationships_local.clear();
        contian_relationships_local.insert(contian_relationships.begin(),contian_relationships.end());

      

        // graphviz 
        // 添加场景
        for(auto iter = Scene_Relation.begin();iter != Scene_Relation.end(); iter++)
        {
            string scene_name = iter->first;
            std::vector<string> simple_scene = iter->second;

            PyObject *pArgs_scene = PyTuple_New(1);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
            PyTuple_SetItem(pArgs_scene, 0, PyString_FromString(scene_name.c_str())); //0--序号,i表示创建int型变量

            PyEval_CallObject(pFunc_scene, pArgs_scene); //调用函数
            

            // 添加节点
            for(auto iter = object_xyz.begin();iter != object_xyz.end(); iter++)
            {
                string object_name = iter->first;
                Vector3d object_xyz_ = iter->second;
                Vector3d object_V_ = object_V[object_name];

                if(std::find(simple_scene.begin(), simple_scene.end(), object_name) == simple_scene.end()) 
                    continue;

                //创建参数:
                PyObject *pArgs_node = PyTuple_New(9);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
                PyTuple_SetItem(pArgs_node, 0, Py_BuildValue("s", scene_name.c_str())); //0--序号,i表示创建int型变量
                PyTuple_SetItem(pArgs_node, 1, Py_BuildValue("s", object_name.c_str())); //0--序号,i表示创建int型变量
                PyTuple_SetItem(pArgs_node, 2, Py_BuildValue("f", double((int)(object_xyz_[0]*1000+0.5f)/1000.0))); //1--序号
                PyTuple_SetItem(pArgs_node, 3, Py_BuildValue("f", double((int)(object_xyz_[1]*1000+0.5f)/1000.0))); //2--序号
                PyTuple_SetItem(pArgs_node, 4, Py_BuildValue("f", double((int)(object_xyz_[2]*1000+0.5f)/1000.0))); //3--序号
                PyTuple_SetItem(pArgs_node, 5, Py_BuildValue("f", double((int)(object_V_[0]*1000+0.5f)/1000.0))); //1--序号
                PyTuple_SetItem(pArgs_node, 6, Py_BuildValue("f", double((int)(object_V_[1]*1000+0.5f)/1000.0))); //2--序号
                PyTuple_SetItem(pArgs_node, 7, Py_BuildValue("f", double((int)(object_V_[2]*1000+0.5f)/1000.0))); //3--序号
                PyTuple_SetItem(pArgs_node, 8, Py_BuildValue("s", Rgb_color[object_name].c_str())); //4--序号
                // 无返回值
                PyEval_CallObject(pFunc_node, pArgs_node); //调用函数

            }

        }

        // 添加关系
        for(auto iter = support_relationships.begin();iter != support_relationships.end(); iter++)
        {
            
            std::string name_on = iter->first;
            std::string name_support = iter->second;
            
            // 添加关系
            //创建参数:
            PyObject *pArgs_rela = PyTuple_New(3);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
            PyTuple_SetItem(pArgs_rela, 0, Py_BuildValue("s", name_on.c_str())); //0--序号,i表示创建int型变量
            PyTuple_SetItem(pArgs_rela, 1, Py_BuildValue("s", name_support.c_str())); //1--序号
            PyTuple_SetItem(pArgs_rela, 2, Py_BuildValue("s", "on")); //2--序号
            // 无返回值
            PyEval_CallObject(pFunc_rela, pArgs_rela); //调用函数

        }

        for(auto iter = contian_relationships.begin();iter != contian_relationships.end(); iter++)
        {
            
            std::string name_in = iter->first;
            std::string name_out = iter->second;
            
            // 添加关系
            //创建参数:
            PyObject *pArgs_rela = PyTuple_New(3);                 //函数调用的参数传递均是以元组的形式打包的,4表示参数个数
            PyTuple_SetItem(pArgs_rela, 0, Py_BuildValue("s", name_in.c_str())); //0--序号,i表示创建int型变量
            PyTuple_SetItem(pArgs_rela, 1, Py_BuildValue("s", name_out.c_str())); //1--序号
            PyTuple_SetItem(pArgs_rela, 2, Py_BuildValue("s", "in")); //2--序号
            // 无返回值
            PyEval_CallObject(pFunc_rela, pArgs_rela); //调用函数

        }

        // update date
        PyEval_CallObject(pFunc_viz, NULL); 

/*
        int rows = 600;
        int cols = 200+(Support_box.size() + On_box.size())*200;
        int x_ = cols/2;
        int y_ = 50;
        int count = 0;
        std::stringstream ss;
        std::map<string,Vector2d> object_pg_pose;
        if(cols > 0)
        {
            cv::Mat image_pg(rows, cols, CV_8UC3, Scalar(255,255,255));

            string pg_name = knowledgegraph.get<string>("name");
            
            Draw_PG::draw_node(image_pg,cols/2,y_,pg_name);

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
                ss << double((int)(object_xyz_[0]*1000+0.5f)/1000.0) 
                    << "," 
                    << double((int)(object_xyz_[1]*1000+0.5f)/1000.0)
                    << "," 
                    << double((int)(object_xyz_[2]*1000+0.5f)/1000.0);
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
                Draw_PG::draw_Arc(image_pg,cv::Point(start[0],start[1]),cv::Point(meddle[0],meddle[1]),cv::Point(end[0],end[1]),2,cv::Scalar(255, 255, 0));

            }
            for(auto iter = contian_relationships.begin();iter != contian_relationships.end(); iter++)
            {
                
                std::string name_in = iter->first;
                std::string name_out = iter->second;
            
                Vector2d start = object_pg_pose[name_in];
                Vector2d end = object_pg_pose[name_out];
                Vector2d meddle = Vector2d((start[0]+end[0])/2,start[1]+50);
                Draw_PG::draw_Arc(image_pg,cv::Point(start[0],start[1]),cv::Point(meddle[0],meddle[1]),cv::Point(end[0],end[1]),2,cv::Scalar(255, 255, 0));

            }
            // for(auto iter = adjoin_relationships.begin();iter != adjoin_relationships.end(); iter++)
            // {
                
            //     std::string name_beadjoin = iter->first;
            //     std::vector<string> name_adjoin = iter->second;
            //     Vector2d start = object_pg_pose[name_beadjoin];
            
            //     for(int i=0; i<name_adjoin.size(); i++)
            //     {
            //         Vector2d end = object_pg_pose[name_adjoin[i]];
            //         Vector2d meddle = Vector2d((start[0]+end[0])/2,start[1]+50);
            //         Draw_PG::draw_Arc(image_pg,cv::Point(start[0],start[1]),cv::Point(meddle[0],meddle[1]),cv::Point(end[0],end[1]),2);

            //     }
                
            // }

            // 发布pg图
            pg_graph = image_pg;
            sensor_msgs::ImagePtr pg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_pg).toImageMsg();
            pub_pg_show.publish(pg_msg);

        }
*/
    }
    

    // 发布带节点关系的图像
    if(Save_Image_Flag)
    {
        if(frame_count == 5)
        {
            std::stringstream image_count;
            image_count << "/home/ybg/ROS_code/catkin_vision/src/parse_graph/vis_result/image_pg/image_" << output_image_count << ".png";
            cv::imwrite(image_count.str(),image);
            output_image_count++;
            frame_count=0;
        }
        frame_count++;
    }
    
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub_pic.publish(msg);

    // visualization_msgs::MarkerArray markerarray;
    // Parse_Node::PublishMarker(object_V_local);
    
    
    // marker_pub.publish(markerarray);

    double t2 = cv::getTickCount();
    time_count++;
    time_sum += 1000 * (double)(t2 - t1) / cv::getTickFrequency();
	cout << "Time count: " << time_count <<" , time sum: "<< time_sum << " ms." << endl;

}

int dark_frame_count=0;
int dark_output_image_count=0;
void Parse_Node::dark_detect_Callback(const sensor_msgs::ImageConstPtr& image_msg)
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

    cv::Mat dark_image = cv_ptr->image;

    // if(Save_Image_Flag)
    {
        if(dark_frame_count == 5)
        {
            std::stringstream image_count;
            image_count << "/home/ybg/ROS_code/catkin_vision/src/parse_graph/vis_result/dark_image/image_" << dark_output_image_count << ".png";
            cv::imwrite(image_count.str(),dark_image);
            dark_output_image_count++;
            dark_frame_count=0;
        }
        dark_frame_count++;
    }
}



void Parse_Node::PublishMarker(std::map<string,Vector3d> object_V)
{
    // 发布object的scale
    std::stringstream sss;
    int count_marker=0;
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;
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

        if(On_box.count(iter->first))
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
        marker.color.a = 0.7f;//1.0;

        marker.lifetime = ros::Duration();

        // markerarray.markers.push_back(marker);
        marker_pub.publish(marker);
        count_marker++;
    }
}

void Parse_Node::CameraInfo(const sensor_msgs::CameraInfo& camera_info)
{
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info);
      // Get camera intrinsic properties for rectified image.
    fx = camera_model.fx(); // focal length in camera x-direction [px]
    fy = camera_model.fy(); // focal length in camera y-direction [px]
    cx = camera_model.cx(); // optical center x-coordinate [px]
    cy = camera_model.cy(); // optical center y-coordinate [px]

}


void Parse_Node::depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    if(Scannet_Flag)
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); // TYPE_32FC1
    else    
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); // TYPE_32FC1

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
  }

  depth_pic = depth_ptr->image;
}