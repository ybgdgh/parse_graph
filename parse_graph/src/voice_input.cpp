#include "ros/ros.h"
#include "voice_input.h"


// receive a sequence of  and send it as a service 
// service form :object, subject, relation


nav_msgs::OccupancyGrid cost_map_;
bool costmap_received_;

int occupancy_threshold_=50;



// update the costmap with received information coming from callback (topic subscription)
void costMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    cost_map_ = *msg;
    costmap_received_ = true;
    cout << "costmap reserve !" << endl;
}

bool Remedial_Nav(Vector3d &nav)
{
    if(!costmap_received_) {
        ROS_ERROR("Costmap not received, ensure that costmap topic has data (default topic: /move_base/global_costmap/costmap)");
        return false;
    }

    //判断导航点是否在costmap中
    geometry_msgs::Point nav_point;
    nav_point.x = nav[0];
    nav_point.y = nav[1];
    occupancy_grid_utils::Cell nav_cell = occupancy_grid_utils::pointCell(cost_map_.info, nav_point);
    if (cost_map_.data[nav_cell.x + nav_cell.y * cost_map_.info.width] > occupancy_threshold_) // 在costmap中
    {
        // 获取机器人坐标  
        Eigen::Vector3d robot;
        tf::TransformListener listener;
        
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform( "map", "base_link", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("map", "base_link",ros::Time(0), transform);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute pose, skipping scan (%s)", e.what());
            ros::Duration(1.0).sleep();
            return false;
        }
        robot << transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();
        geometry_msgs::Point robot_point;
        robot_point.x = robot[0];
        robot_point.y = robot[1];

        // Check if the turtlebot is going to collide with any known obstacle.
        occupancy_grid_utils::RayTraceIterRange ray_range = occupancy_grid_utils::rayTrace(cost_map_.info, nav_point, robot_point);

        for (occupancy_grid_utils::RayTraceIterator i = ray_range.first; i != ray_range.second; ++i)
        {
            const occupancy_grid_utils::Cell& cell = *i;

            // Check if this cell is occupied.
            if (cost_map_.data[cell.x + cell.y * cost_map_.info.width] < occupancy_threshold_)
            {
                geometry_msgs::Point real_point = occupancy_grid_utils::cellCenter(cost_map_.info, cell);

                nav[0] = real_point.x;
                nav[1] = real_point.y;

                return true;
            }
        }

    }
    return true;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "voice_input");
    ros::NodeHandle nh;
    ros::NodeHandle np("~");

    costmap_received_ = false;

    ros::Subscriber costmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, costMapCallback);

    ros::Publisher pub_nav = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);

    boost::property_tree::ptree PG = INPUT::loadPoseFile("/home/dm/kg.json");
    boost::property_tree::ptree PG_object = PG.get_child("object");
  
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        string object,subject,relation;

        
        if(cin >> object >> relation >> subject)
        {
            // cin >> object >> relation >> subject;
            // ROS_INFO("I heard: %s, %s, %s", object,relation,subject);
            cout << "I heard : " << object << " " << relation << " " << subject << endl;


            Vector3d nav;
            BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, PG_object)
            {
                boost::property_tree::ptree vt = vtt.second;
                string ob = vt.get<string>("name");
                string::size_type idx;
                idx = ob.find(object);
                if(idx != string::npos) // have
                {
                    BOOST_FOREACH (boost::property_tree::ptree::value_type &v, vt)
                    {
                        if(relation == v.first)
                        {
                            string sub = vt.get<string>(relation);
                            string::size_type idxx;
                            idxx = sub.find(subject);
                            if(idxx != string::npos)
                            {
                                boost::property_tree::ptree xyz = vt.get_child("XYZ");                     
                                boost::property_tree::ptree::iterator xyz_ = xyz.begin();
                                int i=0;
                                for(; xyz_ != xyz.end(); ++xyz_)
                                {
                                    nav[i] = xyz_->second.get_value<float>();
                                    i++;
                                }
                            
                                cout << "succeed  xyz :" << nav[0] << " , " << nav[1] << " , " << nav[2] << endl;
                                cout << "Its : " << ob << " " << relation << " " << sub << endl;


                                if(Remedial_Nav(nav))
                                {
                                    move_base_msgs::MoveBaseActionGoal scene_nav;

                                    scene_nav.header.stamp = ros::Time::now();
                                    scene_nav.goal.target_pose.header.stamp=ros::Time::now();
                                    scene_nav.goal.target_pose.header.frame_id = "map";

                                    scene_nav.goal.target_pose.pose.position.x = nav[0];
                                    scene_nav.goal.target_pose.pose.position.y = nav[1];
                                    scene_nav.goal.target_pose.pose.position.z = 0;

                                    scene_nav.goal.target_pose.pose.orientation.x = 0;
                                    scene_nav.goal.target_pose.pose.orientation.y = 0;
                                    scene_nav.goal.target_pose.pose.orientation.z = 0;
                                    scene_nav.goal.target_pose.pose.orientation.w = 1;

                                    pub_nav.publish(scene_nav);
                                }

                                break;
                            }
                        }
                            
        
                    }
                }
            }
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
    

    // cout << "finish!" << endl;
   
    return 0;
}



void PublishNav(Vector3d nav_)
{
    move_base_msgs::MoveBaseActionGoal scene_nav;

    scene_nav.header.stamp = ros::Time::now();
    scene_nav.goal.target_pose.header.stamp=ros::Time::now();
    scene_nav.goal.target_pose.header.frame_id = "map";

    scene_nav.goal.target_pose.pose.position.x = nav_[0];
    scene_nav.goal.target_pose.pose.position.y = nav_[1];
    scene_nav.goal.target_pose.pose.position.z = 0;

    scene_nav.goal.target_pose.pose.orientation.x = 0;
    scene_nav.goal.target_pose.pose.orientation.y = 0;
    scene_nav.goal.target_pose.pose.orientation.z = 0;
    scene_nav.goal.target_pose.pose.orientation.w = 1;

    // pub_nav.publish(scene_nav);

}