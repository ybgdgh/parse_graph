#include "ros/ros.h"
#include "node.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "parse_node");
    ros::NodeHandle nh;
    ros::NodeHandle np("~");

    Parse_Node parse_debug(nh,np);

    ros::spin();
    cout << "finish!" << endl;
   
    return 0;
}