#include "ros/ros.h"
#include "node.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "parse");
    ros::NodeHandle nh;
    ros::NodeHandle np("~");

    PARSE parse_debug(nh,np);

    ros::spin();
    cout << "finish!" << endl;
   
    return 0;
}