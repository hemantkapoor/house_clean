#include <memory>
#include "ros/ros.h"
#include "./moveManager.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "house_clean");
    ros::NodeHandle nodeHandle;
    ros::Rate loopRate(100);
    MoveManager myMoveMgr(&nodeHandle);
    int x,y;
    while(true)
    {
        ROS_INFO("Enter x and y");
        std::cin>>x;
         std::cin >> y;
        if(x == 0 || y == 0) { break; }
        myMoveMgr.gotoPosition(x,y);
    }
    return 0;
}