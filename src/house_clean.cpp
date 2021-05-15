#include <memory>
#include "ros/ros.h"
#include "./moveManager.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "house_clean");
    ros::NodeHandle nodeHandle;
    MoveManager myMoveMgr(&nodeHandle);
    //First go to center
    myMoveMgr.gotoPosition(5,5);
    myMoveMgr.spiralMtion(0,5);
    #ifdef NEVER
    int x,y;
    while(true)
    {
        ROS_INFO("Enter x and y");
        std::cin>>x;
         std::cin >> y;
        if(x == 0 && y == 0) { break; }
        if(x < 0 || x >11 || y < 0 || y >11 )
        {
            ROS_INFO("Please enter values between 0 and 11");
            continue;
        }
        myMoveMgr.gotoPosition(x,y);
    }
    #endif
    return 0;
}