#pragma once

#include <stdint.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"

namespace MoveManagerName
{

};

class MoveManager
{
public:
    MoveManager() = delete;
    ~MoveManager() = default;
    MoveManager(ros::NodeHandle* nodeHandler);
    bool moveStraight(double speed, float distance, bool forward=true);
    bool rotate(double angleSpeed, float angle, bool clockwise=true);
    bool spiralMtion(float linearSpeed, float angularSpeed);
    bool gotoPosition(float x, float y);
private:
    ros::NodeHandle* m_nodeHandle = nullptr;
    //turtlesim/Pose
    float degreeToRadian(float degree) { return (degree * M_PI) / 180; }
    float radiantToDegree(float radian) { return (radian * 180) / M_PI; }
    void currentPositionCallback(const turtlesim::Pose::ConstPtr& msg);

    float m_x;
    float m_y;
    float m_theta;
    volatile bool m_positionUpdated = false;

};