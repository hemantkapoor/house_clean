#include <math.h>
#include "./moveManager.h"
#include "geometry_msgs/Twist.h"

MoveManager::MoveManager(ros::NodeHandle* nodeHandler):m_nodeHandle(nodeHandler)
{
}

bool MoveManager::moveStraight(double speed, float distance, bool forward /*=true */)
{
    if(m_nodeHandle == nullptr) 
    { 
        ROS_ERROR("Empty Node Handler");
        return false; 
    }
    //First lets subscribe to current position
    ros::Rate loopRate(100);
    m_positionUpdated = false;
    ros::Subscriber sub = m_nodeHandle->subscribe("turtle1/pose", 10, &MoveManager::currentPositionCallback, this); 
    ROS_INFO("Waiting for Current POstion ");
    ros::spinOnce();
    while(m_positionUpdated == false)
    {
        loopRate.sleep();   
        ros::spinOnce();
    }
    ROS_INFO("Got Current POstion ");
    auto distanceRemaining = distance;
    if(forward == false)
    {
        speed = -speed;
    }
    /*
    hemant@hemant-VirtualBox:~$ rostopic info /turtle1/cmd_vel
    Type: geometry_msgs/Twist

    Publishers: None

    Subscribers: 
    * /turtlesim (http://hemant-VirtualBox:37999/)

    */
    ros::Publisher moveAction = m_nodeHandle->advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);
    geometry_msgs::Twist twistComand;
    twistComand.linear.x = speed;
    twistComand.linear.y = 0;
    twistComand.linear.z = 0;
    twistComand.angular.x = 0;
    twistComand.angular.y = 0;
    twistComand.angular.z = 0;
    while(distanceRemaining > 0)
    {
        //Lets check x and Y before requesting a move
        auto currentX = m_x;
        auto currentY = m_y;
        ROS_INFO("Distance Remaining = %f, x = %f, y = %f",distanceRemaining,m_x,m_y);
        
        //Lets request movement
        moveAction.publish(twistComand);
        //we wait for 10 ms and then check distance
        loopRate.sleep();
        ros::spinOnce();
        auto distanceTraveled = abs(sqrt(pow((m_y - currentY),2) + pow((m_x - currentX),2)));
        ROS_INFO("Distance Travelled = %f ",distanceTraveled);
        distanceRemaining -= distanceTraveled;
    }
    //Finally stop moving
    twistComand.linear.x = 0;
    moveAction.publish(twistComand);
    ROS_INFO("Final Distance Remaining = %f, x = %f, y = %f",distanceRemaining,m_x,m_y);
}

 bool MoveManager::rotate(double angleSpeed, float angle, bool clockwise /*=true*/ )
{
    if(m_nodeHandle == nullptr) 
    { 
        ROS_ERROR("Empty Node Handler");
        return false; 
    }

    //First lets subscribe to current position
    ros::Rate loopRate(100);
    m_positionUpdated = false;
    ros::Subscriber sub = m_nodeHandle->subscribe("turtle1/pose", 10, &MoveManager::currentPositionCallback, this); 
    ROS_INFO("Waiting for Current POstion ");
    ros::spinOnce();
    while(m_positionUpdated == false)
    {
        loopRate.sleep();   
        ros::spinOnce();
    }
    ROS_INFO("Got Current POstion ");
    auto angleReamining = degreeToRadian(angle);
    geometry_msgs::Twist twistComand;
    twistComand.linear.x = 0;
    twistComand.linear.y = 0;
    twistComand.linear.z = 0;
    twistComand.angular.x = 0;
    twistComand.angular.y = 0;
    twistComand.angular.z = -degreeToRadian(angleSpeed);    
    if(clockwise == false)
    {
        twistComand.angular.z = degreeToRadian(angleSpeed);
    }

    ros::Publisher moveAction = m_nodeHandle->advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);

    while(angleReamining > 0)
    {
        //Lets check x and Y before requesting a move
        auto currentTheta = m_theta;
        ROS_INFO("Angle Remaining = %f",radiantToDegree(angleReamining));
        
        //Lets request movement
        moveAction.publish(twistComand);
        //we wait for 10 ms and then check distance
        ros::spinOnce();
        loopRate.sleep();
        ros::spinOnce();
        auto angleTraveled = abs(m_theta - currentTheta);
        ROS_INFO("Angle Turned = %f ",radiantToDegree(angleTraveled));
        angleReamining -= angleTraveled;
    }
    //Finally stop moving
    twistComand.angular.z = 0;
    moveAction.publish(twistComand);
    ROS_INFO("Final Angle Remaining = %f and current angle = %f",radiantToDegree(angleReamining),radiantToDegree(m_theta));
}

bool MoveManager::gotoPosition(float x, float y)
{
    if(m_nodeHandle == nullptr) 
    { 
        ROS_ERROR("Empty Node Handler");
        return false; 
    }
    ROS_INFO("Target = %f,%f",x,y);
    //First lets subscribe to current position
    ros::Rate loopRate(100);
    m_positionUpdated = false;
    ros::Subscriber sub = m_nodeHandle->subscribe("turtle1/pose", 10, &MoveManager::currentPositionCallback, this); 
    ROS_INFO("Waiting for Current POstion ");
    ros::spinOnce();
    while(m_positionUpdated == false)
    {
        loopRate.sleep();   
        ros::spinOnce();
    }
    ROS_INFO("Got Current POstion ");
    geometry_msgs::Twist twistComand;
    twistComand.linear.x = 0;
    twistComand.linear.y = 0;
    twistComand.linear.z = 0;
    twistComand.angular.x = 0;
    twistComand.angular.y = 0;
    twistComand.angular.z = 0;    

    ros::Publisher moveAction = m_nodeHandle->advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);
    auto k_linear = 0.5;
    auto k_angular = 4;

    while(true)
    {
        //Lets check linesr distance remaining first
        auto linearDistance = abs(sqrt(pow((y - m_y),2) + pow((x - m_x),2)));
        twistComand.linear.x = linearDistance * k_linear;
        //Now angle
        auto angularDistance = atan2((y - m_y),(x - m_x));
        twistComand.angular.z = (angularDistance - m_theta) * k_angular;
        //Lets request movement
        moveAction.publish(twistComand);
        //we wait for 10 ms and then check distance
        ros::spinOnce();
        loopRate.sleep();
        ros::spinOnce();
        if(linearDistance < 0.1)
        {
            break;
        }
    }
    //Finally stop moving
    twistComand.linear.x = 0;
    twistComand.angular.z = 0;
    moveAction.publish(twistComand);
    ROS_INFO("Final position x= %f, y = %f", m_x, m_y);
}

bool MoveManager::spiralMtion(float linearSpeed, float angularSpeed)
{
    if(m_nodeHandle == nullptr) 
    { 
        ROS_ERROR("Empty Node Handler");
        return false; 
    }
    //First lets subscribe to current position
    ros::Rate loopRate(100);
    m_positionUpdated = false;
    ros::Subscriber sub = m_nodeHandle->subscribe("turtle1/pose", 10, &MoveManager::currentPositionCallback, this); 
    ros::spinOnce();
    while(m_positionUpdated == false)
    {
        loopRate.sleep();   
        ros::spinOnce();
    }
    geometry_msgs::Twist twistComand;
    twistComand.linear.x = linearSpeed;
    twistComand.linear.y = 0;
    twistComand.linear.z = 0;
    twistComand.angular.x = 0;
    twistComand.angular.y = 0;
    twistComand.angular.z = angularSpeed;    

    ros::Publisher moveAction = m_nodeHandle->advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);
    //while(m_x < 10.5 && m_y < 10.5)
    while(m_x >= 0.1 && m_x < 10.8 && m_y >= 0.1 && m_y < 10.8)
    {
        moveAction.publish(twistComand);
        ros::spinOnce();
        loopRate.sleep();
        ros::spinOnce();
        twistComand.linear.x += 0.01;
    }
    //Finally stop moving
    twistComand.linear.x = 0;
    twistComand.angular.z = 0;
    moveAction.publish(twistComand);
    ROS_INFO("Final position x= %f, y = %f", m_x, m_y);
}

//Callbacks here
void MoveManager::currentPositionCallback(const turtlesim::Pose::ConstPtr& msg)
{
    m_positionUpdated = true;
    m_x = msg->x;
    m_y = msg->y;
    m_theta = msg->theta;
}

