#ifndef ROBOT_H
#define ROBOT_H

#include "tcpserver.h"
#include "Aria.h"
#include "kinect.h"



class Robot
{
public:
    Robot();
    Robot(TcpServer *_t);

    void run();

private:
    void init();

    //机器人旋转给定角度
    void RobotRotate(double);

    //机器人前进给定距离
    void Move(int);

    //避障
    void AvoidSide();

    //对齐
    void Align();

    //重置
    void Reset(); 

    void str2int(int &int_temp,const std::string &string_temp);

private:
    double tmpangle;
    int status; // used to avoid obstacle
    int RotateVelMax;
    TcpServer *t;
    ArKeyHandler keyHandler;
    ArRobotConnector *connector;
    int robot_vel;
    ArRobot robot;
    Kinect *k;
    bool rg[3]; //1-red,0-green
    bool rgflag = false; // be true when counter traffic light
    bool stop;
};

#endif // ROBOT_H
