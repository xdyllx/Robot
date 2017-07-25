#ifndef ROBOT_H
#define ROBOT_H

#include "tcpserver.h"
#include "Aria.h"
#include "knect.h"



class Robot
{
public:
    Robot();
    Robot(TcpServer *_t);

    void run();

private:
    void init();
    void RobotRotate(double);
    void Move(int);
    void Reset();
    //void PrintInfo();
    void str2int(int &int_temp,const std::string &string_temp);
    void AvoidSide();
    void Align();

private:
    double tmpangle;
    int status; // used to avoid obstacle
    int RotateVelMax;
    TcpServer *t;
    ArKeyHandler keyHandler;
    ArRobotConnector *connector;
    int robot_vel;
    ArRobot robot;
    Knect *k;
    bool rg[3]; //1-red,0-green
    bool rgflag = false; // be true when counter rg traffic
    bool stop;
};

#endif // ROBOT_H
