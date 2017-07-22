#include "robot.h"
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
using namespace std;

double getRand() //get random number in [-1,1]
{
    return (double)(rand()%20)/10 - 1;
}

Robot::Robot()
{
    t = new TcpServer(8888);
    init();
}

Robot::Robot(TcpServer *_t)
{
    t = _t;

    init();
}

void Robot::init()
{
    //cout << "into init" <<endl;
    memset(rg, 0, sizeof(rg));
    robot_vel = 65;
    RotateVelMax = 20;
    status = 0;
    stop = false;
    Aria::init(); //初始化

    int argc = 3;
    char* argv[3];
    argv[0] = "";
    argv[1] = "-rp";
    argv[2] = "/dev/ttyUSB0";
    ArArgumentParser parser(&argc,argv);
    parser.loadDefaultArguments();
    cout << "in init" <<endl;
    connector = new ArRobotConnector(&parser,&robot);
    if (!connector->connectRobot())
    {
        if (!parser.checkHelpAndWarnUnparsed())
        {
            ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
        }
        else
        {
            ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if(!robot.isConnected())
    {
        ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
    }

    if(!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }

    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);
    printf("You may press escape to exit\n");

    robot.runAsync(true); //启动机器人线程
    //ArUtil::sleep(1000);
    sleep(1);
    robot.lock();
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.comInt(ArCommands::ENABLE,1);
    robot.unlock();
    sleep(1);
    //robot.setAbsoluteMaxRotVel(15);
    robot.setRotVelMax(RotateVelMax);

    k = new Knect();

}

void Robot::run()
{
    cout <<"run "<<endl;
#define inst t->ins
//#define rg t->rg
//#define rgflag t->rgflag
    while(1)
    {
        k->getOneSence();
        if(k->rg_message.substr(0,4) == "turn")
        {
            rgflag = true;
            if(k->rg_message[5] == 'r')
            {
                stop = true;
            }
            for(int i=0;i<3;i++)
            {
                if(k->rg_message[i+4] == 'r')
                    rg[i] = true;
                else if(k->rg_message[i+4] == 'g')
                    rg[i] = false;
                else
                {
                    std::cout << "shit error,reveive "<<k->rg_message << std::endl;
                }
            }
            k->rg_message = "";
        }

        if(t->flag == 1)
        {
            //cout << "in flag" <<endl;
            //double rotate_temp = 90 + getRand();

            //cout <<"rotate_tmp = "<<rotate_temp << endl;
            cout<<"rg:" << rg[0] << rg[1] << rg[2] <<endl;
            if(inst == "forward"){
                if(rg[1])
                    continue;
                if(rgflag)
                    Reset();
                cout << "receive forward" << endl;
                robot.setVel(robot_vel);
                Reset();
            }
            else if(inst == "back")
            {
                cout << "receive back" << endl;
                robot.setVel(-200);
            }
            else if(inst == "stop" ){
                cout << "receive stop" << endl;
                stop = true;
            }
            else if(inst == "left"){
                if(rg[0])
                    continue;
                if(rgflag)
                    Reset();
                cout << "receive left" << endl;
                RobotRotate(90);
                Reset();
                //t->sendMessage("haveturn");
            }
            else if(inst == "right"){
                if(rg[2])
                    continue;
                if(rgflag)
                    Reset();
                cout << "receive right" << endl;
                RobotRotate(-90);
                Reset();
                //t->sendMessage("haveturn");
            }
            else if(inst == "exit"){
                cout << "receive exit" << endl;
                break;
            }
            else if(inst == "reset"){
                Reset();
                cout << "receive reset" << endl;
            }

            else if(inst.substr(0,3) == "set")
            {
                if(inst.substr(3,3) == "vel")
                {
                    int vel;
                    str2int(vel, inst.substr(6, inst.length()-6));
                    cout << "set vel = "<<vel <<endl;
                    robot_vel = vel;
                }
                else if(inst.substr(3,5) == "angle")
                {
                    int angle;
                    str2int(angle, inst.substr(8, inst.length()-8));
                    cout << "set angle = "<< angle <<endl;
                    RobotRotate(angle);

                }
            }
            else if(inst == "obstacle")
            {
//                status = 1;
//                AvoidSide();
                k->isWorking = 1 - k->isWorking;

                cout << "k->isWorking = " << k->isWorking <<endl;
            }
            else{
                cout << "error, inst= " << inst <<" rg_ins="<<k->rg_message<< endl;
            }
            inst = "";
            t->flag = 0;
        }
        if(stop)
        {
            robot.stop();
            continue;
        }
        if(k->robot_status == 1)
        {
            k->isWorking = false;
            status = 1;
            tmpangle = k->angle;
            AvoidSide();
        }
        sleep(0.8);
    }
}

void Robot::RobotRotate(double angle)
{
    robot.setVel(0);
    cout << "angle = "<< angle<<endl;
    if(angle > 25)
        angle += getRand();
    robot.setDeltaHeading(angle);
    int tmpcount = 0;
    while(!robot.isHeadingDone() && tmpcount <10 && t->ins != "stop")
    {
        sleep(1);
        tmpcount++;
        cout << "in rotate" << endl;

    }
}

void Robot::Move(int distance)
{
    robot.move(distance);
    while(!robot.isMoveDone() && t->ins != "stop")
    {
        sleep(1);
        cout << "is moving" <<endl;
    }
//    int vel = 120;
//    robot.setVel(vel);
//    int seconds = distance / vel;
//    int tmp = 0;
//    while(tmp < seconds)
//    {
//        sleep(1);
//        ++tmp;
//    }
//    robot.setVel(0);
}

void Robot::Reset()
{
    memset(rg,0,sizeof(rg));
    rgflag = false;
    t->flag = 0;
    status = 0;
    k->robot_status = 0;
    stop = false;
}

void Robot::str2int(int &int_temp, const string &string_temp)
{
    stringstream stream(string_temp);
    stream>>int_temp;
}

void Robot::AvoidSide()
{
    // status machine
    //0: no obstacle
    //1: left 30.move 1000, right30
    //2: move 1000,right 30 observe if have status = 2 if not status = 3  left 30
    //3:move 1000 right 30 observe if have left 30 status = 2 if not status = 4
    //4:move 1000 left 30
    bool working = 1;
    int smalldistance = 220;
    int movedis = 540 / cos(tmpangle/180*3.1415926);
    //double tmpangle = 37;
    //double smallangle = 20;
    int pictureres;
    while(working && inst != "stop")
    {
        switch(status)
        {
        case 0:
            working = 0;
            Reset();
            break;
        case 1:
            RobotRotate(tmpangle);
            Move(movedis);
            RobotRotate(-tmpangle);
            status = 2;
            break;
        case 2:
            Move(smalldistance);
            //RobotRotate(-smallangle);
            //sleep(0.5);
            //bool tmp = k->getOnePicture();
            pictureres = k->getOnePicture();
            cout << "picture res=" <<pictureres <<endl;
            if(pictureres == 0)
                status = 3;
            else if(pictureres == -1)
                return;
            //RobotRotate(smallangle);
            break;
        case 3:
            Move(700);
            RobotRotate(-tmpangle);
            sleep(0.5);
            //bool tmp1 = k->getOnePicture();
            pictureres = k->getOnePicture();
            if(pictureres == 1)
            {
                RobotRotate(tmpangle);
                cout << "still have obstacle??"<<endl;
                status = 2;
            }
            else if(pictureres == 0)
                status = 4;
            else
                return;
            break;
        case 4:
            Move(movedis);
            RobotRotate(tmpangle);
            status = 0;
            break;
        default:
            cout << "error status:"<<status <<endl;
        }

    }
}
