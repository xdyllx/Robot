#include "robot.h"
#include <time.h>
#include <stdlib.h>
#include <sstream>

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
    robot_vel = 65;
    RotateVelMax = 20;
    status = 0;
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

    //k = new Knect();

}

void Robot::run()
{
    cout <<"run "<<endl;
#define inst t->ins
#define rg t->rg
#define rgflag t->rgflag
    while(1)
    {
        if(t->flag == 1){
            cout << "in flag" <<endl;
            //double rotate_temp = 90 + getRand();
            t->flag = 0;
            //cout <<"rotate_tmp = "<<rotate_temp << endl;
            cout<<"rg:" << rg[0] << rg[1] << rg[2] <<endl;
            if(inst == "forward"){
                if(rg[1])
                    continue;
                if(rgflag)
                {
                    rgflag = false;
                    memset(rg,0,sizeof(rg));
                }
                cout << "receive forward" << endl;
                robot.setVel(robot_vel);
            }
            else if(inst == "back")
            {
                cout << "receive back" << endl;
                robot.setVel(-200);
            }
            else if(inst == "stop"){
                cout << "receive stop" << endl;
                robot.stop();
                continue;
            }
            else if(inst == "left"){
                if(rg[0])
                    continue;
                if(rgflag)
                    Reset();
                cout << "receive left" << endl;
                RobotRotate(90);
                t->sendMessage("haveturn");
            }
            else if(inst == "right"){
                if(rg[2])
                    continue;
                if(rgflag)
                    Reset();
                cout << "receive right" << endl;
                RobotRotate(-90);
                t->sendMessage("haveturn");
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
                status = 1;
                AvoidSide();
            }
            else{
                cout << "error, inst= " << inst <<" rg_ins="<<t->rg_message<< endl;
            }
            inst = "";
        }
        sleep(0.5);

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
}

void Robot::Reset()
{
    memset(rg,0,sizeof(rg));
    rgflag = false;
    t->flag = 0;
    status = 0;
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
    bool tmp = 1;
    int smalldistance = 100;
    double tmpangle = 40;
    int pictureres;
    while(tmp && inst != "stop")
    {
        switch(status)
        {
        case 0:
            tmp = 0;
            break;
        case 1:
            RobotRotate(tmpangle);
            Move(900);
            RobotRotate(-tmpangle);
            status = 2;
            break;
        case 2:
            Move(smalldistance);
            RobotRotate(-tmpangle);
            sleep(0.5);
            //bool tmp = k->getOnePicture();
            pictureres = k->getOnePicture();
            if(pictureres == 0)
                status = 3;
            else if(pictureres == -1)
                return;
            RobotRotate(tmpangle);
            break;
        case 3:
            Move(600);
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
            Move(900);
            RobotRotate(tmpangle);
            status = 0;
            break;
        default:
            cout << "error status:"<<status <<endl;
        }

    }
}
