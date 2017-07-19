#include <iostream>

#include "robot.h"
using namespace std;

int main()
{
    //TcpServer *tserver = new TcpServer(8888);
    Robot r;
    r.run();
    return 0;
}
