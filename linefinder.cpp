#include "linefinder.h"
#include <iostream>

void LineFinder::setAccResolution(double dRho, double dTheta){

    deltaRho = dRho;
    deltaTheta = dTheta;
}

//设置最小投票数
void LineFinder::setMinVote(int minv){

    minVote = minv;
}

//设置缺口及最小长度
void LineFinder::setLineLengthAndGap(double length, double gap){

    minLength = length;
    maxGap = gap;
}

//使用概率霍夫变换
vector<Vec4i> LineFinder::findLines(Mat &binary)
{
    lines.clear();
    HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
    lineRestrict();
    if(reslines.size() == 0)
    {
        cout << "No line detected" <<endl;
        find = false;
        return reslines;
    }
    find = true;
    minpos = 0;
    for(int i=0;i<reslines.size();i++)
    {
        double tmp = abs((double(reslines[i][0]-reslines[i][2]))/(reslines[i][1]-reslines[i][3]));
        resslope.push_back(tmp);
        if(resslope[i] < resslope[minpos])
            minpos = i;
    }

    printf("minpos = %d, min slope = %f\n", minpos, resslope[minpos]);
    return reslines;
}

void LineFinder::lineRestrict()
{
    reslines.clear();
    vector<Vec4i>::const_iterator it2 = lines.begin();
    while (it2!=lines.end())
    {
        if(((*it2)[3]>Min || (*it2)[1] > Min) && (abs((*it2)[3] - (*it2)[1]) > 100) && (*it2)[0] > 500 && (*it2)[0] < 1100)
        {
            bool flag = true;
            for(vector<Vec4i>::iterator it = reslines.begin();it!=reslines.end();it++)
            {
                if(abs((*it2)[0]-(*it)[0]) < 10)
                {
                    flag = false;
                    break;
                }
            }
            if(flag)
                reslines.push_back((*it2));

        }
        ++it2;
    }
}

//绘制检测到的直线
void LineFinder::drawDetectedLines(Mat &image,Scalar color)
{

    //画线
    if(find)
    {
        Point pt1(reslines[minpos][0], reslines[minpos][1]);
        Point pt2(reslines[minpos][2], reslines[minpos][3]);
        line(image, pt1, pt2, color);
    }

//    vector<Vec4i>::const_iterator it2 = reslines.begin();
//    while (it2!=reslines.end())
//    {
//        Point pt1((*it2)[0],(*it2)[1]);
//        Point pt2((*it2)[2], (*it2)[3]);
//        line(image, pt1, pt2, color);

//        ++it2;
//    }
}
