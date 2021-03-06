﻿#ifndef LINEFINDER_H
#define LINEFINDER_H


#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <list>
using namespace std;
using namespace cv;


class LineFinder{

public:
    //默认的累加器的分辨率为单个像素即1
    LineFinder() :deltaRho(1), deltaTheta(M_PI / 180), minVote(10), minLength(0.), maxGap(0.){}   


    //使用概率霍夫变换获得直线起始端和结束端对
    vector<Vec4i> findLines(Mat &binary);

    //筛选直线
    void lineRestrict();

    //绘制检测到的直线
    void drawDetectedLines(Mat &image,Scalar color = Scalar(255,0,0));

    //判断机器人左歪（返回1）或右歪（返回2）或条件不满足（返回0）
    int judgeLine();

    //设置累加器的分辨率
    void setAccResolution(double dRho, double dTheta);

    //设置最小投票数
    void setMinVote(int minv);

    //设置缺口及最小长度
    void setLineLengthAndGap(double length, double gap);

private:
    //Mat img;    //原图
    vector<Vec4i>lines;   //向量中检测到的直线的端点
    vector<Vec4i>reslines;
    vector<double>resslope;

    int minpos;
    bool find = false;
    //累加器的分辨率
    double deltaRho;
    double deltaTheta;
    int minVote;    //直线被接受时所需的最小投票数
    double minLength;   //直线的最小长度
    double maxGap;  //沿着直线方向的最大缺口

public:
    double slopelist[10];
    int listnum = 0;
    //list<double> slopelist;
    static const int listsize = 3;
    static const int Min = 980;
};

#endif // LINEFINDER_H
