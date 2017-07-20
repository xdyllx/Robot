#ifndef KNECT_H
#define KNECT_H

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <pthread.h>


class Knect
{
private:
    enum
    {
        Processor_cl,
        Processor_gl,
        Processor_cpu
    };



    cv::Mat rgbmat, depthmat, irmat, graymat, hsv;



    std::vector<cv::Vec3f> circles;
    int flag[256];
    bool protonect_shutdown = false;
    float dep[424][512];
    float init[424][512];

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
    libfreenect2::PacketPipeline  *pipeline = NULL;
    std::string serial;

    libfreenect2::SyncMultiFrameListener *listener;/*(
            libfreenect2::Frame::Color |
            libfreenect2::Frame::Depth |
            libfreenect2::Frame::Ir);*/

    pthread_t thr;

public:
    Knect();
    void Init();
    int getOnePicture();
    void ObserveObstacle();
    //static int cmp(type x, type y);
private:
    double dist(double x1, double y1, double x2, double y2)		//计算圆心间距离判断是否重叠
    {
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }

    void ChangeColor(cv::Mat *p,int x,int y,int r,int g,int b);		//改变图中的颜色信息

    CvScalar Get(cv::Mat *p, int x, int y);		//获得某个点的颜色信息

    size_t Floodfill(size_t x);		//使用floodfill算法对圆重叠进行判断

    CvScalar Color(double xx, double yy, double rr);   //计算圆内有效颜色的中位数得到颜色

    double isCircle(double xx, double yy, double rr); //计算圆边界有效颜色占比，用来判断是否是圆

    double Per(double xx, double yy, double rr);   //计算圆中有效颜色点占比

    double Dist(float *dep, double xx, double yy, double rr);   //计算圆与kinect距离的中位数得到距离信息

    bool isObst(int i, int j);


public:
    int robot_status; //1-have obstacle 0-no obstacle
    bool isWorking;
    std::string rg_message;
    bool rgWorking;
    struct type
    {
        int RorG;
        double Dis;
        double x, y, r;
    };
};



void* thread_run(void*);

#endif // KNECT_H
