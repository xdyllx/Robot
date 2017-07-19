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
    bool isObst(int i, int j);
public:
    int robot_status; //1-have obstacle 0-no obstacle
    bool isWorking;

};

void* thread_run(void*);

#endif // KNECT_H
