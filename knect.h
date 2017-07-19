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
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
    libfreenect2::PacketPipeline  *pipeline = NULL;
    std::string serial;

    libfreenect2::SyncMultiFrameListener *listener;/*(
            libfreenect2::Frame::Color |
            libfreenect2::Frame::Depth |
            libfreenect2::Frame::Ir);*/

public:
    Knect();
    void init();
    int getOnePicture();
    void ObserveObstacle();
private:
//    void sigint_handler(int s)
//    {
//        protonect_shutdown = true;
//    }

};

void* thread_run(void*);

#endif // KNECT_H
