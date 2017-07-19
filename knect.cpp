#include "knect.h"

using namespace std;
using namespace cv;

Knect::Knect()
{
    init();
}

void Knect::init()
{
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        //return -1;
    }

    serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;

    int depthProcessor = Processor_cl;

    if(depthProcessor == Processor_cpu)
    {
        if(!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
        //! [pipeline]
    }
    else if (depthProcessor == Processor_gl) // if support gl
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
        {
            pipeline = new libfreenect2::OpenGLPacketPipeline();
        }
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if (depthProcessor == Processor_cl) // if support cl
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        //return -1;
    }

    //signal(SIGINT, sigint_handler);
    protonect_shutdown = false;
    listener = new libfreenect2::SyncMultiFrameListener(
            libfreenect2::Frame::Color |
            libfreenect2::Frame::Depth |
            libfreenect2::Frame::Ir);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    dev->start();
}

int Knect::getOnePicture()
{
    libfreenect2::FrameMap frames;
    if(!listener->waitForNewFrame(frames,1000))
    {
        std::cout << "*** CameraFreenect2: Failed to get frames!" << std::endl;
        return -1;
    }
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);



    //cv::imshow("rgb", depthmat);
    float *dep_ = (float*)depthmat.data;		//将深度图信息放到dep数组中

    for (int i=0; i<424; i++)
        for (int j=0; j<512; j++)
            dep[i][j] = dep_[i*512+j];
    cout << "dep=" <<dep[0][0] <<endl;
    cv::imwrite("output.jpg", depthmat);

    int black_count = 0;
    for(int i=280;i<424;i++)
    {
        for(int j=0;j<40;j++)
        {
            if(dep[i][j] == 0)
                ++black_count;
        }
    }
    listener->release(frames);
    if(black_count > 40*140)
    {
        cout << "right obstacle" <<endl;
        return 1;
    }
    else
        return 0;
}

void Knect::ObserveObstacle()
{

}
