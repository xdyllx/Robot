#include "knect.h"
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
using namespace std;
using namespace cv;

Knect::Knect()
{
    Init();
}

void Knect::Init()
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
    robot_status = 0;
    isWorking = true;

    FILE *fin = fopen("init.txt","r");
    for (int i=0; i<424; i++)
        for (int j=0; j<512; j++)
            fscanf(fin, "%f", &init[i][j]);

    fclose(fin);
    pthread_create(&thr, NULL, thread_run, this);
    pthread_detach(thr);
}

int Knect::getOnePicture()
{
    libfreenect2::FrameMap frames;
    cout << "before wait"<<endl;
    if(!listener->waitForNewFrame(frames,1000))
    {
        std::cout << "*** CameraFreenect2: Failed to get frames!" << std::endl;
        return -1;
    }
    cout << "after wait" <<endl;
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);



    //cv::imshow("rgb", depthmat);
    float *dep_ = (float*)depthmat.data;		//将深度图信息放到dep数组中

    for (int i=0; i<424; i++)
        for (int j=0; j<512; j++)
            dep[i][j] = dep_[i*512+j];
    //cout << "dep=" <<dep[0][0] <<endl;
    //cv::imwrite("output.jpg", depthmat);

    int black_count = 0;
    for(int i=300;i<424;i++)
    {
        for(int j=0;j<25;j++)
        {
            if(dep[i][j] == 0)
                ++black_count;
        }
    }
    listener->release(frames);
    if(black_count > 25*115)
    {
        cout << "right obstacle" <<endl;
        return 1;
    }
    else
        return 0;
}

void Knect::ObserveObstacle()
{
    libfreenect2::FrameMap frames;
    while(!protonect_shutdown)
    {
        if(!isWorking)
        {
            sleep(1);
            continue;
        }
        listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);


        cv::imwrite("output1.jpg",depthmat);
        cv::imshow("rgb", depthmat);
        float *dep_ = (float*)depthmat.data;		//将深度图信息放到dep数组中

        for (int i=0; i<424; i++)
            for (int j=0; j<512; j++)
                dep[i][j] = dep_[i*512+j];
        FILE *fin = fopen("init.txt","r");
        for (int i=0; i<424; i++)
            for (int j=0; j<512; j++)
                fscanf(fin, "%f", &init[i][j]);

        fclose(fin);

        for (int i=0; i<414; i++)
            for (int j=0; j<512; j++)
            {
                if (dep[i][j] < 1e-8) dep[i][j] = 500.0;
                if (init[i][j] < 1e-8) init[i][j] = 500.0;
            }

        int minj = 512, maxj = 0, mindep = 0;

        for (int i=0; i<414; i++)
            for (int j=128; j<384; j++)
            {
                if (isObst(i, j)
                   && isObst(i-1, j) && isObst(i+1, j) && isObst(i, j-1) && isObst(i, j+1)
                   && isObst(i-1, j-1) && isObst(i+1, j-1) && isObst(i-1, j+1) && isObst(i+1, j+1)
                   && isObst(i-10, j-10) && isObst(i+10, j-10) && isObst(i-10, j+10) && isObst(i+10, j+10))
                {
                    if (j < minj) minj = j;
                    if (j > maxj) maxj = j;
                    mindep++;
                    dep_[i*512+j] = 0;
                }
            }

        if (mindep > 5000)
        {
            robot_status = 1;
//            int  sendF= (int)(float(256 - minj) / 512.0 * 60.0) + 5;
//            if (minj > 256) sendF = 5;
//            strcpy(send_buf, "00obstacle");
//            send_buf[10] = (sendF/10) + '0';
//            send_buf[11] = (sendF%10) + '0';
//            send_buf[12] = '\0';
            printf("sendF =%d\n", sendF);
//            printf("%s\n", send_buf);
            printf("Obstacle, Stop\n");
            //send(sockfd, send_buf, strlen(send_buf), 0);   // 向服务器发送信息
        }
        else
        {
            robot_status = 0;
        }
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener->release(frames);
        sleep(1);
    }
}

bool Knect::isObst(int i, int j)
{
    if (i < 0 || j < 0 || i >= 424 || j >= 512) return 0;
    return init[i][j] - dep[i][j] > 20 && dep[i][j] < 600;
}

void *thread_run(void *args)
{
    Knect *k = static_cast<Knect *>(args);
    k->ObserveObstacle();
}
