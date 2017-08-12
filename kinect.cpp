#include "kinect.h"
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

using namespace std;
using namespace cv;

Kinect::Kinect()
{
    Init();
}

void Kinect::Init()
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
    isWorking = false;
    FILE *fin = fopen("init.txt","r");
    for (int i=0; i<424; i++)
        for (int j=0; j<512; j++)
            fscanf(fin, "%f", &init[i][j]);

    fclose(fin);
    for (int i=0; i<414; i++)
        for (int j=0; j<512; j++)
        {
            if (init[i][j] < 1e-8) init[i][j] = 500.0;
        }
    pthread_create(&thr, NULL, thread_run, this);
    pthread_detach(thr);
    cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);

    //设置概率Hough参数
    finder.setLineLengthAndGap(100, 20);
    finder.setMinVote(80);
}

int Kinect::observeSideObstacle()
{
//    libfreenect2::FrameMap frames;
//    cout << "before wait"<<endl;
//    if(!listener->waitForNewFrame(frames,1000))
//    {
//        std::cout << "*** CameraFreenect2: Failed to get frames!" << std::endl;
//        return -1;
//    }
//    cout << "after wait" <<endl;
//    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

//    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);



    //cv::imshow("rgb", depthmat);
    float *dep_ = (float*)depthmat2.data;		//将深度图信息放到dep数组中

    for (int i=0; i<424; i++)
        for (int j=0; j<512; j++)
            depobstacle[i][j] = dep_[i*512+j];

//    static int pic_count = 0;
//    pic_count++;
//    std::string name = "output";
//    name += char(48+pic_count);
//    name += ".jpg";
//    cv::imwrite(name, depthmat);

    int black_count = 0;
    for(int i=300;i<424;i++)
    {
        for(int j=0;j<20;j++)
        {
            if(depobstacle[i][j] == 0)
                ++black_count;
        }
    }
    //listener->release(frames);
    if(black_count > 20*115)
    {
        cout << "right obstacle" <<endl;
        return 1;
    }
    else
        return 0;
}

int cmp(Kinect::type x, Kinect::type y)
{
    return x.x < y.x;
}

double sqr(double x)
{
    return x*x;
}

void Kinect::Observe()
{

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
//    int k = 25;	//调整参数以得到圆
//    int minR = 20;
//    int minDis = 2;
    libfreenect2::FrameMap frames;
    double dis = 600;
    double width = 310;
    while(!protonect_shutdown)
    {
        //cout << "before wait" <<endl;
        listener->waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        //cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat2);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(depthmat);


        //cv::imwrite("output1.jpg",depthmat);
        //cv::imshow("rgb", depthmat);
        float *dep_ = (float*)depthmat.data;		//将深度图信息放到dep数组中
        cv::cvtColor(rgbmat, hsv, CV_BGR2HSV);	//bgr图转hsv图
        CvScalar now;
        //通过颜色对hsv图进行过滤
        for (int i=0;i<hsv.rows;i++)
            for (int j=0;j<hsv.cols;j++)
            {
                now = Get(&hsv, i, j);
                if (i < 100 || i > hsv.rows-400 || j < 300 || j > hsv.cols-300)
                    ChangeColor(&hsv,i,j,0,0,0);
                else
                if (now.val[0]<185 && now.val[0]>160)
                    ChangeColor(&hsv,i,j,245,255,255);
                else if (now.val[0]<73 && now.val[0]>57
                    //&& now.val[1]<240 && now.val[1]>15
                    //&& now.val[2]<240 && now.val[2]>15
                    )
                    ChangeColor(&hsv,i,j,250,255,255);
                else
                    ChangeColor(&hsv,i,j,0,0,0);
            }

        // 腐蚀操作
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 7*MORPH_ELLIPSE + 1, 7*MORPH_ELLIPSE+1 ),
                                             Point( MORPH_ELLIPSE, MORPH_ELLIPSE ) );
        erode(hsv , hsv, element );
        cv::cvtColor(hsv, graymat, CV_BGR2GRAY);
        GaussianBlur(graymat, graymat, Size(7, 7), 2, 2);
        //霍夫圆
        int minDis_, k_, minR_;
        minDis_ = 20;
        minR_ = 200;
        k_ = 25;
        while (minDis_ > 0)
        {
            circles.clear();
            HoughCircles(graymat, circles, CV_HOUGH_GRADIENT, 1, minDis_, 240, k_, minR_, 1000);
            if ((int)circles.size() < 30)
            {
                minDis_ --;
                minR_ -= 10;
                if (minDis_ > 0)
                {
                    minDis_ --;
                    minR_ -= 10;
                }
                if (minDis_ >7)
                {
                    minDis_ --;
                    minR_ -= 10;
                }
            }
            else break;
        }
        if ((int)circles.size() > 150) circles.clear();
        /*
        while (1)
        {
            HoughCircles(graymat, circles, CV_HOUGH_GRADIENT, 1, 5, 240, k, 25, 500);
            if (T> 100) break;
            if ((int)circles.size() < 20){T+=2;circles.clear();k--;}
            else if ((int)circles.size() > 50){T+=10;circles.clear();k+=5;}
            else break;
        }
        */
        printf("%d\n" ,(int)circles.size());
        for (int i=0; i<256; i++)
            flag[i]=0;

        //通过有效颜色在圆中/圆边界占比，判断是否应该舍弃该圆
        for (size_t i=0; i< circles.size(); i++)
            if (Per(circles[i][0], circles[i][1], circles[i][2])<0.8
               || isCircle(circles[i][0], circles[i][1], circles[i][2])<0.95)
                flag[(int)i]=1;

        //对圆重叠进行判断
        while (1)
        {
            size_t Now;
            size_t i;
        //找到未扫描过的圆，找到所有与其重叠（包括：两个圆不重叠，但有另一个圆与这两个圆同时重叠）的圆并标记
            for (i = 0; i < circles.size(); i++)
                if (flag[(int)i] == 0)
                {
                    Now = Floodfill(i);
                    break;
                }

            if (i >= circles.size()) break;
            //通过两种方式得到最优圆
            //1.有效颜色占比超过0.9的圆中最大的圆
            //2.没有有效颜色占比超过0.9的圆时采用有效颜色占比最高的圆
            double max = Per(circles[Now][0], circles[Now][1], circles[Now][2]);
            size_t best = Now;
            int maxR = -1;

            for (i = 0; i < circles.size(); i++)
            {
                if (flag[(int)i] == 2)
                {
                    flag[(int)i] = 1;
                    if ((circles[i][2]*1.1) < circles[Now][2])
                        continue;
                    double p = Per(circles[i][0], circles[i][1], circles[i][2]);
                    if (p > max)
                    {
                        best = i;
                        max = p;
                    }
                    if (p > 0.90  &&  (maxR == -1 || circles[i][2] > circles[(size_t)maxR][2]))
                        maxR = (int)i;
                }
            }
            if (maxR != -1) best = (size_t)maxR;
            flag[(int)best] = 3;
        }

        type Circle[10];
        int tot=0;
        for (size_t i = 0; i < circles.size(); i++)
        {
            if (flag[(int)i] != 3) continue;
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            //绘制圆心
            circle(rgbmat, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            //绘制圆轮廓
            circle(rgbmat, center, radius, Scalar(155, 50, 255), 3, 8, 0);
            if (tot < 3)
            {
                CvScalar mid = Color(circles[i][0], circles[i][1], circles[i][2]);	//得到圆颜色
                Circle[tot].RorG = (int)mid.val[0];
                Circle[tot].x = circles[i][0];
                Circle[tot].y = circles[i][1];
                Circle[tot].r = circles[i][2];
                //std::cout << (int)mid.val[0] << ' ' << (int)mid.val[1] << ' ' << (int)mid.val[2] << std::endl;
                Circle[tot].Dis = Dist(dep_, circles[i][0], circles[i][1], circles[i][2]);	//得到圆距离
            }
            tot++;
            //std::cout << Dist(dep_, circles[i][0], circles[i][1], circles[i][2]) << std::endl;
        }
        printf("tot = %d\n", tot);
        if (tot == 3)
        {
            sort(Circle, Circle+3, cmp);
            double argDis = (Circle[0].Dis + Circle[1].Dis + Circle[2].Dis)/3.0;
            printf("dis0=%f,dis1=%f,dis2=%f\n", Circle[0].Dis, Circle[1].Dis, Circle[2].Dis);
            if (sqr(argDis-Circle[0].Dis) + sqr(argDis-Circle[1].Dis) + sqr(argDis-Circle[2].Dis) < 750.0)
            {
                printf("%0.2lf\n", argDis);
                if (argDis < 750.0)
                {
                    if (Circle[0].RorG == 250)
                    {
                        rg_message = "turnrrg";
                        printf("Right\n");
                    }
                    else if (Circle[1].RorG == 250)
                    {
                        rg_message = "turnrgr";
                        printf("Go\n");
                    }
                    else if (Circle[2].RorG == 250)
                    {
                        rg_message = "turngrr";
                        printf("Left\n");
                    }
                    else
                    {
                        rg_message = "turnrrr";
                        printf("Stop\n");
                    }

                }
            }
        }
        Mat result;
        cvtColor(rgbmat, result, CV_BGR2GRAY);
        //应用Canny算法
        Mat contours;
        Canny(result,   //灰度图
            contours,   //输出轮廓
            125,    //低阈值
            350);   //高阈值


        //检测并绘制直线
        vector<Vec4i>reslines = finder.findLines(contours);

        for (vector<Vec4i>::iterator it2 = reslines.begin();it2!= reslines.end();it2++)
        {
            cout << (*it2) <<(double((*it2)[0]-(*it2)[2]))/((*it2)[1]-(*it2)[3]) <<endl;
        }
        finder.drawDetectedLines(rgbmat);

        float *dep2_ = (float*)depthmat2.data;		//将深度图信息放到dep数组中

        cv::imshow("rgb", rgbmat);
        if(!isWorking)
        {
            //sleep(1);
            int key = cv::waitKey(1);
            listener->release(frames);
            continue;
        }
        for (int i=0; i<424; i++)
            for (int j=0; j<512; j++)
                depobstacle[i][j] = dep2_[i*512+j];

        for (int i=0; i<414; i++)
            for (int j=0; j<512; j++)
            {
                if (depobstacle[i][j] < 1e-8) depobstacle[i][j] = 500.0;
                //if (init[i][j] < 1e-8) init[i][j] = 500.0;
            }

        int minj = 512, maxj = 0, mindep = 0;
        int Flag = 0;

        for (int i=0; i<414; i++)
            for (int j=100; j<485; j++)
            {
                Flag = 0;
                for (int k = i-8; k <= i+8; k++)
                    for (int l = j-8; l <= j+8; l++)
                        if (!isObst(k, l))
                        {
                            Flag = 1;
                            break;
                        }
                if (!Flag)
                {
                    if (j < minj) minj = j;
                    if (j > maxj) maxj = j;
                    mindep++;
                    dep2_[i*512+j] = 0;
                }
            }


        if (mindep > 5000)
        {           
            //double  sendF= (double( minj - 256) / 512.0 * 70.0) - 5;
            double tmpF = (double( maxj - 256) / 512.0 * 70.0);
            angle = atan((dis * tan(tmpF * PI / 180) +width)/dis) / PI * 180;
            robot_status = 1;
            printf("tmpF =%f, angle = %f\n",tmpF,angle);
            printf("Obstacle, Stop\n");
            cv::imwrite("depthmat.jpg",depthmat);
            cv::imwrite("depthmat2.jpg",depthmat2);
            cv::imwrite("rgbmat.jpg",rgbmat);
        }
        else
        {
            robot_status = 0;
        }
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener->release(frames);
        //sleep(0.5);
    }
}



void Kinect::ChangeColor(Mat *p, int x, int y, int r, int g, int b)
{
    p->data[(x*p->cols+y)*3]=r;
    p->data[(x*p->cols+y)*3+1]=g;
    p->data[(x*p->cols+y)*3+2]=b;
}

CvScalar Kinect::Get(Mat *p, int x, int y)
{
    CvScalar t;
    t.val[0] = p->data[(x*p->cols+y)*3];
    t.val[1] = p->data[(x*p->cols+y)*3+1];
    t.val[2] = p->data[(x*p->cols+y)*3+2];
    return t;
}

size_t Kinect::Floodfill(size_t x)
{
    size_t now = x;
    flag[(int)x] = 2;
    for (size_t i = 0; i < circles.size(); i++)
        if (flag[(int)i] == 0)
        {
            double Dis=dist(circles[i][0],circles[i][1],circles[x][0],circles[x][1]);
            if (Dis < circles[x][2] || Dis < circles[i][2])
            {
                size_t y = Floodfill(i);
                if (circles[y][2] > circles[now][2])
                    now = y;
            }
        }
    return now;
}

CvScalar Kinect::Color(double xx, double yy, double rr)
{
    int r=cvRound(rr);
    CvScalar now;
    int sumH[256], sumS[256], sumV[256];
    int sum;
    for (int i = 0; i < 255; i ++)
        sumH[i]=sumS[i]=sumV[i]=0;
    sum=0;

    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            if (cvRound(xx)+j > hsv.cols || cvRound(xx)+j < 0) continue;
            if (cvRound(yy)+i > hsv.rows || cvRound(yy)+i < 0) continue;
            now = Get(&hsv, cvRound(yy)+i, cvRound(xx)+j);
            if (now.val[0] == 0 && now.val[1] == 0 && now.val[2] == 0) continue;
            sum ++;
            sumH[(int)now.val[0]] ++;
            sumS[(int)now.val[1]] ++;
            sumV[(int)now.val[2]] ++;
        }
    }

    int p;
    CvScalar mid;
    for (mid.val[0]=p=0;mid.val[0] < 255 && p<sum/2;mid.val[0]++,p+=sumH[(int)mid.val[0]]);
    for (mid.val[1]=p=0;mid.val[1] < 255 && p<sum/2;mid.val[1]++,p+=sumS[(int)mid.val[1]]);
    for (mid.val[2]=p=0;mid.val[2] < 255 && p<sum/2;mid.val[2]++,p+=sumV[(int)mid.val[2]]);

    return mid;
}

double Kinect::isCircle(double xx, double yy, double rr)
{
    int r=cvRound(rr);
    CvScalar now;
    int sum=0, sumAll=0;
    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            if (cvRound(xx)+j > hsv.cols || cvRound(xx)+j < 0) continue;
            if (cvRound(yy)+i > hsv.rows || cvRound(yy)+i < 0) continue;
            sumAll += 1;
            now = Get(&hsv, cvRound(yy)+i, cvRound(xx)+j);
            if (now.val[0] == 0 && now.val[1] == 0 && now.val[2] == 0) continue;
            sum += 1;
            if (j == -p+3 && j < p-4) j = p-4;
        }
    }
    return (double(sum)/double(sumAll));
}

double Kinect::Per(double xx, double yy, double rr)
{
    int r=cvRound(rr);
    CvScalar now;
    int sum=0, sumAll=0;
    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            if (cvRound(xx)+j > hsv.cols || cvRound(xx)+j < 0) continue;
            if (cvRound(yy)+i > hsv.rows || cvRound(yy)+i < 0) continue;
            sumAll += 1;
            now = Get(&hsv, cvRound(yy)+i, cvRound(xx)+j);
            if (now.val[0] == 0 && now.val[1] == 0 && now.val[2] == 0) continue;
            sum += 1;
        }
    }
    return double(sum)/double(sumAll);
}

double Kinect::Dist(float *dep, double xx, double yy, double rr)
{
    int r=cvRound(rr);
    double sumDep = 0, sumD = 0;
    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            int x, y;
            x = cvRound(yy)+i;
            y = cvRound(xx)+j;
            if (x > depthmat.rows || x < 0) continue;
            if (y > depthmat.cols || y < 0) continue;
            if (!isinf(dep[x*depthmat.cols+y]) && !isnan(dep[x*depthmat.cols+y])
                    && dep[x*depthmat.cols+y] > 0 && dep[x*depthmat.cols+y] < 5000.0)
            {
                sumDep += dep[x*depthmat.cols+y];
                sumD+=1;
            }
        }
    }
    return sumDep/(sumD+1e-8);
}

bool Kinect::isObst(int i, int j)
{
    if (i < 0 || j < 0 || i >= 424 || j >= 512) return 0;
    return init[i][j] - depobstacle[i][j] > 20 && depobstacle[i][j] < 650;
}

void *thread_run(void *args)
{
    Kinect *k = static_cast<Kinect *>(args);
    k->Observe();
}
