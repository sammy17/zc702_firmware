// #include "node/MyTypes.h"
// #include "node/NodeClient.h"

#include "drivers/xbacksub.h"
#include "drivers/xfeature.h"

#include "include/xparameters.h"
#include <chrono>
#include <string.h>
#include <fstream>
#include <iostream>

// #include "detection/MyTypes.h"
#include "detection/ClientUDP.h"
#include "detection/Frame.h"
#include "detection/BGSDetector.h"

//V4L2 Includes

#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <string>
// #include <termios.h>
#include <csignal>
//#include<opencv2/opencv.hpp>

/***************** Macros (Inline Functions) Definitions *********************/

#define TX_BASE_ADDR 0x01000000
#define RGB_TX_BASE_ADDR 0x03000000
#define DDR_RANGE 0x01000000
#define RX_BASE_ADDR 0x02000000

#define AXILITES_BASEADDR 0x43C00000
#define CRTL_BUS_BASEADDR 0x43C10000
#define AXILITE_RANGE 0xFFFF

#define M_AXI_BOUNDING 0x21000000
#define M_AXI_FEATUREH 0x29000000


using namespace cv;
using namespace std;

/***************** Global Variables *********************/

XBacksub backsub;
XFeature feature;

int fdIP;
int fd; // A file descriptor to the video device
int type;
// uint8_t * ybuffer = new uint8_t[N];

uint8_t * src; 
uint8_t * rgb_src; 
uint8_t * dst; 

uint16_t *m_axi_bound;
uint16_t *m_axi_feature;

int feature_init(XFeature *ptr)
{
    ptr->Axilites_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, XPAR_FEATURE_0_S_AXI_AXILITES_BASEADDR);
    ptr->Crtl_bus_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, XPAR_FEATURE_0_S_AXI_CRTL_BUS_BASEADDR);
    ptr->IsReady = XIL_COMPONENT_IS_READY;
    return 0;
}

void feature_rel(XFeature *ptr)
{
    munmap((void *)ptr->Crtl_bus_BaseAddress, AXILITE_RANGE);
    munmap((void *)ptr->Axilites_BaseAddress, AXILITE_RANGE);
}

void feature_config()
{
    printf("config\n");
    XFeature_Set_frame_in(&feature, (u32)TX_BASE_ADDR);
    XFeature_Set_bounding(&feature, (u32)M_AXI_BOUNDING);
    XFeature_Set_featureh(&feature, (u32)M_AXI_FEATUREH);
}

int backsub_init(XBacksub *backsub_ptr)
{
    backsub_ptr->Axilites_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, XPAR_BACKSUB_0_S_AXI_AXILITES_BASEADDR);
    backsub_ptr->Crtl_bus_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, XPAR_XBACKSUB_0_S_AXI_CRTL_BUS_BASEADDR);
    backsub_ptr->IsReady = XIL_COMPONENT_IS_READY;
    return 0;
}

void backsub_rel(XBacksub *backsub_ptr)
{
    munmap((void *)backsub_ptr->Axilites_BaseAddress, AXILITE_RANGE);
    munmap((void *)backsub_ptr->Crtl_bus_BaseAddress, AXILITE_RANGE);
}

void backsub_config(bool ini)
{
    //printf("config\n");
    XBacksub_Set_frame_in(&backsub, (u32)TX_BASE_ADDR);
    //printf("config1\n");
    XBacksub_Set_frame_out(&backsub, (u32)RX_BASE_ADDR);
    //printf("config2\n");
    XBacksub_Set_init(&backsub, ini);
}

void print_config()
{
    printf("Is Ready = %d \n", XBacksub_IsReady(&backsub));
    printf("Frame in = %X \n", XBacksub_Get_frame_in(&backsub));
    printf("Frame out = %X \n", XBacksub_Get_frame_out(&backsub));
    printf("Init = %d \n", XBacksub_Get_init(&backsub));
}

void signalHandler(int signum)
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        perror("Could not end streaming, VIDIOC_STREAMOFF");
    }

    close(fd);

    //Release IP Core
    backsub_rel(&backsub);
    feature_rel(&feature);

    munmap((void *)src, DDR_RANGE);
    munmap((void *)dst, DDR_RANGE);
    munmap((void *)m_axi_bound, 80);
    munmap((void *)m_axi_feature, 5120 * 2);

    close(fdIP);

    exit(signum);
}

int main(int argc, char *argv[])
{

    signal(SIGINT, signalHandler);

    // Initialization communication link
    boost::asio::io_service io_service;
    ClientUDP client(io_service, "10.10.23.237", 8080);
    uint16_t frameNo = 0;
    const uint8_t cameraID = 0;

    // Initializing IP Core Starts here .........................
    fdIP = open("/dev/mem", O_RDWR);
    if (fdIP < 1)
    {
        perror(argv[0]);
        return -1;
    }

    src = (uint32_t *)mmap(NULL, DDR_RANGE, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, TX_BASE_ADDR);
    dst = (uint8_t *)mmap(NULL, DDR_RANGE, PROT_EXEC | PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, RX_BASE_ADDR);

    m_axi_bound = (uint16_t *)mmap(NULL, 80, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, M_AXI_BOUNDING);
    m_axi_feature = (uint16_t *)mmap(NULL, 5120 * 2, PROT_READ | PROT_WRITE, MAP_SHARED, fdIP, M_AXI_FEATUREH);

    if (backsub_init(&backsub) == 0)
    {
        printf("Backsub IP Core Initialized\n");
    }

    if (feature_init(&feature) == 0)
    {
        printf("Feature IP Core Initialized\n");
    }
    // Initializing IP Core Ends here .........................

    /******************Initializing V4L2 Driver Starts Here**********************/
    // 1.  Open the device
    fd = open("/dev/video0", O_RDWR);
    if (fd < 0)
    {
        perror("Failed to open device, OPEN");
        return 1;
    }

    // 2. Ask the device if it can capture frames
    v4l2_capability capability;
    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0)
    {
        // something went wrong... exit
        perror("Failed to get device capabilities, VIDIOC_QUERYCAP");
        return 1;
    }

    // 3. Set Image format
    v4l2_format imageFormat;
    imageFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    imageFormat.fmt.pix.width = 320;
    imageFormat.fmt.pix.height = 240;
    imageFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    imageFormat.fmt.pix.field = V4L2_FIELD_NONE;
    // tell the device you are using this format
    if (ioctl(fd, VIDIOC_S_FMT, &imageFormat) < 0)
    {
        perror("Device could not set format, VIDIOC_S_FMT");
        return 1;
    }


    v4l2_streamparm streamParm;
    memset(&streamParm, 0, sizeof(streamParm));
    streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //Must be set to V4L2_BUF_TYPE_VIDEO_CAPTURE.
    streamParm.parm.capture.timeperframe.numerator = 1;
    streamParm.parm.capture.timeperframe.denominator = 30;
    if (ioctl(fd, VIDIOC_S_PARM, &streamParm) == -1)
        return false;

    // 4. Request Buffers from the device
    v4l2_requestbuffers requestBuffer = {0};
    requestBuffer.count = 1;                          // one request buffer
    requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // request a buffer wich we an use for capturing frames
    requestBuffer.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &requestBuffer) < 0)
    {
        perror("Could not request buffer from device, VIDIOC_REQBUFS");
        return 1;
    }

    // 5. Quety the buffer to get raw data ie. ask for the you requested buffer
    // and allocate memory for it
    v4l2_buffer queryBuffer = {0};
    queryBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queryBuffer.memory = V4L2_MEMORY_MMAP;
    queryBuffer.index = 0;
    if (ioctl(fd, VIDIOC_QUERYBUF, &queryBuffer) < 0)
    {
        perror("Device did not return the buffer information, VIDIOC_QUERYBUF");
        return 1;
    }
    // use a pointer to point to the newly created buffer
    // mmap() will map the memory address of the device to
    // an address in memory
    char *buffer = (char *)mmap(NULL, queryBuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                fd, queryBuffer.m.offset);
    memset(buffer, 0, queryBuffer.length);

    // 6. Get a frame
    // Create a new buffer type so the device knows whichbuffer we are talking about
    v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;

    // Activate streaming
    type = bufferinfo.type;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
        perror("Could not start streaming, VIDIOC_STREAMON");
        return 1;
    }
    /******************Initializing V4L2 Driver Ends Here**********************/

    /***************************** Begin looping here *********************/
    //    auto begin = std::chrono::high_resolution_clock::now();
    bool isFirst = true;
    BGSDetector detector(30,
                          BGS_MOVING_AVERAGE,
                          false,
                          "./pca_coeff.xml",
                          false);
    for (;;)
    {
        // Queue the buffer
        auto begin = std::chrono::high_resolution_clock::now();
        if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0)
        {
            perror("Could not queue buffer, VIDIOC_QBUF");
            return 1;
        }

        // Dequeue the buffer
        if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0)
        {
            perror("Could not dequeue the buffer, VIDIOC_DQBUF");
            return 1;
        }

        int outFileMemBlockSize = bufferinfo.bytesused;
        int remainingBufferSize = bufferinfo.bytesused;

        auto begin2 = std::chrono::high_resolution_clock::now();
        // printf("t1\n");
        // for(int j=0;j<N;j++)
        // {
        //     ybuffer[j] = buffer[2*j];
        // }
        // printf("t2\n");

        memcpy(src, buffer, sizeof(uint32_t) * 76800 / 2);
        // printf("t3\n");
        //print_config();
        if (isFirst)
        {
            backsub_config(true);
            isFirst = false;
        }
        else
        {
            backsub_config(false);
        }
        // printf("t4\n");

        XBacksub_Start(&backsub);

        while (!XBacksub_IsDone(&backsub))
            ;
        //printf("backsub finished\n");
        auto end2 = std::chrono::high_resolution_clock::now();
        //printf("Elapsed time Backsub: %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end2-begin2).count());
        // for (int i=0;i<100;i++){
        // printf("src : %d , dst : %d \n",ybuffer[i],dst[i]);
        // }

        // Contour detection using opencv
        ;
        Mat mask = Mat(240, 320, CV_8UC1, dst);

        std::vector<Rect> detections = detector.detect(mask);

        int len = detections.size();
        if (len > 10)
        {
            len = 10;
        }
        //printf("Detection Length: %d",len);
        memset(m_axi_bound, 0, 80); // initialize bounds to 0
        for (int k = 0; k < len; k++)
        {
            m_axi_bound[k * 4 + 0] = detections.at(k).x;
            m_axi_bound[k * 4 + 1] = detections.at(k).y;
            m_axi_bound[k * 4 + 2] = detections.at(k).x + detections.at(k).width;
            m_axi_bound[k * 4 + 3] = detections.at(k).y + detections.at(k).height;
            //printf("testloop %d \n",k);
        }
        auto end3 = std::chrono::high_resolution_clock::now();
        feature_config();
        XFeature_Start(&feature);

        while (!XFeature_IsDone(&feature))
            ;
        auto end4 = std::chrono::high_resolution_clock::now();
        //printf("feature finished\nPrinting first histogram :\n");

        // for (int h=0;h<512;h++){
        //     printf("%d, ",m_axi_feature[h]);
        // }
        // printf("\n");
        //client.sendBinMask(dst);

        Frame frame;
        frame.frameNo = frameNo;
        frame.cameraID = cameraID;
        frame.detections.clear();
        frame.histograms.clear();
        for (int q = 0; q < len; q++)
        {
            BoundingBox bbox;
            bbox.x = detections[q].x;
            bbox.y = detections[q].y;
            bbox.width = detections[q].width;
            bbox.height = detections[q].height;
            frame.detections.push_back(bbox);

            vector<uint16_t> histogram(512);
            // for(int r=0;r<512;r++)
            // {
            //     histogram[r] = (uint16_t)detector.histograms[q].at<short>(r);
            // }
            std::copy(m_axi_feature + 512 * q, m_axi_feature + 512 * (q + 1), histogram.begin());
            frame.histograms.push_back(histogram);
        }
        frameNo++;

        client.send(frame);

        // outFile.close();
        auto end = std::chrono::high_resolution_clock::now();
        // printf("Elapsed time : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count());

        // char c=getch();
        // if (c=='q')
        //   break;
        printf("Elapsed time backsub : %lld us\n", std::chrono::duration_cast<std::chrono::microseconds>(begin2 - begin).count());
        printf("Elapsed time backsub : %lld us\n", std::chrono::duration_cast<std::chrono::microseconds>(end2 - begin2).count());
        printf("Elapsed time opencv  : %lld us\n", std::chrono::duration_cast<std::chrono::microseconds>(end3 - end2).count());
        printf("Elapsed time feature : %lld us\n", std::chrono::duration_cast<std::chrono::microseconds>(end4 - end3).count());
        printf("Elapsed time send    : %lld us\n", std::chrono::duration_cast<std::chrono::microseconds>(end - end4).count());
        //printf("Elapsed time : %lld us\n",std::chrono::duration_cast<std::chron$
    }

    //auto end = std::chrono::high_resolution_clock::now();
    /***************************** End looping here *********************/
    // printf("Elapsed time : %lld us\n",std::chrono::duration_cast<std::chrd::chrono::microseconds>(end-begin).count()/it);
    // end streaming
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        perror("Could not end streaming, VIDIOC_STREAMOFF");
        return 1;
    }

    close(fd);

    //Release IP Core
    backsub_rel(&backsub);
    feature_rel(&feature);

    munmap((void *)src, DDR_RANGE);
    munmap((void *)dst, DDR_RANGE);
    munmap((void *)m_axi_bound, 80);
    munmap((void *)m_axi_feature, 5120 * 2);

    close(fdIP);

    //printf("Elapsed time : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count()/1000);

    printf("Device unmapped\n");

    return 0;
}
