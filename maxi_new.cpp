#include "drivers/xbacksub.h"
#include "drivers/xfeature.h"

#include "include/xparameters.h"
#include <chrono>
#include <string.h>
#include <fstream>
#include <iostream>

// #include "detection/MyTypes.h"
#include "detection/ClientUDP.h"
#include "detection/MyTypes.h"
#include "detection/BGSDetector.h"
#include <csignal>

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


uint16_t * m_axi_bound;
uint16_t * m_axi_feature;



int feature_init(XFeature * ptr){
    ptr->Axilites_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, XPAR_FEATURE_0_S_AXI_AXILITES_BASEADDR);
    ptr->Crtl_bus_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, XPAR_FEATURE_0_S_AXI_CRTL_BUS_BASEADDR);
    ptr->IsReady = XIL_COMPONENT_IS_READY;
    return 0;
}

void feature_rel(XFeature * ptr){
    munmap((void*)ptr->Crtl_bus_BaseAddress, AXILITE_RANGE);
    munmap((void*)ptr->Axilites_BaseAddress, AXILITE_RANGE);
}

void feature_config() {
    printf("config\n");
    XFeature_Set_frame_in(&feature,(u32)RGB_TX_BASE_ADDR);
    XFeature_Set_bounding(&feature,(u32)M_AXI_BOUNDING);
    XFeature_Set_featureh(&feature,(u32)M_AXI_FEATUREH);
}



int backsub_init(XBacksub * backsub_ptr){
    backsub_ptr->Axilites_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, XPAR_BACKSUB_0_S_AXI_AXILITES_BASEADDR);
    backsub_ptr->Crtl_bus_BaseAddress = (u32)mmap(NULL, AXILITE_RANGE, PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, XPAR_XBACKSUB_0_S_AXI_CRTL_BUS_BASEADDR);
    backsub_ptr->IsReady = XIL_COMPONENT_IS_READY;
    return 0;
}

void backsub_rel(XBacksub * backsub_ptr){
    munmap((void*)backsub_ptr->Axilites_BaseAddress, AXILITE_RANGE);
    munmap((void*)backsub_ptr->Crtl_bus_BaseAddress, AXILITE_RANGE);
}

void backsub_config(bool ini) {
    //printf("config\n");
    XBacksub_Set_frame_in(&backsub,(u32)TX_BASE_ADDR);
    //printf("config1\n");
    XBacksub_Set_frame_out(&backsub,(u32)RX_BASE_ADDR);
    //printf("config2\n");
    XBacksub_Set_init(&backsub, ini);
}

void print_config() {
    printf("Is Ready = %d \n", XBacksub_IsReady(&backsub));
    printf("Frame in = %X \n", XBacksub_Get_frame_in(&backsub));
    printf("Frame out = %X \n", XBacksub_Get_frame_out(&backsub));
    printf("Init = %d \n", XBacksub_Get_init(&backsub));
}


void signalHandler( int signum ) {
    cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    // terminate program

    //Release IP Core
    backsub_rel(&backsub);
    feature_rel(&feature);

    munmap((void*)src, DDR_RANGE);
    munmap((void*)dst, DDR_RANGE);
    munmap((void*)rgb_src, DDR_RANGE);
    munmap((void*)m_axi_bound, 80);
    munmap((void*)m_axi_feature, 5120*2);

    close(fdIP);

    exit(signum);
}


int main(int argc, char *argv[]) {

    signal(SIGINT, signalHandler);

    // Initialization communication link
    boost::asio::io_service io_service;
    ClientUDP client(io_service,"10.10.23.237",8080);
    uint16_t frameNo=0;
    const uint8_t cameraID = 0;

    // Initializing IP Core Starts here .........................
    fdIP = open ("/dev/mem", O_RDWR);
    if (fdIP < 1) {
        perror(argv[0]);
        return -1;
    }

    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FPS,30);
    cap.set(CV_CAP_PROP_CONVERT_RGB,true);


    src = (uint8_t*)mmap(NULL, DDR_RANGE,PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, TX_BASE_ADDR); 
    rgb_src = (uint8_t*)mmap(NULL, DDR_RANGE,PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, RGB_TX_BASE_ADDR); 
    dst = (uint8_t*)mmap(NULL, DDR_RANGE,PROT_EXEC|PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, RX_BASE_ADDR); 


    m_axi_bound = (uint16_t*)mmap(NULL, 80,PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, M_AXI_BOUNDING);
    m_axi_feature = (uint16_t*)mmap(NULL, 5120*2,PROT_READ|PROT_WRITE, MAP_SHARED, fdIP, M_AXI_FEATUREH);


    if(backsub_init(&backsub)==0) {
        printf("Backsub IP Core Initialized\n");
    }

    if(feature_init(&feature)==0) {
        printf("Feature IP Core Initialized\n");
    }
    // Initializing IP Core Ends here .........................

    
    BGSDetector detector(30,
                          BGS_MOVING_AVERAGE,
                          false,
                          "./pca_coeff.xml",
                          false);

    /***************************** Begin looping here *********************/
//    auto begin = std::chrono::high_resolution_clock::now();
    bool isFirst = true;
    bool isSecond = false;
    Mat img, grey;


    for (;;){
        // Queue the buffer
        auto begin = std::chrono::high_resolution_clock::now();

        if (isFirst){
            backsub_config(true);
            isFirst = false;
            isSecond = true;
        }
        if (isSecond){
            backsub_config(false);
            isSecond = false;
        }

        cap>>img;
        cv::cvtColor(img, grey, CV_BGR2GRAY);
        memcpy(rgb_src,img.data,76800*3);
        memcpy(src,grey.data,76800);

        auto begin2 = std::chrono::high_resolution_clock::now();

        XBacksub_Start(&backsub);

        while(!XBacksub_IsDone(&backsub));

        auto end2 = std::chrono::high_resolution_clock::now();

        Mat mask = Mat(240, 320, CV_8UC1, dst); 

        std::vector<cv::Rect> detections = detector.detect(mask);

            int len = detections.size();
            if (len>10){
                len = 10;
            }
            //printf("Detection Length: %d",len);
            memset(m_axi_bound,0,80); // initialize bounds to 0
            for (int k=0;k<len;k++){
                m_axi_bound[k*4+0] = detections.at(k).x;
                m_axi_bound[k*4+1] = detections.at(k).y;
                m_axi_bound[k*4+2] = detections.at(k).x + detections.at(k).width;
                m_axi_bound[k*4+3] = detections.at(k).y + detections.at(k).height;
                //printf("testloop %d \n",k);
            }
	auto end3 = std::chrono::high_resolution_clock::now();
        feature_config();
        XFeature_Start(&feature);
        
        while(!XFeature_IsDone(&feature));
	auto end4 = std::chrono::high_resolution_clock::now();


        Frame frame;
        frame.frameNo = frameNo;
        frame.cameraID = cameraID;
        frame.detections.clear();
        frame.histograms.clear();
        for(int q=0;q<len;q++)
        {
            BoundingBox bbox;
            bbox.x = detections[q].x;
            bbox.y = detections[q].y;
            bbox.width = detections[q].width;
            bbox.height = detections[q].height;
            frame.detections.push_back(bbox);

            vector<uint16_t> histogram(512);
      
            std::copy ( m_axi_feature+512*q, m_axi_feature+512*(q+1), histogram.begin() );
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
	printf("Elapsed time backsub : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(begin2-begin).count());
	printf("Elapsed time backsub : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end2-begin2).count());
	printf("Elapsed time opencv  : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end3-end2).count());
	printf("Elapsed time feature : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end4-end3).count());
	printf("Elapsed time send    : %lld us\n",std::chrono::duration_cast<std::chrono::microseconds>(end-end4).count());
	//printf("Elapsed time : %lld us\n",std::chrono::duration_cast<std::chron$    
}


    //Release IP Core
    backsub_rel(&backsub);
    feature_rel(&feature);

    munmap((void*)src, DDR_RANGE);
    munmap((void*)dst, DDR_RANGE);
    munmap((void*)rgb_src, DDR_RANGE);
    munmap((void*)m_axi_bound, 80);
    munmap((void*)m_axi_feature, 5120*2);

    close(fdIP);
     
    printf("Device unmapped\n");

    return 0;
}

