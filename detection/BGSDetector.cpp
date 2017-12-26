//
// Created by dilin on 10/23/17.
//

#include "BGSDetector.h"

std::vector<cv::Rect> BGSDetector::detect(cv::Mat &img)
{
    std::vector<cv::Rect> detections,found;
    mask = img.clone();

    cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat structuringElement5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat structuringElement7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat structuringElement9x9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));

    /*
    cv::dilate(imgThresh, imgThresh, structuringElement7x7);
    cv::erode(imgThresh, imgThresh, structuringElement3x3);
    */

    cv::dilate(mask, mask, structuringElement5x5);
    cv::dilate(mask, mask, structuringElement5x5);
    cv::erode(mask, mask, structuringElement5x5);



    std::vector<std::vector<cv::Point> > contours;

    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > convexHulls(contours.size());

    for (unsigned int i = 0; i < contours.size(); i++)
    {
        cv::convexHull(contours[i], convexHulls[i]);
    }


   

    for (auto &convexHull : convexHulls)
    {
        Blob possibleBlob(convexHull);

        float record[8] = {
                possibleBlob.currentBoundingRect.x,
                possibleBlob.currentBoundingRect.y,
                possibleBlob.currentBoundingRect.width,
                possibleBlob.currentBoundingRect.height,
                possibleBlob.currentBoundingRect.area(),
                (float)possibleBlob.dblCurrentAspectRatio,
                (float)cv::contourArea(possibleBlob.currentContour),
                (float)possibleBlob.dblCurrentDiagonalSize
        };

        if(trainingMode)
        {
            DetectionRecord dr;
            memcpy(dr.data,record,8* sizeof(float));
            data.push_back(dr);
            found.push_back(possibleBlob.currentBoundingRect);
        }
        else
        {
            Mat x1(1,8,CV_32F,record);
            Mat d = pca.project(x1);
            if(d.at<float>(0)>detectorTH)
                found.push_back(possibleBlob.currentBoundingRect);
        }

//        Mat x1(1,8,CV_64F,data);
//        Mat cf(8,8,CV_64F,coeff);
//
//        Mat d = x1 * cf;
//        if(d.at<double>(0)>4203)
//            found.push_back(possibleBlob.currentBoundingRect);
    }

    size_t i, j;

    for (i=0; i<found.size(); i++)
    {
        cv::Rect r = found[i];
        for (j=0; j<found.size(); j++)
            if (j!=i && (r & found[j])==r)
                break;
        if (j==found.size())
        {
//            r.x += cvRound(r.width*0.1);
//            r.width = cvRound(r.width*0.8);
//            r.y += cvRound(r.height*0.07);
//            r.height = cvRound(r.height*0.8);
            detections.push_back(r);
        }

    }

    return detections;
}


void BGSDetector::backgroundSubstraction(cv::Mat &frame0, cv::Mat &frame1, cv::Mat &frame2
        , cv::Mat &bgModel, cv::Mat &mask, double TH)
{
    cv::Mat frame0g,frame1g,frame2g;

   // convert frames to gray
    cvtColor(frame0,frame0g,cv::COLOR_BGR2GRAY);
    cvtColor(frame1,frame1g,cv::COLOR_BGR2GRAY);
    cvtColor(frame2,frame2g,cv::COLOR_BGR2GRAY);

//    cv::GaussianBlur(frame0g,frame0g,cv::Size(5, 5), 0);
//    cv::GaussianBlur(frame1g,frame1g,cv::Size(5, 5), 0);
//    cv::GaussianBlur(frame2g,frame2g,cv::Size(5, 5), 0);

    bgModel = 0.1*frame0g + 0.2*frame1g + 0.7*frame2g;

    cv::Mat diff;
    absdiff(frame0g,bgModel,diff);
    threshold(diff,mask,TH,255,cv::THRESH_BINARY);
}

BGSDetector::BGSDetector(double TH,
                         int method,
                         bool doGammaCorrection,
                         string coeffFilePath,
                         bool trainingMode) :
TH(TH),
method(method),
doGamaCorrection(doGammaCorrection),
trainingMode(trainingMode),
coeffFilePath(coeffFilePath)
{
    frameCount = 0;
    if(!trainingMode)
    {
        coeffFile.open(coeffFilePath,FileStorage::READ);
        if(!coeffFile.isOpened())
            throw runtime_error("Unable to load pca coefficients.");
        pca.read(coeffFile.root());
        coeffFile["TH"] >> detectorTH;
    }

}

void BGSDetector::GammaCorrection(cv::Mat &src, cv::Mat &dst, float fGamma)
{
    CV_Assert(src.data);

    // accept only char type matrices
    CV_Assert(src.depth() != sizeof(uchar));

    // build look up table
    unsigned char lut[256];
    for (int i = 0; i < 256; i++)
    {
        lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
    }

    dst = src.clone();
    const int channels = dst.channels();
    switch (channels)
    {
        case 1:
        {

            cv::MatIterator_<uchar> it, end;
            for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
                //*it = pow((float)(((*it))/255.0), fGamma) * 255.0;
                *it = lut[(*it)];

            break;
        }
        case 3:
        {

            cv::MatIterator_<cv::Vec3b> it, end;
            for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++)
            {

                (*it)[0] = lut[((*it)[0])];
                (*it)[1] = lut[((*it)[1])];
                (*it)[2] = lut[((*it)[2])];
            }

            break;

        }
    }

}

void BGSDetector::trainDetector()
{
    if(!trainingMode)
        throw runtime_error("Training is only available in training mode.");
    Mat dataMat((int)data.size(),8,CV_32F);
    for(int j=0;j<data.size();j++)
    {
        for(int k=0;k<8;k++)
        {
            dataMat.at<float>(j,k) = data[j].data[k];
        }
    }
    pca = PCA(dataMat,Mat(),PCA::DATA_AS_ROW,1);
    coeffFile.open(coeffFilePath,FileStorage::WRITE);
    if(!coeffFile.isOpened())
        throw runtime_error("Unable to open coefficient file.");
    pca.write(coeffFile);


    Mat matSrc = pca.project(dataMat);
    double minVal,maxVal;
    minMaxLoc(matSrc,&minVal,&maxVal);

    int nHistSize = 65536;
    float fRange[] = { (float)minVal, (float)maxVal};
    float binSize = ((float)maxVal -  (float)minVal)/nHistSize;
    const float* fHistRange = { fRange };

    Mat matHist;
    calcHist(&matSrc, 1, 0, cv::Mat(), matHist, 1, &nHistSize, &fHistRange);
    normalize(matHist,matHist,1,0,NORM_MINMAX);

    float total = 0.0;
    float sumB = 0;
    float wB = 0;
    float maximum = 0.0;
    float sum1 = 0;
    float level;
    for(int i=0;i<nHistSize;i++)
    {
        sum1 += (i*binSize+binSize/2+minVal)*matHist.at<float>(i);
        total += matHist.at<float>(i);
    }

    for(int i=0;i<nHistSize;i++)
    {
        wB = wB + matHist.at<float>(i);
        float wF = total - wB;
        if (wB == 0 || wF == 0)
            continue;
        sumB = sumB + (float) (i*binSize+binSize/2+minVal) * matHist.at<float>(i);
        float mF = (sum1 - sumB) / wF;
        float  between = wB * wF * ((sumB / wB) - mF) * ((sumB / wB) - mF);
        if ( between >= maximum )
        {
            level = (float)(i*binSize+binSize/2+minVal);
            maximum = between;
        }

    }

    cout << "TH: " << level << endl;

    coeffFile << "TH" << level;

    coeffFile.release();
}

Blob::Blob(std::vector<cv::Point> _contour)
{
    currentContour = _contour;

    currentBoundingRect = cv::boundingRect(currentContour);

    cv::Point currentCenter;

    currentCenter.x = (currentBoundingRect.x + currentBoundingRect.x + currentBoundingRect.width) / 2;
    currentCenter.y = (currentBoundingRect.y + currentBoundingRect.y + currentBoundingRect.height) / 2;

    centerPositions.push_back(currentCenter);

    dblCurrentDiagonalSize = sqrt(pow(currentBoundingRect.width, 2) + pow(currentBoundingRect.height, 2));

    dblCurrentAspectRatio = (float)currentBoundingRect.width / (float)currentBoundingRect.height;

    blnStillBeingTracked = true;
    blnCurrentMatchFoundOrNewBlob = true;

    intNumOfConsecutiveFramesWithoutAMatch = 0;
}
