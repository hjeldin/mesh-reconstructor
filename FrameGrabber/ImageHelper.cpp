/* 
 * File:   ImageHelper.cpp
 * Author: hjeldin
 * 
 * Created on November 16, 2012, 6:43 AM
 */

#include <pcl/io/openni_camera/openni_depth_image.h>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include "ImageHelper.h"
#include <fstream>
#include <ios>
#include <XnCppWrapper.h>
#include <sstream>
using namespace std;

ImageHelper::ImageHelper() {
}

ImageHelper::ImageHelper(const ImageHelper& orig) {
}

ImageHelper::~ImageHelper() {
}

void ImageHelper::GetRGBFrame(const boost::shared_ptr<openni_wrapper::Image>& img,int Device){
    cv::Mat frameRGB=cv::Mat(img->getHeight(),img->getWidth(),CV_8UC3);
    img->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
    cv::Mat frameBGR;
    cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);
    stringstream ss;
    ss << "color"<< Device << ".bmp";
    cv::imwrite(ss.str().c_str(), frameBGR); 
    return;
}


void ImageHelper::GetDepthFrame(const boost::shared_ptr<openni_wrapper::DepthImage>& img,int Device){
    const xn::DepthMetaData& depthMD = img->getDepthMetaData();
    const XnDepthPixel* depthData = depthMD.Data();
    cv::Mat1f data = cv::Mat1f(480, 640);
    XnUInt32 width = depthMD.XRes();
    XnUInt32 height = depthMD.YRes();
    
    for(XnUInt32 nY = 0; nY < height; nY++){
        for(XnUInt32 nX = 0; nX < width; nX++){
              data[nY][nX] = (*depthData)/1000.0;
              depthData++;
        }
    }
    stringstream ss;
    ss << "depth"<< Device << ".raw";

    std::ofstream f (ss.str().c_str(), std::ios::binary);
    int rows = data.rows, cols = data.cols;
    f.write((char*)&rows, sizeof(int));
    f.write((char*)&cols, sizeof(int));
    f.write((char*)data.data, data.rows*data.cols*sizeof(float));
    f.close(); 
    return;
}

