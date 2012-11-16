/* 
 * File:   ImageHelper.cpp
 * Author: hjeldin
 * 
 * Created on November 16, 2012, 6:43 AM
 */

#include <pcl/io/openni_camera/openni_depth_image.h>

#include "ImageHelper.h"

ImageHelper::ImageHelper() {
}

ImageHelper::ImageHelper(const ImageHelper& orig) {
}

ImageHelper::~ImageHelper() {
}

cv::Mat ImageHelper::GetRGBFrame(const boost::shared_ptr<openni_wrapper::Image>& img){
  cv::Mat frameRGB=cv::Mat(img->getHeight(),img->getWidth(),CV_8UC3);

  img->fillRGB(frameRGB.cols,frameRGB.rows,frameRGB.data,frameRGB.step);
  cv::Mat frameBGR;
  cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);

  return frameBGR;
}

cv::Mat ImageHelper::GetDepthFrame(const boost::shared_ptr<openni_wrapper::DepthImage>& img){
  cv::Mat frameRGB=cv::Mat(img->getHeight(),img->getWidth(),CV_16SC1);
  img->fillDepthImageRaw(frameRGB.cols,frameRGB.rows,(short unsigned int*)frameRGB.data,frameRGB.step);
  //cv::Mat frameBGR;
  //cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);

  return frameRGB;
}

