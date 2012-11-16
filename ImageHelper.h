/* 
 * File:   ImageHelper.h
 * Author: hjeldin
 *
 * Created on November 16, 2012, 6:43 AM
 */

#ifndef IMAGEHELPER_H
#define	IMAGEHELPER_H
#include <opencv/cv.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/openni_grabber.h>
class ImageHelper {
public:
    ImageHelper();
    ImageHelper(const ImageHelper& orig);
    virtual ~ImageHelper();
    
    static cv::Mat GetRGBFrame(const boost::shared_ptr<openni_wrapper::Image> &img);
    static cv::Mat GetDepthFrame(const boost::shared_ptr<openni_wrapper::DepthImage> &img);
    
private:

};

#endif	/* IMAGEHELPER_H */

