/* 
 * File:   main.cpp
 * Author: kinect
 *
 * Created on 20 novembre 2012, 15.49
 */
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "ImageHelper.h"
#include "ntk/camera/rgbd_grabber_factory.h"
#include "ntk/camera/rgbd_grabber.h"
#include "ntk/camera/rgbd_processor.h"
#include "ntk/camera/rgbd_frame_recorder.h"
using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
//    ntk::RGBDGrabberFactory gFactory;
//    ntk::RGBDGrabberFactory::Params params;
//
//    params.directory = "./";
//    params.type = ntk::RGBDGrabberFactory::OPENNI;
//    params.calibration_file = "calibration";
//    std::vector<ntk::RGBDGrabberFactory::GrabberData> grabbers;
//    
//    grabbers = gFactory.createGrabbers(params);
//    ntk::RGBDGrabber* grabber = grabbers[0].grabber;
//    ntk::RGBDProcessor* processor = grabbers[0].processor;
//
//    ntk::RGBDFrameRecorder frame_recorder ("calibration");
//    frame_recorder.setFrameIndex(0);
//    frame_recorder.setSaveOnlyRaw(true);
//    frame_recorder.setUseBinaryRaw(true);
//    frame_recorder.setSavePCLPointCloud(false);

    stringstream ss;
    ss << "#1";
    std::cout << sizeof(unsigned short) << std::endl;
    string deviceName = ss.str();
    pcl::Grabber* grab = new pcl::OpenNIGrabber(deviceName.c_str());
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> k =
                boost::bind (&ImageHelper::GetRGBFrame, _1,1);
    grab->registerCallback(k);
    
    boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)>
                f = boost::bind(&ImageHelper::GetDepthFrame, _1, 1);
    
    grab->registerCallback(f);
    grab->start();
    grab->stop();
    stringstream ss1;
    ss1 << "#2";
    std::cout << sizeof(unsigned short) << std::endl;
    deviceName = ss1.str();
    pcl::Grabber* grab1 = new pcl::OpenNIGrabber(deviceName.c_str());
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> n =
                boost::bind (&ImageHelper::GetRGBFrame, _1,2);
    grab1->registerCallback(n);
    
    boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)>
                m = boost::bind(&ImageHelper::GetDepthFrame, _1, 2);
    
    grab1->registerCallback(m);
    grab1->start();
    grab1->stop();
    
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    return 0;
}
