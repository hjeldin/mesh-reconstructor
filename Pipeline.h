/* 
 * File:   Pipeline.h
 * Author: kinect
 *
 * Created on 9 novembre 2012, 15.28
 */

#ifndef PIPELINE_H
#define	PIPELINE_H

#include <cstdlib>
#include <map>
#include <pthread.h>
#include <sstream>
#include <string>
#include <pcl-1.5/pcl/io/openni_grabber.h>
#include <pcl-1.5/pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <X11/Xlib.h>
#include <pcl-1.5/pcl/common/io.h>
#include <pcl-1.5/pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/bilateral.h>
#include <pcl/surface/mls.h>
#include <pcl-1.5/pcl/io/pcd_io.h>
#include <list>
#include <pcl/filters/voxel_grid.h>

#include "utilities.h"

class Pipeline {
public:
    Pipeline();
    Pipeline(const Pipeline& orig);
    virtual ~Pipeline();
    void AddPipelineStage(std::string, pcl::Filter<PointType>*);
    bool Execute(MyPointCloud::ConstPtr, MyPointCloud::Ptr);
private:
    std::map<std::string,pcl::Filter<PointType> *> pipelineList;
    boost::mutex mtx_;
};

#endif	/* PIPELINE_H */

