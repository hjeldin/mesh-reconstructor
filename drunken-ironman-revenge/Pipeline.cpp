/* 
 * File:   Pipeline.cpp
 * Author: kinect
 * 
 * Created on 9 novembre 2012, 15.28
 */

#include "Pipeline.h"

Pipeline::Pipeline() {
}

Pipeline::Pipeline(const Pipeline& orig) {
}

Pipeline::~Pipeline() {
}

void Pipeline::AddPipelineStage(std::string s, pcl::Filter<PointType>* f){
    std::pair<std::string,pcl::Filter<PointType> *> newPair(s,f);
    std::pair<std::map<std::string,pcl::Filter<PointType>*>::iterator,bool> ret;
    ret = pipelineList.insert(newPair);
    if(ret.second == false){
        std::cout << "Existing pipeline stage. NOT ADDED"<<std::endl;
    }
}

bool Pipeline::Execute(MyPointCloud::ConstPtr pcPtr, MyPointCloud::Ptr out){
    boost::mutex::scoped_lock lock (mtx_);
    std::map<std::string,pcl::Filter<PointType> *>::iterator it;
    //boost::shared_ptr<MyPointCloud> cptr = boost::const_pointer_cast<MyPointCloud >(pcPtr);
    MyPointCloud::Ptr cptr = MyPointCloud::Ptr(new MyPointCloud);
    
    *cptr = *pcPtr;
    for(it = pipelineList.begin();it != pipelineList.end(); it++){
        //Execute stage of pipeline
        pcl::Filter<PointType> * fPtr = static_cast<pcl::Filter<PointType> *>(it->second);        
        fPtr->setInputCloud(cptr);
        fPtr->filter(*cptr);
    }
    *out = *cptr;
}