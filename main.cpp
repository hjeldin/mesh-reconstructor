/*
 * threadedApp.cpp
 *
 *  Created on: 25/ott/2012
 *      Author: kinect
 */
#include <pthread.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sstream>
#include <pcl/filters/passthrough.h>
#include <opencv2/opencv.hpp>
#include <pcl/registration/icp.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/filters/voxel_grid.h>
//#include "KinectWrapper.h"
#include <unistd.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <list>
using namespace std;


class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : nCloudSaved(0), copyCloud(0),viewer ("PCL OpenNI Viewer") {
    	 //prevCloud = 0;
    	 //prevCloud(new pcl::PointCloud<pcl::PointXYZ>);
    	pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0f, 2.0f);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(0.0f, 2.0f);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0f, 2.0f);

     }

     ~SimpleOpenNIViewer(){

     }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pass.setInputCloud(cloud);
            pass.filter(*filtered);

            pcl::copyPointCloud(*filtered,*downsampled);

            if (!viewer.wasStopped()){
                    viewer.showCloud (filtered);
                    if(copyCloud == 2){
                            /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
                            if(((pcl::PointCloud<pcl::PointXYZRGBA>)*cloud).width != 0 &&((pcl::PointCloud<pcl::PointXYZRGBA>)*cloud).height != 0)
                            pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(*filtered,*tmp_cloud);*/
                            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
                            sor.setInputCloud (downsampled);
                            sor.setLeafSize (0.01f, 0.01f, 0.01f);
                            sor.filter (*downsampled);

                            //normal_estimation.compute (&downsampled);

                            std::stringstream ss;
                            ss << "mesh" << nCloudSaved << ".pcd";
                            pcl::io::savePCDFileASCII(ss.str(), *cloud);
                            filesToICP.push_back(ss.str());
                            nCloudSaved++;
                            //first->stop();
                            copyCloud = 1;

                    }
                    //savedArray
            }

    }

    void cloud_cb_second (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pass.setInputCloud(cloud);
            pass.filter(*filtered);

            pcl::copyPointCloud(*filtered,*downsampled);

            if (!viewer.wasStopped()){
                    viewer.showCloud (filtered);
                    if(copyCloud == 1){
                            /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
                            if(((pcl::PointCloud<pcl::PointXYZRGBA>)*cloud).width != 0 &&((pcl::PointCloud<pcl::PointXYZRGBA>)*cloud).height != 0)
                            pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(*filtered,*tmp_cloud);*/
                            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
                            sor.setInputCloud (downsampled);
                            sor.setLeafSize (0.01f, 0.01f, 0.01f);
                            sor.filter (*downsampled);

                            //normal_estimation.compute (&downsampled);

                            std::stringstream ss;
                            ss << "mesh" << nCloudSaved << ".pcd";
                            pcl::io::savePCDFileASCII(ss.str(), *cloud);
                            filesToICP.push_back(ss.str());
                            nCloudSaved++;
                            copyCloud = 0;
                    }
                    //savedArray
            }

    }
    static void * threadedExecutePassthrough(void * thisptr){
            list<string> files = ((SimpleOpenNIViewer*)thisptr)->filesToICP;
            list<string>::iterator it;
            cout << "Executing passthrough on " << files.size() << " elements"<< endl;
            for(it = files.begin(); it!= files.end();it++){
                    pcl::PointCloud<pcl::PointXYZRGBA> Final;
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr toAlignptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
                    pcl::io::loadPCDFile(*it,*toAlignptr);
                    ((SimpleOpenNIViewer*)thisptr)->pass.setInputCloud(toAlignptr);
                    ((SimpleOpenNIViewer*)thisptr)->pass.filter(Final);
                    pcl::io::savePCDFile(*it,Final);
            }
            cout << "...Done" << endl;
    }
    static void * threadedExecuteICP(void * thisptr){
            list<string> files = ((SimpleOpenNIViewer*)thisptr)->filesToICP;
            list<string>::iterator it;
            int current = 0;
            pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
            //icp.setMaxCorrespondenceDistance (0.1);
            icp.setRANSACIterations(0);
            icp.setRANSACOutlierRejectionThreshold(0);
            icp.setMaximumIterations (100);
            icp.setEuclideanFitnessEpsilon(0.00000005);
            pcl::PointCloud<pcl::PointXYZRGBA> starting;
            stringstream ss;
            ss << "mesh0.pcd";
            pcl::io::loadPCDFile(ss.str(),starting);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr megamesh(new pcl::PointCloud<pcl::PointXYZRGBA>(starting));
            for(it = files.begin()++; it!= files.end();it++){
                    cout << "Using " << *it << " pcl";
                    pcl::PointCloud<pcl::PointXYZRGBA> toAlign,Final;
                    pcl::io::loadPCDFile(*it,toAlign);

                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr iPtr(new pcl::PointCloud<pcl::PointXYZRGBA>(toAlign));
                    icp.setInputCloud(iPtr);
                    icp.setInputTarget(megamesh);
                    icp.align(Final);
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr finalPtr(new pcl::PointCloud<pcl::PointXYZRGBA>(Final));
                    Eigen::Matrix4f transformation = icp.getFinalTransformation();
                    if(icp.hasConverged())
                            cout << "FINE!" << endl;
                    cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
                    cout << transformation << endl;
                    //cout << *it << endl;
                    *megamesh += *finalPtr;
            }
            pcl::io::savePCDFileASCII("finalnew.pcd", *megamesh);
            std::cout << "ICP End" << std::endl;
    }

    static void keycallback(const pcl::visualization::KeyboardEvent &b, void * mboh){
            int key = b.getKeyCode ();
            if (b.keyUp ()){
                     if(key == (int)'a'){
                             cout << "Getting point cloud" << std::endl;
                             ((SimpleOpenNIViewer *)mboh)->copyCloud = 2;
                     }
                     else if(key == (int)'n'){
                             pthread_create(&t,NULL,((SimpleOpenNIViewer *)mboh)->threadedExecutePassthrough, (SimpleOpenNIViewer *)mboh);
                     }
                     else if(key == 27){
                             pcl::visualization::CloudViewer * k = &(((SimpleOpenNIViewer *)mboh)->viewer);
                             k->wasStopped(0);
                     } else if(key==(int)'i'){
                             //pthread_t t;
                             pthread_create(&t,NULL,((SimpleOpenNIViewer *)mboh)->threadedExecuteICP, (SimpleOpenNIViewer *)mboh);
                     }
            }
    }
    void run ()
    {
            //first kinect
            first= new pcl::OpenNIGrabber("B00363210002036B");
            //second kinect
            second = new pcl::OpenNIGrabber("A00364A16016051A");
            boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
            boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> k =
                            boost::bind (&SimpleOpenNIViewer::cloud_cb_second, this, _1);

            first->registerCallback (f);
            second->registerCallback(k);
            viewer.registerKeyboardCallback(keycallback,(void *)this);
            first->start ();
            second->start();
            while (!viewer.wasStopped())
            {
                    //boost::this_thread::sleep (boost::posix_time::seconds (1));
                    pcl_sleep(1);
            }
            pthread_join(SimpleOpenNIViewer::t,NULL);
            first->stop ();
            second->stop();
    }
   public:
	pcl::Grabber* first;
	pcl::Grabber* second;
	int copyCloud;
	int nCloudSaved;
    pcl::visualization::CloudViewer viewer;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud;
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    list<string> filesToICP;
    static pthread_t t;
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> savedArray;
};
pthread_t SimpleOpenNIViewer::t = NULL;

 int mainlol ()
 {
	//KinectMotor km;
	//km.setLight(KinectMotor::LED_RED);
	//km.Open();
	//km.Move(0);
	//km.Close();
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
