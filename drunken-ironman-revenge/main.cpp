/* 
 * File:   newmain.cpp
 * Author: kinect
 *
 * Created on 2 novembre 2012, 15.34
 */
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#include <cstdlib>
#include <pthread.h>
#include <sstream>
#include <string>
#include <pcl/io/openni_grabber.h>
#include <pcl-1.5/pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <list>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "utilities.h"


using namespace std;

boost::mutex mtx_,mtxAsync0,mtxAsync1,mtxAsync2;

pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
pcl::visualization::PCLVisualizer viewer("first device");
MyPointCloud::Ptr firstCloud(new MyPointCloud);
MyPointCloud::Ptr mycloud[3];
MyPointCloud::Ptr secondCloud(new MyPointCloud);
MyPointCloud::Ptr myScloud(new MyPointCloud);
MyPointCloud::Ptr thirdCloud(new MyPointCloud);
MyPointCloud::Ptr myTcloud(new MyPointCloud);
pcl::PassThrough<PointType> pass_;
bool saveClouds[3]= {false,false,false};
int nSaved = 0;
int nDevices = 2;
list<string> files;

bool async = true;
pcl::Grabber * grabbers[3];
bool updatedText = true;
Eigen::Matrix4f transformation;
Eigen::Matrix<float,4,4> Aff,Aff2,A,B;

void * cloud_cb(const MyPointCloud::ConstPtr &cloud, int nDevice)
{
    boost::mutex::scoped_lock lock(mtx_);
    mycloud[nDevice].reset(new MyPointCloud);
    //vg.setInputCloud (cloud);
    //vg.filter (*(mycloud[nDevice]));
    pass_.setInputCloud(cloud);
    pass_.filter(*(mycloud[nDevice]));
    switch(nDevice){
        case 0:
            pcl::transformPointCloud(*mycloud[nDevice],*mycloud[nDevice],Aff2);
        break;
        case 1:
            pcl::transformPointCloud(*mycloud[nDevice],*mycloud[nDevice],Aff2);
            pcl::transformPointCloud(*mycloud[nDevice],*mycloud[nDevice],Aff);
        break;
    }
    //*mycloud[nDevice] = *cloud;
    if(saveClouds[nDevice]){
        cout << "Grabbing frame from kinect : " << nDevice << endl;
        stringstream ss;
        ss << "mesh" << nSaved << "_" << (nDevice+1) << ".pcd";
        pcl::io::savePCDFile(ss.str(),*(mycloud[nDevice]));
        saveClouds[nDevice] = false;
        files.push_back(ss.str());
    }
}

void StartDevice(int arg){
    int device = arg;
    try{
        stringstream ss;
        ss << "#" << (device+1);
        string deviceName = ss.str();
        pcl::Grabber* grab = new pcl::OpenNIGrabber(deviceName.c_str());
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> k =
                    boost::bind (&cloud_cb, _1,device);        
        grab->registerCallback(k);
        grab->start();
        grabbers[device] = grab;
    }catch(std::exception &e){
        boost::this_thread::sleep(boost::posix_time::seconds(5));
        stringstream ss;
        ss << "#" << (device+1);
        string deviceName = ss.str();
        pcl::Grabber* grab = new pcl::OpenNIGrabber(deviceName.c_str());
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> k =
                    boost::bind (&cloud_cb, _1,device);
        grab->registerCallback(k);
        grab->start();
        grabbers[device] = grab;
    }
}

void executeComplexICP(MyPointCloud::Ptr input, MyPointCloud::Ptr target, MyPointCloud::Ptr out){
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setMaxCorrespondenceDistance (0.001);
    icp.setRANSACIterations(10);//10
    icp.setRANSACOutlierRejectionThreshold(0.01);//0.05);
    icp.setMaximumIterations (10);//100
    //icp.setEuclideanFitnessEpsilon(0.000000005);
    icp.setInputCloud(input);
    icp.setInputTarget(target);
    icp.align(*out);
    transformation = icp.getFinalTransformation();
    if(icp.hasConverged())
            cout << "FINE!" << endl;
    cout << "has converged:"<< ANSI_COLOR_RED << icp.hasConverged() << ANSI_COLOR_RESET << " score: " << icp.getFitnessScore() << std::endl;
    cout << transformation << endl;
}

//eseguo icp tra le mesh di acquisite da 1 kinect
void executeICPonList(list<string> lst,string fileNameOut){
    list<string>::iterator it;
    MyPointCloud target;
    pcl::io::loadPCDFile(*lst.begin(),target);
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr megamesh(new pcl::PointCloud<pcl::PointXYZRGBA>(target));
    
    for(it=lst.begin()++;it!=lst.end();it++){
        MyPointCloud input;
        MyPointCloud::Ptr out(new MyPointCloud);
        pcl::io::loadPCDFile((*it).c_str(),input);
        MyPointCloud::Ptr inputPtr(new MyPointCloud(input));
        //richiamo icp meshINIZIALE(target) e meshN..(input) e mi riporta il risultato in out
        executeComplexICP(inputPtr,megamesh,out);//cambio targetPtr con megamesh
        *megamesh += *out;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vuoto (new pcl::PointCloud<pcl::PointXYZRGBA>);
     vuoto->width    = 500;
     vuoto->height   =500;
     vuoto->points.resize (vuoto->width * vuoto->height);
     for (int i = 0; i < vuoto->points.size (); ++i){
        vuoto->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        vuoto->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        vuoto->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
     }
     pcl::io::savePCDFileASCII(fileNameOut, *megamesh);	
     pcl::io::savePLYFileASCII("ciao.ply", *vuoto);
}

void executeICP(int ncams){
    list<string>::iterator it;
    
    boost::thread t[ncams];
    list<string> * camFiles = new list<string>[ncams];
    list<string> partString;
    //itero sui tutti file .pcd e li inserisvo ordinatamente in nella lista camfiles
    for(it=files.begin();it!=files.end();it++){
        int device = 0;
        vector<string> res;
        //prendo la sottostringa dopo il carattere "_" e la salvo in res
        StringExplode(*it,"_", &res);
        //eseguo l atoi del numero per ottenere il numero del device
        device = atoi(static_cast<string>(res.at(1)).substr(0,1).c_str());
        camFiles[device-1].push_back(*it);
    }
    //itero sul numero delle camere
    for(int n=0;n<ncams;n++){
        //stampo ogni contenuto per righe
        for(it=camFiles[n].begin();it!=camFiles[n].end();it++){
            cout << *it << endl;
        }
        //richiamo il metodo per fare icp tra tutti le mesh di una camera
        cout << ANSI_COLOR_RED << "Spawning thread for ICP on cam " << n << ANSI_COLOR_RESET << endl;
        stringstream ss;
        ss<< "part_" << n<<".pcd";
        partString.push_back(ss.str());
//        executeKeypointsExtraction(camFiles[n]);
        //executeICPonList(camFiles[n],ss.str());
        t[n] = boost::thread(executeICPonList,camFiles[n], ss.str());
    }
    //Join su thread di elaborazione
    for(int i = 0; i<ncams;i++){
       cout << ANSI_COLOR_RED << "Joining on elaboration threads"<< ANSI_COLOR_RESET << endl;
       t[i].join();
    }
    //Eseguo ICP su elaborazione finale
    cout << ANSI_COLOR_RED << "Starting final elaboration" << ANSI_COLOR_RESET << endl;
    executeICPonList(partString,string("finalNew.pcd"));
    char command[] = "pcd_viewer finalNew.pcd";
    int status = system( command );
}

void StartAll(){
    for(int i=0; i<nDevices;i++){
        grabbers[i]->start();
    }
}

void StopAll(){
    for(int i=0; i<nDevices;i++){
        grabbers[i]->stop();
    }
}

static void keycallback(const pcl::visualization::KeyboardEvent &b, void * mboh){
    int key = b.getKeyCode ();
    if (b.keyUp ()){
        if(key == (int)'a'){
            nSaved++;
            if(async){
                cout << "Getting point cloud" << std::endl;
                for(int i=0; i<nDevices;i++){
                    saveClouds[i] = true;
                }
            }else {
                for(int i=0; i<nDevices;i++){
                    saveClouds[i] = true;
                    grabbers[i]->start();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(30));
                    boost::mutex::scoped_lock lock(mtx_);
                    grabbers[i]->stop();
                  }
            }
        }else if(key == (int)'q'){
            StopAll();
        }else if(key == (int)'s'){
            async = !async;
            updatedText = false;
            if(!async){
                StopAll();
            }else{
                StartAll();
            }
        } else if(key == (int)'n'){
            //Executes ICP
            boost::thread workerThread(executeICP,nDevices);
        } else if(key == 27){
            //Stops grabbers
            boost::thread wk(StopAll);
        } else if(key==(int)'c'){
            //Clears file cache
            nSaved = 0;
            files.clear();
            viewer.removeAllPointClouds(0);
            viewer.addPointCloud<pcl::PointXYZRGBA>(firstCloud,"kinect1");
            viewer.addPointCloud<pcl::PointXYZRGBA>(secondCloud,"kinect2");
            viewer.addPointCloud<pcl::PointXYZRGBA>(thirdCloud,"kinect3");
        }
    }
}


int main(int argc, char** argv) {
    Eigen::Matrix<float,3,3> Reigen,Reigen3;
    Eigen::Matrix<float,3,1> Teigen,Teigen3;
    Eigen::Matrix<float,3,3> Ident;
    Eigen::Matrix<float,3,1> vett_colomn;
    Eigen::Matrix<float,1,4> vett_riga;
    cv::Mat R,T;
    cv::FileStorage fs("calibration_multikinect.yml", cv::FileStorage::READ);

    fs["R_extrinsics"] >> R;
    fs["T_extrinsics"] >> T;
    cv2eigen(R,Reigen);
    cv2eigen(T,Teigen);

    Reigen3 << 1.f,0.f,0.f,
                0.f,-1,-0.f,
                0.f,0.f,-1.f;
   
    Teigen3 << 0.f,0.f,0.f;
   
    vett_colomn <<      0.0,
                        0.0,
                        0.0;
    
    Ident << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0;
    
    vett_riga << 0.0, 0.0, 0.0, 1.0;
    
    A << Ident,Teigen,vett_riga;
    
    B << Reigen,vett_colomn, 0.0, 0.0, 0.0, 1.0;
    
    cout << "Matrice di Rototraslazione ->" << endl;
    Aff= A*B;
    cout << ANSI_COLOR_GREEN << Aff << ANSI_COLOR_RESET << endl;
    
    Aff2 << Reigen3, Teigen3, 0.0, 0.0, 0.0, 1.0f;
    
    for(int i = 0; i<nDevices;i++)
        mycloud[i] = MyPointCloud::Ptr(new MyPointCloud);
    
    pass_.setFilterFieldName ("z");
    pass_.setFilterLimits (0, 2.0f);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();
    int v1(0);
    viewer.addText("Simultaneo", 10, 10, "v1 text", v1);
    viewer.addPointCloud<pcl::PointXYZRGBA>(firstCloud,"kinect1");
    viewer.addPointCloud<pcl::PointXYZRGBA>(secondCloud,"kinect2");
    viewer.addPointCloud<pcl::PointXYZRGBA>(thirdCloud,"kinect3");
    
    
    viewer.registerKeyboardCallback(keycallback,0);
    
    switch (nDevices){
        case(3):
            StartDevice(2);
        case(2):
           StartDevice(1);
        case(1):
           StartDevice(0);
    };
    
    while (!viewer.wasStopped ())
    {
        if(!updatedText){
            if(async){
                viewer.removeText3D("v1 text", 0);
                viewer.addText("Simultaneo",10,10,"v1 text",v1);
            } else {
                viewer.removeText3D("v1 text", 0);
                viewer.addText("Sequenziale",10,10,"v1 text",v1);
            }
            updatedText = true;
        }
        if(mycloud[0]){
            boost::mutex::scoped_lock lock (mtx_);
            MyPointCloud::Ptr tmp_point;
            tmp_point.swap(mycloud[0]);
            viewer.updatePointCloud(tmp_point,"kinect1");
        }
        if(mycloud[1]){
            boost::mutex::scoped_lock lock (mtx_);
            MyPointCloud::Ptr second_tmp;
            second_tmp.swap(mycloud[1]);
          
            viewer.updatePointCloud(second_tmp,"kinect2");
            
        }
        if(mycloud[2]){
            boost::mutex::scoped_lock lock (mtx_);
            MyPointCloud::Ptr third_tmp;
            third_tmp.swap(mycloud[2]);
            viewer.updatePointCloud(third_tmp,"kinect3");
        }
        viewer.spinOnce (100);
    }
    return 0;
}

