/* 
 * File:   utilities.h
 * Author: kinect
 *
 * Created on 6 novembre 2012, 17.56
 */

#ifndef UTILITIES_H
#define	UTILITIES_H
#include <string>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBA> MyPointCloud;
typedef pcl::PointXYZRGBA PointType;

inline void StringExplode(string str, string separator, vector<string>* results){
    int found;
    found = str.find_first_of(separator);
    while(found != string::npos){
        if(found > 0){
            results->push_back(str.substr(0,found));
        }
        str = str.substr(found+1);
        found = str.find_first_of(separator);
    }
    if(str.length() > 0){
        results->push_back(str);
    }
}

#endif	/* UTILITIES_H */

