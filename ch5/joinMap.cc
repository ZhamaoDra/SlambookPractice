#include<iostream>
#include<fstream>
using namespace std;

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<eigen3/Eigen/Geometry>
#include<boost/format.hpp>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>
#define IMAGE_NUM 5
#define POSE_DATA_NUM 7

int main(){
    vector<cv::Mat> colorImags,depthImags;
    vector<Eigen::Isometry3d> poses;

    ifstream fin("pose.txt");
    if(!fin.is_open()){
        cerr<<"fail to open pose.txt.. \n";
        exit(1);
    }
    //read the content of file color depth and pose
    for(int i=0;i<IMAGE_NUM;i++){
        boost::format fmt("%s/%d.%s");
        colorImags.push_back(cv::imread((fmt%"color"%(i+1)%"png").str()));
        depthImags.push_back(cv::imread((fmt%"depth"%(i+1)%"pgm").str(),    -1));
        
        double data[POSE_DATA_NUM] = {0};
        for(auto &i:data)
            fin>>i;
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d (data[0], data[1], data[2]));
        poses.push_back(T);
    }

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    Eigen::Matrix3d K,k1,k2;
    K << fx,0,cx,0,fy,cy,0,0,1;

    cout<<"making the photo into point cloud..\n";

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr pointCloud(new PointCloud);
    for(int i=0;i<IMAGE_NUM;i++){
        cout<<"for photo :"<<i<<"\n";
        cv::Mat color = colorImags[i];
        cv::Mat depth = depthImags[i];
        Eigen::Isometry3d T = poses[i];
        for(int v=0; v<color.rows;v++){
            for(int u=0; u<color.cols;u++){
                unsigned int d = depth.at<unsigned short>(v,u);
                double z;
                z = (double)d/depthScale;
                if(!d) continue;
                Eigen::Vector3d point;
                point << u,v,1;
                point = (z*K.inverse()*point);          //do not try to use K = K.inverse() which may casue a err 
                Eigen::Vector3d pointWorld = T*point;   //and do not try to assign it to another variable because the type double is not accuracy

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                pointCloud->points.push_back( p );
            }
        }
    }
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
    return 0;
}