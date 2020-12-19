
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"

//#include <pcl/filters/filter.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "vector"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geodesy/utm.h>


#define foreach BOOST_FOREACH
#pragma once
namespace matcher
{
class IO {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IO();
    virtual ~IO();
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    struct Datacontainer {
        std::vector<std::vector<double>> vect;
        std::vector<std::string> header;
        std::string frame_id;
    } ;
    struct TFcontainer {
        std::vector<std::string> frame_id;
        std::vector<std::string> child_frame_id;
        std::vector<std::string> bothFrames;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transform; // 4x4 affine transform
      //  Eigen::Matrix3d rotation;  // qx qy qz qw

    } ;

    struct pCloudcontainer {
        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
        std::vector<pcl::PointCloud<pcl::Normal>> normals;
        pcl::PointCloud<pcl::Intensity> Intensity;
        std::vector<double> timestamp;
        std::string frame_id;
    } ;


   // void read_directory(const std::string& name, stringvec& v);
    void alignTrajectories(Eigen::MatrixXd& newTrajectory, Eigen::MatrixXd& oldTrajectory, PM::TransformationParameters& prior);
    void readClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL, std::vector<pcl::PointCloud<pcl::Normal>> normals, std::vector<std::string> sourceTrajectories, std::string currentPath, std::string rMethod , int mapNumber, bool semantics, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic);

    //bool compareFunction (std::string a, std::string b);

private:





};
}
