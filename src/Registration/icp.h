#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#pragma once



namespace matcher
{
class ICP {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ICP();
    virtual ~ICP();

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    void alignMaps(pcl::PointCloud<pcl::PointXYZRGBL> pcl_ref,  pcl::PointCloud<pcl::Normal> ref_normals, pcl::PointCloud<pcl::PointXYZRGBL> pcl_new, pcl::PointCloud<pcl::Normal> new_normals, PM::TransformationParameters prior, std::string currentPath, std::string filename, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, bool semantics, PM::TransformationParameters &T_previous_alignment);

    //bool compareFunction (std::string a, std::string b);

private:





};
}

