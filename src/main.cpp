#include <rosbag/bag.h>
#include <stdlib.h>
#include <stdio.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/types.h>
#include <time.h>
#include <sys/stat.h>
#include "IO/IO.h"

using namespace matcher;



int main()
{

    int dir;
    YAML::Node config = YAML::LoadFile("../config.yaml");
    std::string pathOut = config["pathOut"].as<std::string>();
    std::string rMethod = config["mapRegistration"].as<std::string>();

    std::string leafSize0 = config["voxelSize"].as<std::string>();
    float leafSize=std::stod(leafSize0);
    std::string icpConfigFilePath = config["icpConfig"].as<std::string>();
    std::string inputFiltersConfigFilePath = config["icpInputFilters"].as<std::string>();
    std::string mapPostFiltersConfigFilePath = config["icpPostFilters"].as<std::string>();
    std::string	rtkTopic="";
    std::vector<std::string> sourceClouds = config["sourceClouds"].as<std::vector<std::string>>();
    std::vector<std::string> sourceTrajectories = config["sourceTrajectories"].as<std::vector<std::string>>();
    int mapNumber=sourceClouds.size();




    std::string computeProbDynamicstr = config["icpComputeProbDynamic"].as<std::string>();
    bool computeProbDynamic=false;

    if (computeProbDynamicstr=="True" || computeProbDynamicstr=="true")
    {
        computeProbDynamic=true;
    }


    bool semantics=false;
    std::string semanticsstr = config["semantics"].as<std::string>();

    if (semanticsstr=="True" || semanticsstr=="true")
    {
        semantics=true;
    }


    //std::cout<<sourceBags.size()<<std::endl;


    std::cout<<"Loading point clouds, voxelizing and satistical outlier filter..."<<std::endl;
    std::vector<pcl::PointCloud<pcl::Normal>> normals;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
    for (int i=0; i<mapNumber; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc2 (new pcl::PointCloud<pcl::PointXYZRGBL>);

        if (pcl::io::loadPCDFile<pcl::PointXYZRGBL> (sourceClouds[i], *pc2) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
        std::cout << "Loaded "
                  << pc2->width * pc2->height
                  << " data points from "<<sourceClouds[i]<< std::endl;
        std::vector<int> indices;
        //  pcl::removeNaNFromPointCloud(*pc2, *pc2, indices);

        pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
        sor.setInputCloud (pc2);
        sor.setMinimumPointsNumberPerVoxel(2);
        sor.setLeafSize (leafSize, leafSize, leafSize);
        sor.filter (*pc2);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor2;
        sor2.setInputCloud (pc2);
        sor2.setMeanK (5);
        sor2.setStddevMulThresh (1.0);
        sor2.filter (*pc2);
        //    std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pc2, *pc2, indices);

        pcl::NormalEstimationOMP<pcl::PointXYZRGBL, pcl::Normal> ne;
        ne.setInputCloud (pc2);
        pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBL> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (0.2);
        ne.compute (*cloud_normals);
        normals.push_back(*cloud_normals);
        XYZRGBL.push_back(*pc2);
        pc2=NULL;
    }




    std::string currentPath="";



    //std::cout<<i<<std::endl;
    int mapCounter=0;
    //std::cout<<pathOut<<std::endl;
    //  std::ostringstream ss; ss<<(i);
    currentPath=pathOut;
    //std::cout<<currentPath<<std::endl;
    dir=mkdir (currentPath.c_str(),S_IRWXU);
    IO* Io =new IO();
    Io ->readClouds(XYZRGBL, normals, sourceTrajectories, currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic);
    delete Io;



}
