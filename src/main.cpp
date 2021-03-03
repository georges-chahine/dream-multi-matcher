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


    std::string loopCloseKF = config["loopCloseKF"].as<std::string>();

    std::string autoNameTemplate = config["autoNameTemplate"].as<std::string>();
    std::string leafSize0 = config["voxelSize"].as<std::string>();
    float leafSize=std::stod(leafSize0);
    std::string icpConfigFilePath = config["icpConfig"].as<std::string>();
    std::string inputFiltersConfigFilePath = config["icpInputFilters"].as<std::string>();
    std::string mapPostFiltersConfigFilePath = config["icpPostFilters"].as<std::string>();
    std::string	rtkTopic="";
    std::vector<std::string> sourceClouds = config["sourceClouds"].as<std::vector<std::string>>();
    std::vector<std::string> sourceTrajectories = config["sourceTrajectories"].as<std::vector<std::string>>();
    std::vector<std::string> autoMatchDir = config["autoMatchDir"].as<std::vector<std::string>>();
    std::string autoMatchStr = config["autoMatch"].as<std::string>();
    bool autoMatch=false;
    bool spatialAlignment=false;
    if (autoMatchStr=="True" || autoMatchStr=="true")
    {
        autoMatch=true;
    }

    std::string loopCloseStr = config["autoLoopClose"].as<std::string>();

    bool loopClose=false;

    if (loopCloseStr=="True" || loopCloseStr=="true")
    {
        loopClose=true;
    }



    int mapNumber=sourceClouds.size();
    int dirNumber=autoMatchDir.size();
    std::vector<std::vector<std::string>> pcdFiles;
    std::vector<std::vector<std::string>> csvFiles;
    std::vector<std::vector<std::vector<unsigned int>>> matchIdx;


    std::string strPcdNameLc;
    std::string strCsvNameLc;

    if (autoMatch){

        for (int i=0;  i<dirNumber; i++ )
        {


            int j=0;

            std::vector<std::string> pcdFile;
            std::vector<std::string> csvFile;
            if (loopClose){

                strPcdNameLc=autoMatchDir[i]+"/"+loopCloseKF+".pcd";
                strCsvNameLc=autoMatchDir[i]+"/"+loopCloseKF+".csv";

                pcdFile.push_back(strPcdNameLc);
                csvFile.push_back(strCsvNameLc);
            }

            while(true){

                std::string strPcdName=autoMatchDir[i]+"/"+autoNameTemplate+std::to_string(j)+".pcd";
                std::string strCsvName=autoMatchDir[i]+"/"+autoNameTemplate+std::to_string(j)+".csv";
                std::ifstream fCsv(strCsvName.c_str());
                std::ifstream fPcd(strPcdName.c_str());

                if (!fPcd.good()||!fCsv.good())
                {
                    if (loopClose){
                        pcdFile.push_back(strPcdNameLc);
                        csvFile.push_back(strCsvNameLc);
                    }
                    break;
                }
                //    std::cout<<"string is \n"<<strPcdName<<std::endl;

                pcdFile.push_back(strPcdName);
                csvFile.push_back(strCsvName);
                j++;

            }
            pcdFiles.push_back(pcdFile);
            csvFiles.push_back(csvFile);

        }




        for (unsigned int i=0;  i<(dirNumber-1); i++ ){

            unsigned int tempIdx=i;
            std::vector<std::vector<unsigned int>> matchIdx0;
            while (true){
                if (tempIdx+1>=dirNumber) break;
                matchIdx0.push_back({i, tempIdx+1});
                std::cout<<i<<","<<tempIdx+1<<std::endl;
                tempIdx++;
            }
            matchIdx.push_back(matchIdx0);
        }

    }

    std::string autoVerifyStr = config["autoVerify"].as<std::string>();
    bool autoVerify=false;

    if (autoVerifyStr=="True" || autoVerifyStr=="true")
    {
        autoVerify=true;
    }





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


    if (autoMatch){

        //  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBL>>> XYZRGBLvec;


        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> prevXYZRGBL;
        std::vector<std::string> prevTrajectories;
        std::vector<pcl::PointCloud<pcl::Normal>> prevNormals;

        for (int k=0; k<pcdFiles[0].size();k++){


            std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
            std::vector<std::string> newTrajectories;
            std::vector<pcl::PointCloud<pcl::Normal>> normals;

            for (int ii=0; ii<pcdFiles.size(); ii++)
            {
                newTrajectories.push_back(csvFiles[ii][k]);
                pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc2 (new pcl::PointCloud<pcl::PointXYZRGBL>);

                if (pcl::io::loadPCDFile<pcl::PointXYZRGBL> (pcdFiles[ii][k], *pc2) == -1) //* load the file
                {
                    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                    return (-1);
                }
                std::cout << "Loaded "
                          << pc2->width * pc2->height
                          << " data points from "<<pcdFiles[ii][k]<< std::endl;
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
            //----------------------------------------------------temporal matching---------------------------------------------------------------------------//
            spatialAlignment=false;
            for (int i=0; i<matchIdx.size(); i++)
            {
                std::vector<std::vector<unsigned int>> matchLogIdx;  // keyframe time period; keyframe timeperiod
                std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL1;
                std::vector<std::string> newTrajectories1;
                std::vector<pcl::PointCloud<pcl::Normal>> normals1;

                unsigned int k0=matchIdx[i][0][0];
                XYZRGBL1.push_back(XYZRGBL[k0]);
                normals1.push_back(normals[k0]);
                newTrajectories1.push_back(newTrajectories[k0]);
                matchLogIdx.push_back({k, i});
                for (unsigned int j=0; j<matchIdx[i].size(); j++)
                {

                    unsigned int k1=matchIdx[i][j][1];
                    matchLogIdx.push_back({k, k1});
                    XYZRGBL1.push_back(XYZRGBL[k1]);
                    newTrajectories1.push_back(newTrajectories[k1]);
                    normals1.push_back(normals[k1]);


                }

                std::string currentPath="";

                //std::cout<<i<<std::endl;
                int mapCounter=0;
                //std::cout<<pathOut<<std::endl;
                //  std::ostringstream ss; ss<<(i);
                currentPath=pathOut;
                std::cout<<"matching in progress"<<std::endl;
                dir=mkdir (currentPath.c_str(),S_IRWXU);
                IO* Io =new IO();
                Io ->readClouds(XYZRGBL1, normals1, newTrajectories1, matchLogIdx, spatialAlignment,autoMatch,  currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, false);

                delete Io;
            }
            //----------------------------------------------------spatial matching---------------------------------------------------------------------------//

            int mapCounter=0;
            if (k>0){
                spatialAlignment=true;
                for (unsigned int j=0; j<pcdFiles.size(); j++)
                {

                    std::vector<std::vector<unsigned int>> matchLogIdx;  // keyframe time period; keyframe timeperiod
                    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL1;
                    std::vector<std::string> newTrajectories1;
                    std::vector<pcl::PointCloud<pcl::Normal>> normals1;

                    matchLogIdx.push_back({k-1, j});
                    matchLogIdx.push_back({k, j});

                    XYZRGBL1.push_back(prevXYZRGBL[j]);
                    newTrajectories1.push_back(prevTrajectories[j]);
                    normals1.push_back(prevNormals[j]);

                    XYZRGBL1.push_back(XYZRGBL[j]);
                    newTrajectories1.push_back(newTrajectories[j]);
                    normals1.push_back(normals[j]);

                    std::string currentPath="";

                    //std::cout<<i<<std::endl;

                    //std::cout<<pathOut<<std::endl;
                    //  std::ostringstream ss; ss<<(i);
                    currentPath=pathOut;
                    //std::cout<<currentPath<<std::endl;
                    dir=mkdir (currentPath.c_str(),S_IRWXU);
                    IO* Io =new IO();
                    Io ->readClouds(XYZRGBL1, normals1, newTrajectories1, matchLogIdx, spatialAlignment,autoMatch,  currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic,loopClose);
                    mapCounter++;
                    delete Io;
                }




            }
            prevXYZRGBL=XYZRGBL;
            prevTrajectories=newTrajectories;
            prevNormals=normals;
        }
    }


    else {




        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
        std::vector<pcl::PointCloud<pcl::Normal>> normals;
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
        std::vector<std::vector<unsigned int>> matchLogIdx;
        Io ->readClouds(XYZRGBL, normals, sourceTrajectories, matchLogIdx, spatialAlignment, autoMatch, currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, false);

        delete Io;
    }
}
