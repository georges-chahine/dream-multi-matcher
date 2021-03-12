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
using namespace Eigen;
template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}


void loopClosureFilter(std::vector<std::string> trajectories, std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> &XYZRGBL, std::vector<pcl::PointCloud<pcl::Normal>> &normals, Eigen::Matrix4d &prior){

    double xIncrLc, yIncrLc;
    MatrixXd newTrajectory = load_csv<MatrixXd>(trajectories[1]);
    MatrixXd oldTrajectory = load_csv<MatrixXd>(trajectories[0]);

    double xMaxNewTrajectory=newTrajectory(0,1);
    double yMaxNewTrajectory=newTrajectory(0,2);
    double xMaxOldTrajectory=oldTrajectory(0,1);
    double yMaxOldTrajectory=oldTrajectory(0,2);

    double xMinNewTrajectory=newTrajectory(0,1);
    double yMinNewTrajectory=newTrajectory(0,2);
    double xMinOldTrajectory=oldTrajectory(0,1);
    double yMinOldTrajectory=oldTrajectory(0,2);

    for (int j=1; j<newTrajectory.rows(); j++){

        if (xMaxNewTrajectory<newTrajectory(j,1)){
            xMaxNewTrajectory=newTrajectory(j,1);
        }

        if (yMaxNewTrajectory<newTrajectory(j,2)){
            yMaxNewTrajectory=newTrajectory(j,2);
        }

        if (xMinNewTrajectory>newTrajectory(j,1)){
            xMinNewTrajectory=newTrajectory(j,1);
        }


        if (yMinNewTrajectory>newTrajectory(j,2)){
            yMinNewTrajectory=newTrajectory(j,2);
        }

    }
    double oldXMean=oldTrajectory(0,1);
    double oldYMean=oldTrajectory(0,2);

    for (int j=1; j<oldTrajectory.rows(); j++){

        if (xMaxOldTrajectory<oldTrajectory(j,1)){
            xMaxOldTrajectory=oldTrajectory(j,1);
        }


        if (yMaxOldTrajectory<oldTrajectory(j,2)){
            yMaxOldTrajectory=oldTrajectory(j,2);
        }

        if (xMinOldTrajectory>oldTrajectory(j,1)){
            xMinOldTrajectory=oldTrajectory(j,1);
        }


        if (yMinOldTrajectory>oldTrajectory(j,2)){
            yMinOldTrajectory=oldTrajectory(j,2);
        }
        oldXMean=oldTrajectory(j,1)+oldXMean;
        oldYMean=oldTrajectory(j,2)+oldYMean;


    }
    oldXMean=oldXMean/oldTrajectory.rows()-oldTrajectory(0,1);
    oldYMean=oldYMean/oldTrajectory.rows()-oldTrajectory(0,2);
    float xIncr=oldXMean;
    float yIncr=oldYMean;

    xIncrLc=xIncr;yIncrLc=yIncr;
    double xDiff=xMaxOldTrajectory-xMinOldTrajectory;
    double yDiff=yMaxOldTrajectory-yMinOldTrajectory;


    int trimTolerance=20;
    unsigned int k=0;
    pcl::PointCloud<pcl::PointXYZRGBL> pcTemp;
    pcl::PointCloud<pcl::Normal> pcNrmlTmp;
    std::cout<<"start here"<<std::endl;
    while (true){     //lc
        if (k>=XYZRGBL[0].points.size()){break;}


        double x=XYZRGBL[0].points[k].x+oldTrajectory(0,1); double y=XYZRGBL[0].points[k].y+oldTrajectory(0,2);


        if (xDiff>yDiff){


            if ( (x+trimTolerance) <xMinOldTrajectory || (x+trimTolerance) <xMinNewTrajectory ||  (x-trimTolerance) > xMaxNewTrajectory || (x-trimTolerance) > xMaxOldTrajectory)
            {
                //                                std::cout<<"filtered!"<<std::endl;
                //       XYZRGBL[0].points.erase(XYZRGBL[0].points.begin() + k );
                //  normals[0].erase(normals[0].begin() + k );

            }
            else

            {
                pcl::PointXYZRGBL tmpPt= XYZRGBL[0].points[k];
                pcl::Normal tmpNrml=normals[0][k];
                pcTemp.push_back(tmpPt);
                pcNrmlTmp.push_back(tmpNrml);
            }

        }

        else
        {

            if ( (y+trimTolerance) <yMinOldTrajectory || (y+trimTolerance) <yMinNewTrajectory ||  (y-trimTolerance) > yMaxNewTrajectory || (y-trimTolerance) > yMaxOldTrajectory )
            {
                //                                std::cout<<"filtered!"<<std::endl;
                //       XYZRGBL[0].points.erase(XYZRGBL[0].points.begin() + k );
                //  normals[0].erase(normals[0].begin() + k );

            }
            else

            {
                pcl::PointXYZRGBL tmpPt= XYZRGBL[0].points[k];
                pcl::Normal tmpNrml=normals[0][k];
                pcTemp.push_back(tmpPt);
                pcNrmlTmp.push_back(tmpNrml);
            }

        }

        k++;
    }

    XYZRGBL[0]=pcTemp;
    normals[0]=pcNrmlTmp;

    pcl::PointCloud<pcl::PointXYZRGBL> pcTemp1;
    pcl::PointCloud<pcl::Normal> pcNrmlTmp1;
    std::cout<<"end here"<<std::endl;
    k=0;
    while (true){     //lc
        if (k>=XYZRGBL[1].points.size()){break;}
        double x=XYZRGBL[1].points[k].x+newTrajectory(0,1); double y=XYZRGBL[1].points[k].y+newTrajectory(0,2);


        if (xDiff>yDiff){


            if ( (x+trimTolerance) <xMinOldTrajectory || (x+trimTolerance) <xMinNewTrajectory ||  (x-trimTolerance) > xMaxNewTrajectory || (x-trimTolerance) > xMaxOldTrajectory)
            {
                //                                std::cout<<"filtered!"<<std::endl;
                //       XYZRGBL[0].points.erase(XYZRGBL[0].points.begin() + k );
                //  normals[0].erase(normals[0].begin() + k );

            }
            else

            {
                pcl::PointXYZRGBL tmpPt1= XYZRGBL[1].points[k];
                pcl::Normal tmpNrml1=normals[1][k];
                pcTemp1.push_back(tmpPt1);
                pcNrmlTmp1.push_back(tmpNrml1);
            }


        }
        else
        {
            if ( (y+trimTolerance) <yMinOldTrajectory || (y+trimTolerance) <yMinNewTrajectory ||  (y-trimTolerance) > yMaxNewTrajectory || (y-trimTolerance) > yMaxOldTrajectory )
            {
                //                                std::cout<<"filtered!"<<std::endl;
                //       XYZRGBL[0].points.erase(XYZRGBL[0].points.begin() + k );
                //  normals[0].erase(normals[0].begin() + k );

            }
            else

            {
                pcl::PointXYZRGBL tmpPt1= XYZRGBL[1].points[k];
                pcl::Normal tmpNrml1=normals[1][k];
                pcTemp1.push_back(tmpPt1);
                pcNrmlTmp1.push_back(tmpNrml1);
            }

        }

        k++;
    }

    XYZRGBL[1]=pcTemp1;
    normals[1]=pcNrmlTmp1;

    prior(0,3)=xIncrLc;prior(1,3)=yIncrLc;
}


void loadCloud(std::string pcdPath, std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> &XYZRGBL,  std::vector<pcl::PointCloud<pcl::Normal>> &normals, double leafSize){




    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc2 (new pcl::PointCloud<pcl::PointXYZRGBL>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBL> (pcdPath, *pc2) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        //return (-1);
    }
    std::cout << "Loaded "
              << pc2->width * pc2->height
              << " data points from "<<pcdPath<< std::endl;
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
    std::string icpConfigFilePathLoopClosure = config["icpConfigLoopClosure"].as<std::string>();
    std::string inputFiltersConfigFilePath = config["icpInputFilters"].as<std::string>();
    std::string mapPostFiltersConfigFilePath = config["icpPostFilters"].as<std::string>();
    std::string	rtkTopic="";
    std::vector<std::string> sourceClouds = config["sourceClouds"].as<std::vector<std::string>>();
    std::vector<std::string> sourceTrajectories = config["sourceTrajectories"].as<std::vector<std::string>>();
    std::vector<std::string> autoMatchDir = config["autoMatchDir"].as<std::vector<std::string>>();
    std::string autoMatchStr = config["autoMatch"].as<std::string>();
    bool autoMatch=false;
    Eigen::Matrix4d prior=Eigen::Matrix4d::Identity();
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




    if (autoMatch){

        for (int i=0;  i<dirNumber; i++ )
        {


            int j=0;

            std::vector<std::string> pcdFile;
            std::vector<std::string> csvFile;
            //  if (loopClose){


            //  pcdFile.push_back(strPcdNameLc);
            //   csvFile.push_back(strCsvNameLc);
            //  }

            while(true){

                std::string strPcdName=autoMatchDir[i]+"/"+autoNameTemplate+std::to_string(j)+".pcd";
                std::string strCsvName=autoMatchDir[i]+"/"+autoNameTemplate+std::to_string(j)+".csv";
                std::ifstream fCsv(strCsvName.c_str());
                std::ifstream fPcd(strPcdName.c_str());

                if (!fPcd.good()||!fCsv.good())
                {
                    /*             if (loopClose){
                   //     pcdFile.push_back(strPcdNameLc);
                  //      csvFile.push_back(strCsvNameLc);
                        pcdFile.insert(pcdFile.begin() ,pcdFile.back() );
                        csvFile.insert(csvFile.begin() ,csvFile.back() );
                    }
*/
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

    /*  std::string autoVerifyStr = config["autoVerify"].as<std::string>();
    bool autoVerify=false;

    if (autoVerifyStr=="True" || autoVerifyStr=="true")
    {
        autoVerify=true;
    }
*/




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
                loadCloud(pcdFiles[ii][k],XYZRGBL,normals, leafSize) ;


            }
            
            //----------------------------------------------------temporal matching---------------------------------------------------------------------------//
            spatialAlignment=false;
            for (int i=0; i<matchIdx.size(); i++)
            {
                std::vector<std::vector<int>> matchLogIdx;  // keyframe time period; keyframe timeperiod
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

                    int k1=matchIdx[i][j][1];
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
                Io ->readClouds(XYZRGBL1, normals1, newTrajectories1, matchLogIdx, spatialAlignment,autoMatch,  currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, false, prior);

                delete Io;
            }

            

            //----------------------------------------------------spatial matching---------------------------------------------------------------------------//
            
            int mapCounter=0;
            if (k>0){
                spatialAlignment=true;
                for (unsigned int j=0; j<pcdFiles.size(); j++)
                {

                    std::vector<std::vector<int>> matchLogIdx;  // keyframe time period; keyframe timeperiod
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
                    Io ->readClouds(XYZRGBL1, normals1, newTrajectories1, matchLogIdx, spatialAlignment,autoMatch,  currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic,false, prior);
                    mapCounter++;
                    delete Io;
                }




            }
            prevXYZRGBL=XYZRGBL;
            prevTrajectories=newTrajectories;
            prevNormals=normals;
            
        }
        /* ////////////////////LOOP CLOSURE////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        if (loopClose){
            for (int i=0;  i<dirNumber; i++ )  //alignment of loop closure KF with keyframe 0
            {
                int mapCounter=0;

                std::string strPcdNameLc;
                std::string strCsvNameLc;
                strPcdNameLc=autoMatchDir[i]+"/"+loopCloseKF+".pcd";
                strCsvNameLc=autoMatchDir[i]+"/"+loopCloseKF+".csv";

                std::ifstream fCsv(strCsvNameLc.c_str());
                std::ifstream fPcd(strPcdNameLc.c_str());

                if (!fPcd.good()||!fCsv.good())
                {
                    continue;
                }
                std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
                std::vector<std::string> trajectories;
                std::vector<pcl::PointCloud<pcl::Normal>> normals;

                loadCloud(strPcdNameLc,XYZRGBL, normals, leafSize);
                loadCloud(pcdFiles[i][0],XYZRGBL, normals, leafSize);


                trajectories.push_back(strCsvNameLc);
                trajectories.push_back(csvFiles[i][0]);

                loopClosureFilter(trajectories, XYZRGBL, normals, prior);

                std::vector<std::vector<int>> matchLogIdx;  // keyframe time period; keyframe timeperiod
                matchLogIdx.push_back({-1, i});
                matchLogIdx.push_back({0, i});
                std::string currentPath=pathOut;
                dir=mkdir (currentPath.c_str(),S_IRWXU);
                IO* Io =new IO();
                Io ->readClouds(XYZRGBL, normals, trajectories, matchLogIdx, spatialAlignment,autoMatch, currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePathLoopClosure, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic,true, prior);
                mapCounter++;
                delete Io;
                prior=Eigen::Matrix4d::Identity();


            }


            for (unsigned int i=0;  i<dirNumber; i++ )  //alignment of loop closure KF with keyframe x at the end
            {
                bool success=false;
                int mapCounter=0;

                std::string strPcdNameLc;
                std::string strCsvNameLc;
                strPcdNameLc=autoMatchDir[i]+"/"+loopCloseKF+".pcd";
                strCsvNameLc=autoMatchDir[i]+"/"+loopCloseKF+".csv";

                std::ifstream fCsv(strCsvNameLc.c_str());
                std::ifstream fPcd(strPcdNameLc.c_str());
                unsigned int x;
                double xClose, yClose, zClose, xClose0, yClose0, zClose0;

                if (!fPcd.good()||!fCsv.good())
                {
                    continue;
                }

                MatrixXd oldTrajectory = load_csv<MatrixXd>(strCsvNameLc);

                double refTime=oldTrajectory(0,0);

                for (unsigned int j=0; j<csvFiles[i].size(); j++){



                    MatrixXd newTrajectory = load_csv<MatrixXd>(csvFiles[i][j]);


                    for (unsigned int k=0; k<newTrajectory.rows();k++){


                        if (newTrajectory(k,0)==refTime){

                            x=j;
                            xClose=newTrajectory(k,1);
                            yClose=newTrajectory(k,2);
                            zClose=newTrajectory(k,3);
                            xClose0=newTrajectory(0,1);
                            yClose0=newTrajectory(0,2);
                            zClose0=newTrajectory(0,3);
                            success=true;

                        }


                    }

                }
                if (!success){
                    std::cout<<"couldn't find match for loop closure keyframe"<<std::endl;
                    continue;
                }

                prior(0,3)= -(xClose-xClose0);
                prior(1,3)= -(yClose-yClose0);
                prior(2,3)= -(zClose-zClose0);

                std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
                std::vector<std::string> trajectories;
                std::vector<pcl::PointCloud<pcl::Normal>> normals;


                loadCloud(strPcdNameLc,XYZRGBL, normals, leafSize);
                loadCloud(pcdFiles[i][x],XYZRGBL, normals, leafSize);

                trajectories.push_back(strCsvNameLc);
                trajectories.push_back(csvFiles[i][x]);
                std::vector<std::vector<int>> matchLogIdx;  // keyframe time period; keyframe timeperiod

                matchLogIdx.push_back({-1, i});
                matchLogIdx.push_back({x, i});
                std::string currentPath=pathOut;
                dir=mkdir (currentPath.c_str(),S_IRWXU);
                IO* Io =new IO();
                Io ->readClouds(XYZRGBL, normals, trajectories, matchLogIdx, spatialAlignment,autoMatch, currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePathLoopClosure, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic,false, prior);
                mapCounter++;
                delete Io;
                prior=Eigen::Matrix4d::Identity();

            }



        }


    }


    else {

        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
        std::vector<pcl::PointCloud<pcl::Normal>> normals;
        for (int i=0; i<mapNumber; i++)
        {
            loadCloud(sourceClouds[i],XYZRGBL,normals, leafSize) ;

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
        std::vector<std::vector<int>> matchLogIdx;
        Io ->readClouds(XYZRGBL, normals, sourceTrajectories, matchLogIdx, spatialAlignment, autoMatch, currentPath, rMethod, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, false, prior);

        delete Io;
    }
}
