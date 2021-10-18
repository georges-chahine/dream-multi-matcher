#include "IO/IO.h"
#include "IO/settings.h"
#include "Registration/icp.h"


namespace matcher
{
IO::IO()
{
}

IO::~IO()
{
}
typedef PointMatcher<float> PM;
typedef PM::TransformationParameters TP;
struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};
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


void IO::alignTrajectories(MatrixXd& newTrajectory, MatrixXd& oldTrajectory, TP& prior)
{
    for (int i=1; i<newTrajectory.rows(); i++)
    {
        newTrajectory(i,1)=newTrajectory(i,1)-newTrajectory(0,1);
        newTrajectory(i,2)=newTrajectory(i,2)-newTrajectory(0,2);
        newTrajectory(i,3)=newTrajectory(i,3)-newTrajectory(0,3);
    }

    // std::cout<<newTrajectory<<std::endl;
    for (int i=1; i<oldTrajectory.rows(); i++)
    {
        oldTrajectory(i,1)=oldTrajectory(i,1)-oldTrajectory(0,1);
        oldTrajectory(i,2)=oldTrajectory(i,2)-oldTrajectory(0,2);
        oldTrajectory(i,3)=oldTrajectory(i,3)-oldTrajectory(0,3);
    }

    newTrajectory(0,1)=newTrajectory(0,1)-newTrajectory(0,1);
    newTrajectory(0,2)=newTrajectory(0,2)-newTrajectory(0,2);
    newTrajectory(0,3)=newTrajectory(0,3)-newTrajectory(0,3);

    oldTrajectory(0,1)=oldTrajectory(0,1)-oldTrajectory(0,1);
    oldTrajectory(0,2)=oldTrajectory(0,2)-oldTrajectory(0,2);
    oldTrajectory(0,3)=oldTrajectory(0,3)-oldTrajectory(0,3);


    double lowestError=999999999999999999;
    double lowestTheta=0;
    std::cout<<"Aligning Trajectories..."<<std::endl;

    Vector3d newHalfDistPt(0,0,0);
    int newHalfDistIdx=0;
    MatrixXd newTrajectoryDistances=Eigen::MatrixXd::Constant(1, newTrajectory.rows(),0);
    MatrixXd newTrajectoryDistancesCumulative=Eigen::MatrixXd::Constant(1, newTrajectory.rows(),0);

    Vector3d oldHalfDistPt(0,0,0);
    int oldHalfDistIdx=0;
    MatrixXd oldTrajectoryDistances=Eigen::MatrixXd::Constant(1, oldTrajectory.rows(),0);
    MatrixXd oldTrajectoryDistancesCumulative=Eigen::MatrixXd::Constant(1, oldTrajectory.rows(),0);


    std::cout.precision(17);

    for (int i=1; i<newTrajectory.rows(); i++)
    {
        //std::cout<<"i is ----"<<i<<std::endl;
        //std::cout<<"i is "<<i<< "new trajectory is "<< newTrajectory(i,1)<<std::endl;


        double distance=sqrt(pow(newTrajectory(i-1,1)-newTrajectory(i,1),2)+pow(newTrajectory(i-1,2)-newTrajectory(i,2),2) );
        newTrajectoryDistances(0,i)=distance;
        newTrajectoryDistancesCumulative(0,i)=distance+newTrajectoryDistancesCumulative(0,i-1);
        //     std::cout<<"new tracjectory has "<<newTrajectory.rows()<<" rows and "<<newTrajectory.cols()<<" columns"<<std::endl;
    }

    for (int i=1; i<oldTrajectory.rows(); i++)
    {
        double distance=sqrt(pow(oldTrajectory(i-1,1)-oldTrajectory(i,1),2)+pow(oldTrajectory(i-1,2)-oldTrajectory(i,2),2) );
        oldTrajectoryDistances(0,i)=distance;
        oldTrajectoryDistancesCumulative(0,i)=distance+oldTrajectoryDistancesCumulative(0,i-1);
    }

    double newTh=newTrajectoryDistancesCumulative(0,newTrajectoryDistancesCumulative.cols()-1)/2;
    double oldTh=oldTrajectoryDistancesCumulative(0,oldTrajectoryDistancesCumulative.cols()-1)/2;

    for (int i=1; i<newTrajectory.rows(); i++)
    {
        if (newTrajectoryDistancesCumulative(0,i)>newTh){
            newHalfDistIdx=i;
            newHalfDistPt<< newTrajectory(i,1), newTrajectory(i,2), newTrajectory(i,3);
            std::cout<<"newHalfDistPt is "<<newHalfDistPt<< " new half dist is "<< newTh <<std::endl;
            break;
        }
    }


    for (int i=1; i<oldTrajectory.rows(); i++)
    {
        if (oldTrajectoryDistancesCumulative(0,i)>oldTh){
            oldHalfDistIdx=i;
            oldHalfDistPt<< oldTrajectory(i,1), oldTrajectory(i,2), oldTrajectory(i,3);
            std::cout<<"oldHalfDistPt is "<<oldHalfDistPt<< " old half dist is "<< oldTh <<std::endl;
            break;
        }
    }

    Vector3d translation(0,0,0);    //=oldHalfDistPt-newHalfDistPt;

    // for (int i=0; i<newTrajectory.rows(); i++)
    // {
    //    newTrajectory(i,1)=newTrajectory(i,1)+translation(0);
    //    newTrajectory(i,2)=newTrajectory(i,2)+translation(1);
    //    newTrajectory(i,3)=newTrajectory(i,3)+translation(2);

    // }

    //std::cout<<"new trajec is \n"<<newTrajectory<<std::endl;
    float theta=0;
    while (theta<360){

        Eigen::Affine3d prior2= Eigen::Affine3d::Identity();
        float thetaR=theta*3.14159265/180;
        prior2.matrix()(0,0)=cos(thetaR);
        prior2.matrix()(0,1)=-sin(thetaR);
        prior2.matrix()(0,2)=0;
        prior2.matrix()(0,3)=0;   //this works only because we are rotating only in Z and moving x y. Otherwise, should do in this case the translation before the rotation.

        prior2.matrix()(1,0)=sin(thetaR);
        prior2.matrix()(1,1)=cos(thetaR);
        prior2.matrix()(1,2)=0;
        prior2.matrix()(1,3)=0;

        prior2.matrix()(2,0)=0;
        prior2.matrix()(2,1)=0;
        prior2.matrix()(2,2)=1;
        prior2.matrix()(2,3)=0;

        prior2.matrix()(3,0)=0;
        prior2.matrix()(3,1)=0;
        prior2.matrix()(3,2)=0;
        prior2.matrix()(3,3)=1;



        MatrixXd transformedNewTrajectory=Eigen::MatrixXd::Constant(3, newTrajectory.rows(),1);


        for (int i=0; i<newTrajectory.rows(); i++)
        {

            Vector4d transformedPoint(newTrajectory(i,1),newTrajectory(i,2),newTrajectory(i,3),1);
            // transformedPoint(0)=transformedPoint(0)+translation(0);
            // transformedPoint(1)=transformedPoint(1)+translation(1);    //points at mid distances aligned

            transformedPoint=prior2.matrix()*transformedPoint;

            transformedNewTrajectory(0,i)=transformedPoint(0);
            transformedNewTrajectory(1,i)=transformedPoint(1);
            transformedNewTrajectory(2,i)=transformedPoint(2);
        }

        //prior2.matrix()(0,3)=oldTrajectory(0,1)-newTrajectory(0,1);
        // prior2.matrix()(1,3)=oldTrajectory(0,2)-newTrajectory(0,2);


        prior2.matrix()(0,3)=translation(0);
        prior2.matrix()(1,3)=translation(1);
        double lowestDistance;
        double error=0;

        // std::cout<<"new transformed trajec is \n"<<transformedNewTrajectory<<std::endl;
        for (int i=0; i<newTrajectory.rows(); i++){  //nearest neighbor
            lowestDistance=999999999999999;
            for (int j=0; j<oldTrajectory.rows(); j++)
            {
                double distance=sqrt(pow(transformedNewTrajectory(0,i)-oldTrajectory(j,1),2)+pow(transformedNewTrajectory(1,i)-oldTrajectory(j,2),2) );
                // std::cout<<"distance is "<<distance<<std::endl;
                if (distance<lowestDistance){

                    lowestDistance=distance;
                }

            }
            error=lowestDistance+error;
        }
        //std::cout<<"error is "<<error<<std::endl;
        if (error<lowestError)   //get theta that gives the lowest nearest neighbor error
        {
            lowestError=error;
            prior=prior2.matrix().cast<float>();
            lowestTheta=theta;


        }
        theta=theta+5;
        //   std::cout<<"theta is "<<theta<<std::endl;

    }
    std::cout<<"lowestTheta is "<<lowestTheta<<std::endl;
    std::cout<<"lowestError is "<<lowestError<<std::endl;


    //translation

    theta=lowestTheta;
    Eigen::Affine3d prior2= Eigen::Affine3d::Identity();
    float thetaR=theta*3.14159265/180;
    prior2.matrix()(0,0)=cos(thetaR);
    prior2.matrix()(0,1)=-sin(thetaR);
    prior2.matrix()(0,2)=0;
    prior2.matrix()(0,3)=0;   //this works only because we are rotating only in Z and moving x y. Otherwise, should do in this case the translation before the rotation.

    prior2.matrix()(1,0)=sin(thetaR);
    prior2.matrix()(1,1)=cos(thetaR);
    prior2.matrix()(1,2)=0;
    prior2.matrix()(1,3)=0;

    prior2.matrix()(2,0)=0;
    prior2.matrix()(2,1)=0;
    prior2.matrix()(2,2)=1;
    prior2.matrix()(2,3)=0;

    prior2.matrix()(3,0)=0;
    prior2.matrix()(3,1)=0;
    prior2.matrix()(3,2)=0;
    prior2.matrix()(3,3)=1;



    MatrixXd transformedNewTrajectory=Eigen::MatrixXd::Constant(3, newTrajectory.rows(),1);


    for (int i=0; i<newTrajectory.rows(); i++)
    {

        Vector4d transformedPoint(newTrajectory(i,1),newTrajectory(i,2),newTrajectory(i,3),1);
        // transformedPoint(0)=transformedPoint(0)+translation(0);
        // transformedPoint(1)=transformedPoint(1)+translation(1);    //points at mid distances aligned

        transformedPoint=prior2.matrix()*transformedPoint;

        transformedNewTrajectory(0,i)=transformedPoint(0);
        transformedNewTrajectory(1,i)=transformedPoint(1);
        transformedNewTrajectory(2,i)=transformedPoint(2);
    }
    MatrixXd transformedNewTrajectory2=Eigen::MatrixXd::Constant(3, newTrajectory.rows(),1);
    lowestError=9999999999999999;
    for (int x=-30; x<30; x=x+1){
        for (int y=-30; y<30; y=y+1){
            transformedNewTrajectory2=transformedNewTrajectory;
            for (int i=0; i<newTrajectory.rows(); i=i+1)
            {
                transformedNewTrajectory2(0,i)=transformedNewTrajectory2(0,i)+x;
                transformedNewTrajectory2(1,i)=transformedNewTrajectory2(1,i)+y;

            }
            double lowestDistance;
            double error=0;

            // std::cout<<"new transformed trajec is \n"<<transformedNewTrajectory<<std::endl;
            for (int i=0; i<newTrajectory.rows(); i++){  //nearest neighbor
                lowestDistance=999999999999999;
                for (int j=0; j<oldTrajectory.rows(); j++)
                {
                    double distance=sqrt(pow(transformedNewTrajectory2(0,i)-oldTrajectory(j,1),2)+pow(transformedNewTrajectory2(1,i)-oldTrajectory(j,2),2) );
                    // std::cout<<"distance is "<<distance<<std::endl;
                    if (distance<lowestDistance){
                        lowestDistance=distance;
                    }
                }
                error=lowestDistance+error;
            }
            //std::cout<<"error is "<<error<<std::endl;
            if (error<lowestError)   //get theta that gives the lowest nearest neighbor error
            {

                lowestError=error;
                prior(0,3)=x; prior(1,3)=y;
                // std::cout<<"error is "<<error<< " x is " <<x<<" y is "<<y<<std::endl;
            }
        }
    }



}

void IO::readClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL, std::vector<pcl::PointCloud<pcl::Normal>> normals, std::vector<std::string> sourceTrajectories, std::vector<std::vector<int>> matchLogIdx, bool spatialAlignment, bool autoMatch, std::string currentPath, std::string rMethod , int mapNumber, bool semantics, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, bool loopCloseFlag, Eigen::Matrix4d prior0)
{
    std::ofstream poseStream;
    poseStream.precision(16);
    std::string posePath= currentPath + "/transforms.csv" ;
    poseStream.open (posePath.c_str(), std::fstream::out | std::fstream::app);
    poseStream <<"%KF,time_period,x,y,z,qx,qy,qz,qw\n";
    if (autoMatch){
        std::cout<<"matching keyframes \n";
        for (unsigned int i=0; i<matchLogIdx.size(); i++){
            std::cout <<"KF"<<matchLogIdx[i][0]<<" from time period "<<matchLogIdx[i][1]<<std::endl;

        }
    }

    std::vector<std::string> filteredPath;
    //---------------------------------------------REGISTRATION-----------------------------------------------

    if (rMethod=="gps")
    {


        PM::TransformationParameters T_previous_alignment=PM::TransformationParameters::Identity(4,4);
        while (true)
        {

            std::string selection;
            std::cout<< "Map alignment:"<<std::endl;
            std::cout<<std::endl;

            std::cout<< "THIS IS SURVEY #"<<mapNumber<<std::endl;

            std::cout<<std::endl;
            std::cout<<"choose from: icp semantic_icp all exit"<<std::endl;
            std::cout<<std::endl;
            std::cout<<"REMARK: When finished, type exit to process next survey"<<std::endl;

            std::cout<<"Selection:"<<std::endl;
            //std::cin >> selection;

            if (autoMatch) {     //if (autoMatch)
                selection="all";
            }
            else{

                std::cin >> selection;
            }

            std::cout<<std::endl;

            if  (selection=="semantic_icp" || selection=="all" ){

                //   std::string filename0= + "aligned" + std::to_string(0);
                //    std::string fullPath1= currentPath + "/" + filename0 + ".pcd" ;
                //     std::string fullPath2= currentPath + "/" + filename0 + ".ply" ;
                //  std::string fullPath3= currentPath + "/" + filename0 + ".vtk" ;
                //    pcl::io::savePCDFileASCII (fullPath1, XYZRGBL[0]);
                //   pcl::io::savePLYFileASCII (fullPath2, XYZRGBL[0]);

                if (autoMatch){
                    poseStream<<matchLogIdx[0][0]<<","<<matchLogIdx[0][1]<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<1<<std::endl;
                }
                else{

                    poseStream<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<0<<","<<1<<std::endl;

                }
                std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBLTemp;
                std::vector<pcl::PointCloud<pcl::Normal>> normalsTemp;
                for (int i=1; i<XYZRGBL.size(); i++)
                {
                    MatrixXd newTrajectory ;
                    MatrixXd oldTrajectory ;
                    if (sourceTrajectories[0]!="SKIP"){
                        newTrajectory = load_csv<MatrixXd>(sourceTrajectories[i]);
                        oldTrajectory = load_csv<MatrixXd>(sourceTrajectories[0]);
                    }



                    //    std::cout<<xMaxNewTrajectory<<" "<<yMaxNewTrajectory<<" "<< xMaxOldTrajectory<<" "<<yMaxOldTrajectory<<" "<<xMinNewTrajectory<<" "<<yMinNewTrajectory<<" "<<xMinOldTrajectory<<" "<<yMinOldTrajectory<<std::endl;

                    TP prior = TP::Identity(4,4);
                    //TP prior = initialEstimate.matrix().cast<float>();

                    //   std::cout<<"T_previous_alignment is "<<T_previous_alignment<<std::endl;
                    std::string filename= "aligned_" + std::to_string(i);
                    //std::string filename= "aligned_" + std::to_string(matchLogIdx[i][0])+"_"+std::to_string(matchLogIdx[i][1]);
                    //prior2.matrix()(0,3)=oldTrajectory(0,1)-newTrajectory(0,1);
                    // prior2.matrix()(1,3)=oldTrajectory(0,2)-newTrajectory(0,2);


                    if (prior0.isIdentity(0.0001) && sourceTrajectories[0]!="SKIP"){


                        if ( spatialAlignment && !(loopCloseFlag && mapNumber==0) ){ //spatialAlignment
                            double distance=9999999;
                            double timeMatch=newTrajectory(0,0);
                            unsigned int spatialIndex;

                            double smallestTime=999;
                            for (int kk=0; kk<oldTrajectory.rows(); kk++)
                            {
                                double dt=oldTrajectory(kk,0)-timeMatch;
                                //std::cout<< "abs(dt) is" << abs(dt) <<std::endl;
                                if (abs(dt)<smallestTime)
                                {
                                    smallestTime=abs(dt);
                                    spatialIndex=kk;
                                }
                            }
                            std::cout<<"spatial index is "<<spatialIndex<<std::endl;
                            float xIncr=oldTrajectory(spatialIndex,1)-oldTrajectory(0,1);
                            float yIncr=oldTrajectory(spatialIndex,2)-oldTrajectory(0,2);
                            float zIncr=oldTrajectory(spatialIndex,3)-oldTrajectory(0,3);



                            std::cout<<"spacial alignment active"<<std::endl;
                            prior(0,3)=xIncr;prior(1,3)=yIncr;prior(2,3)=zIncr;

                        }





                        else{
                            //    if (loopCloseFlag&& mapNumber==0){
                            //        prior(0,3)=xIncrLc;prior(1,3)=yIncrLc;

                            //    }
                            //    else
                            //     {
                            alignTrajectories(newTrajectory,oldTrajectory, prior);
                            //  }
                        }
                    }


                    else
                    {

                        prior=prior0.cast<float>();

                    }
                    std::cout<<"prior is \n"<<prior.matrix()<<std::endl;
                    T_previous_alignment=PM::TransformationParameters::Identity(4,4);  //comment out this line if matching with XYZRGBL[i-1] instead of XYZRGBL[0]
                    int icpCounter=0;
                    while (T_previous_alignment.isIdentity()){
                        std::cout<<"ICP attempt #"<<icpCounter<<std::endl;
                        if (icpCounter==3){break;}
                        icpCounter++;

                        ICP* Icp =new ICP();

                        Icp->alignMaps(XYZRGBL[0],normals[0], XYZRGBL[i],normals[i], prior, spatialAlignment, currentPath, filename, leafSize,icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, semantics, autoMatch, T_previous_alignment );
                        delete Icp;
                    }
                    Eigen::Matrix3f poseRot=T_previous_alignment.block(0,0,3,3);
                    Eigen::Quaternionf q(poseRot);
                    if (autoMatch){
                        poseStream<<matchLogIdx[i][0]<<","<<matchLogIdx[i][1]<<","<<T_previous_alignment(0,3)<<","<<T_previous_alignment(1,3)<<","<<T_previous_alignment(2,3)<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<std::endl;
                    }
                    else{
                        poseStream<<0<<","<<0<<","<<T_previous_alignment(0,3)<<","<<T_previous_alignment(1,3)<<","<<T_previous_alignment(2,3)<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<std::endl;
                    }

                    
                }

            }

            if  (selection=="exit" || selection=="all" ){break;}
        }
    }
    poseStream.close();
}
}
