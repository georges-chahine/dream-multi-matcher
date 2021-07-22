#include "IO/IO.h"
#include "IO/settings.h"
#include "Registration/icp.h"
#include "yaml-cpp/yaml.h"
namespace matcher
{
	using namespace YAML;

ICP::ICP()
{

}

ICP::~ICP()
{

}

void ICP::alignMaps(pcl::PointCloud<pcl::PointXYZRGBL> pcl_ref,  pcl::PointCloud<pcl::Normal> ref_normals, pcl::PointCloud<pcl::PointXYZRGBL> pcl_new, pcl::PointCloud<pcl::Normal> new_normals, PM::TransformationParameters prior, bool spacialAlignment, std::string currentPath, std::string filename, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, bool semantics, bool autoMatch, PM::TransformationParameters &T_previous_alignment )
{

    // ---------------------------------------------------------------------------------------------------------------//
    typedef PointMatcher<float> PM;
    //typedef PointMatcherIO<float> PMIO;
    typedef PM::TransformationParameters TP;
    typedef PM::DataPoints DP;
    float priorDynamic;
    using namespace PointMatcherSupport;
    //string outputFileName(argv[0]);

    // Rigid transformation
    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    // Create filters manually to clean the global map
    std::shared_ptr<PM::DataPointsFilter> densityFilter =
            PM::get().DataPointsFilterRegistrar.create(
                "SurfaceNormalDataPointsFilter",
    {
                    {"knn", "10"},
                    {"epsilon", "5"},
                    {"keepNormals", "0"},
                    {"keepDensities", "1"}
                }
                );

    std::shared_ptr<PM::DataPointsFilter> maxDensitySubsample =
            PM::get().DataPointsFilterRegistrar.create(
                "MaxDensityDataPointsFilter",
    {{"maxDensity", toParam(30)}}
                );
    // Main algorithm definition
    PM::ICP icp;


    PM::DataPointsFilters inputFilters;
    PM::DataPointsFilters mapPostFilters;
    if(!icpConfigFilePath.empty())
    {
        std::ifstream ifs(icpConfigFilePath.c_str());
        icp.loadFromYaml(ifs);
        std::cout<<"loaded icp yaml!"<<std::endl;
        ifs.close();
    }
    else
    {
        icp.setDefault();
    }

    if(!inputFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(inputFiltersConfigFilePath.c_str());
        inputFilters = PM::DataPointsFilters(ifs);
        std::cout<<"loaded input filter yaml!"<<std::endl;
        ifs.close();
    }

    if(!mapPostFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(mapPostFiltersConfigFilePath.c_str());
        mapPostFilters = PM::DataPointsFilters(ifs);
        std::cout<<"loaded post filter yaml!"<<std::endl;
        ifs.close();
    }

    // load YAML config
    //ifstream ifs(icpyml);
    //validateFile(argv[1]);
    // icp.loadFromYaml(ifs);

    //PMIO::FileInfoVector list(argv[2]);

    //PM::DataPoints;
    TP T_to_map_from_new = TP::Identity(4,4); // assumes 3D

    //-----------------------------------------------------------------------------/////////////////////////////////---------------------------/


    // IO::pCloudcontainer pcContainer2=pcContainer;
    std::cout<<"ICP Map Registration...."<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    // std::cout<<"Preaparing voxel grid with leafSize "<<leafSize<<" m"<<std::endl;
    // Eigen::Affine3d transform0= Eigen::Affine3d::Identity();
    // Eigen::Affine3d transformPrev= Eigen::Affine3d::Identity();
    // Eigen::Affine3d initialEstimate= Eigen::Affine3d::Identity();
    // Eigen::Affine3d transformRot0= Eigen::Affine3d::Identity();
    // Eigen::Affine3d transformPrevRot= Eigen::Affine3d::Identity();

    Eigen::Affine3d transformICP= Eigen::Affine3d::Identity();

    //  for (unsigned int i=0; i<pcContainer.timestamp.size();i++){



    //pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    //*tempCloud=pcl_new;


    //pcl_new=*tempCloud;
    DP data, mapPointCloud, newCloud;
    DP ref;

    //std::cout<<" new_normals.getMatrixXfMap() " << std::endl;
    //std::cout<<new_normals.getMatrixXfMap(3,8,0).row(0)<<std::endl;
    Eigen::MatrixXf dataNormals(3,new_normals.getMatrixXfMap(3,8,0).row(0).size());

    dataNormals.row(0)=new_normals.getMatrixXfMap(3,8,0).row(0);
    dataNormals.row(1)=new_normals.getMatrixXfMap(3,8,0).row(1);
    dataNormals.row(2)=new_normals.getMatrixXfMap(3,8,0).row(2);

    /*   for (int i=0; i<dataNormals.row(0).size(); i++){

        dataNormals(0,i)=(std::round(dataNormals(0,i)*1000.0))/1000.0;
        dataNormals(1,i)=(std::round(dataNormals(1,i)*1000.0))/1000.0;
        dataNormals(2,i)=(std::round(dataNormals(2,i)*1000.0))/1000.0;



    }
    */
    Eigen::MatrixXf datax(1,pcl_new.getMatrixXfMap(3,8,0).row(0).size());
    Eigen::MatrixXf datay(1,pcl_new.getMatrixXfMap(3,8,0).row(1).size());
    Eigen::MatrixXf dataz(1,pcl_new.getMatrixXfMap(3,8,0).row(2).size());
    datax=pcl_new.getMatrixXfMap(3,8,0).row(0);
    datay=pcl_new.getMatrixXfMap(3,8,0).row(1);
    dataz=pcl_new.getMatrixXfMap(3,8,0).row(2);


    //   for (int i=0; i<datax.row(0).size(); i++){

    //      datax(0,i)=(std::round(datax(0,i)*1000.0))/1000.0;
    //     datay(0,i)=(std::round(datay(0,i)*1000.0))/1000.0;
    //    dataz(0,i)=(std::round(dataz(0,i)*1000.0))/1000.0;
    //   std::cout<<datax(0,i)<<" "<<datay(0,i)<<" "<<dataz(0,i)<<" "<<std::endl;



    //    }

    //double x0=pcl_new.points[1500].x; double y0=pcl_new.points[1500].y; double z0=pcl_new.points[1500].z;
    std::cout.precision(17);
    // std::cout<<x0<<" "<<y0<<" "<<z0<<" "<<std::endl;
    //std::cout<<datax(0,1500)<<" "<<datay(0,1500)<<" "<<dataz(0,1500)<<" "<<std::endl;
    //datax.array()=datax.array()-datax(0,0);
    //datay.array()=datay.array()-datay(0,0);
    //dataz.array()=dataz.array()-dataz(0,0);

    data.addFeature("x", datax);
    data.addFeature("y", datay);
    data.addFeature("z", dataz);
    //  data.save("data.pcd");
    data.addDescriptor("normals", dataNormals);


    if (semantics)
    {
        Eigen::MatrixXf dataSemantics(1,new_normals.getMatrixXfMap(3,8,0).row(0).size());
        Eigen::MatrixXf semanticWeightsFull(1,new_normals.getMatrixXfMap(3,8,0).row(0).size());
        for (unsigned int j=0; j<pcl_new.points.size(); j++){
            dataSemantics(0,j)=(float)pcl_new.points[j].label;
            semanticWeightsFull(0,j)=semanticWeights[pcl_new.points[j].label];
        }
        data.addDescriptor("semantics", dataSemantics);
        data.addDescriptor("semanticWeights",semanticWeightsFull);


        Eigen::MatrixXf refSemantics(1,ref_normals.getMatrixXfMap(3,8,0).row(0).size());
        Eigen::MatrixXf semanticWeightsFull1(1,ref_normals.getMatrixXfMap(3,8,0).row(0).size());
        for (unsigned int j=0; j<pcl_ref.points.size(); j++){
            refSemantics(0,j)=(float)pcl_ref.points[j].label;
            semanticWeightsFull1(0,j)=semanticWeights[pcl_ref.points[j].label];
        }
        ref.addDescriptor("semantics", refSemantics);
        ref.addDescriptor("semanticWeights",semanticWeightsFull1);
    }
    else
    {
        Eigen::MatrixXf dataSemantics, semanticWeightsFull;
        dataSemantics=Eigen::MatrixXf::Constant (1,new_normals.getMatrixXfMap(3,8,0).row(0).size(),1);
        semanticWeightsFull=Eigen::MatrixXf::Constant (1,new_normals.getMatrixXfMap(3,8,0).row(0).size(),1);
        data.addDescriptor("semantics", dataSemantics);
        data.addDescriptor("semanticWeights",semanticWeightsFull);

        Eigen::MatrixXf refSemantics, semanticWeightsFull1;
        refSemantics=Eigen::MatrixXf::Constant (1,ref_normals.getMatrixXfMap(3,8,0).row(0).size(),1);
        semanticWeightsFull1=Eigen::MatrixXf::Constant (1,ref_normals.getMatrixXfMap(3,8,0).row(0).size(),1);
        ref.addDescriptor("semantics", refSemantics);
        ref.addDescriptor("semanticWeights",semanticWeightsFull1);


    }

    Eigen::MatrixXf refNormals(3,ref_normals.getMatrixXfMap(3,8,0).row(0).size());

    refNormals.row(0)=ref_normals.getMatrixXfMap(3,8,0).row(0);
    refNormals.row(1)=ref_normals.getMatrixXfMap(3,8,0).row(1);
    refNormals.row(2)=ref_normals.getMatrixXfMap(3,8,0).row(2);

    Eigen::MatrixXf refx(1,pcl_ref.getMatrixXfMap(3,8,0).row(0).size());
    Eigen::MatrixXf refy(1,pcl_ref.getMatrixXfMap(3,8,0).row(1).size());
    Eigen::MatrixXf refz(1,pcl_ref.getMatrixXfMap(3,8,0).row(2).size());
    refx=pcl_ref.getMatrixXfMap(3,8,0).row(0);
    refy=pcl_ref.getMatrixXfMap(3,8,0).row(1);
    refz=pcl_ref.getMatrixXfMap(3,8,0).row(2);
    //refx.array()=refx.array()-refx(0,0);
    //refy.array()=refy.array()-refy(0,0);
    //refz.array()=refz.array()-refz(0,0);
    ref.addFeature("x", refx);
    ref.addFeature("y", refy);
    ref.addFeature("z", refz);
    ref.addDescriptor("normals", refNormals);

    std::cout<<"---------------------------"<<std::endl;

    // initialEstimate=transformPrev.inverse()*transform;

    newCloud=data;
    mapPointCloud = ref;
    //newCloud.save("newCloud.vtk");
    //ref.save("ref.vtk");

    //newCloud = rigidTrans->compute(newCloud, T_previous_alignment);
    std::cout<<"T_previous_alignment is: "<<T_previous_alignment<<std::endl;

    if(computeProbDynamic)
    {
        newCloud.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newCloud.features.cols(), priorDynamic));
        mapPointCloud.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, mapPointCloud.features.cols(), priorDynamic));
    }

    inputFilters.apply(newCloud);
    inputFilters.apply(mapPointCloud);
    
    //Parser parser ("conf.yaml");
   // YAML_PM::Node doc;
    //parser.GetNextDocument(doc);



    Node config2 = LoadFile(icpConfigFilePath.c_str());
    Node config3 = config2["matcher"];
    Node config4 = config3["KDTreeMatcher"];

    auto nbPts0 = config4["knn"].as<std::string>();
    
    
    int icpNbPts=std::stoi(nbPts0);
   // std::cout<<"number of required points is "<<nbPts<<" points"<<std::endl;

    if (newCloud.getNbPoints()<icpNbPts || mapPointCloud.getNbPoints()<icpNbPts)
    
    {
	std::cout<<"insufficient nb of points, relaxing input filters..."<<std::endl;
	newCloud=data;
	mapPointCloud=ref;
	mapPostFilters.apply(newCloud);
	mapPostFilters.apply(mapPointCloud);
    }

    //std::cout<<"point count is "<<newCloud.getNbPoints()<<std::endl;

    //mapPointCloud=ref;
    //if(mapPointCloud.getNbPoints()  == 0)
    // {

    //     continue;
    // }

    // call ICP
    try
    {
        // We use the last transformation as a prior
        // this assumes that the point clouds were recorded in
        // sequence.
        //const TP prior = TP::Identity(4,4); //T_to_map_from_new*initialEstimate.matrix().cast<float>();



        // maps are already in the same global frame
        if (false)//if (spacialAlignment)
        {
            T_to_map_from_new=prior;

        }

        else
        {
            T_to_map_from_new = icp(newCloud, mapPointCloud, prior);

        }

    }
    catch (PM::ConvergenceError& error)
    {
        std::cout << "ERROR PM::ICP failed to converge: " << std::endl;
        std::cout << "   " << error.what() << std::endl;

    }

    for (int n=0; n<1; n++){


        try
        {
            //  std::cout<<"this is how it looks "<<T_to_map_from_new <<std::endl;
            if (! T_to_map_from_new.isIdentity()  ){
                break;
            }
            else{std::cout<< "condition did not work" <<std::endl;}
            T_to_map_from_new = icp(newCloud, mapPointCloud, prior);
        }
        catch (PM::ConvergenceError& error)
        {
            std::cout << "ERROR PM::ICP failed to converge: " << std::endl;
            std::cout << "   " << error.what() << std::endl;

        }
    }




    // This is not necessary in this example, but could be
    // useful if the same matrix is composed in the loop.
    //std::cout<<"before correction "<<std::endl;
    //std::cout<<T_to_map_from_new<<std::endl;
    T_to_map_from_new = rigidTrans->correctParameters(T_to_map_from_new);
    // std::cout<<"after correction "<<std::endl;
    // std::cout<<T_to_map_from_new<<std::endl;

    // Move the new point cloud in the map reference
    newCloud = rigidTrans->compute(newCloud, T_to_map_from_new);

    // Merge point clouds to map

    // mapPointCloud.concatenate(newCloud);
    //mapPostFilters.apply(newCloud);
    //mapPostFilters.apply(mapPointCloud);

    mapPointCloud = densityFilter->filter(mapPointCloud);
    mapPointCloud = maxDensitySubsample->filter(mapPointCloud);

    newCloud = densityFilter->filter(newCloud);
    newCloud = maxDensitySubsample->filter(newCloud);

    PM::TransformationParameters T=T_to_map_from_new;
    // PM::TransformationParameters T=initialEstimate.matrix().cast<float>();
    std::cout<<"correction is \n"<<T_to_map_from_new<<std::endl;

    transformICP.matrix()=T_previous_alignment.cast <double> () * T.cast <double> ();

    for (unsigned int j=0; j<pcl_new.points.size(); j++){

        double x=pcl_new.points[j].x; double y=pcl_new.points[j].y; double z=pcl_new.points[j].z;
        Eigen::Vector4d pcPoints(x,y,z,1.0);

        Eigen::Vector4d pcPointsTransformed=transformICP.matrix()*pcPoints;


        pcl::PointXYZRGBL tempPoint;
        tempPoint.x=pcPointsTransformed[0];
        tempPoint.y=pcPointsTransformed[1];
        tempPoint.z=pcPointsTransformed[2];
        tempPoint.label=pcl_new.points[j].label;
        tempPoint.r=pcl_new.points[j].r;
        tempPoint.g=pcl_new.points[j].g;
        tempPoint.b=pcl_new.points[j].b;
        pointCloud->points.push_back(tempPoint);

    }

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud0 (new pcl::PointCloud<pcl::PointXYZRGBL>);
    //    transformPrev=transform;
    //  transformPrevRot.matrix().block(0,0,3,3)=transform.matrix().block(0,0,3,3);
    // }

    //  mapPointCloud.save("map.vtk");
    //  pcContainer2.XYZRGBL.clear();
    // pcContainer2.XYZRGBL.shrink_to_fit();
    std::string fullPath1= currentPath + "/" + filename + ".pcd" ;
    std::string fullPath2= currentPath + "/" + filename + ".ply" ;
    std::string fullPath3= currentPath + "/" + filename + ".vtk" ;
    newCloud = rigidTrans->compute(newCloud, T_previous_alignment);
    T_previous_alignment=T_previous_alignment*T_to_map_from_new;

    newCloud.save(fullPath3);
    std::cout<<std::endl;
    std::cout<<fullPath1<<std::endl;
    std::cout<<fullPath2<<std::endl;
    std::cout<<std::endl;
    pointCloud->height=1;
    pointCloud->width=pointCloud->points.size();

    if (!autoMatch){



        std::cout<<"Saving "<<pointCloud->points.size()<<" points"<<std::endl;
        pcl::io::savePCDFileASCII (fullPath1, *pointCloud);
        pcl::io::savePLYFileASCII (fullPath2, *pointCloud);

        if (filename=="aligned_1")
        {
            pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud0 (new pcl::PointCloud<pcl::PointXYZRGBL>);
            std::string fullPath4= currentPath + "/aligned_0.pcd";
            std::string fullPath5= currentPath + "/aligned_0.ply";
            std::string fullPath6= currentPath + "/aligned_0.vtk";
            pcl_ref.height=1;
            pcl_ref.width=pcl_ref.points.size();
            pcl::io::savePCDFileASCII (fullPath4, pcl_ref);
            pcl::io::savePLYFileASCII (fullPath5, pcl_ref);
            mapPointCloud.save(fullPath6);


        }
    }

}





}
