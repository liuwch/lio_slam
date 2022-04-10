#include <iostream>
#include <sstream>  
#include<iomanip>

#include <boost/filesystem.hpp>
#include "yaml-cpp/yaml.h"

#include <boost/thread/thread.hpp>
// #include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "pcl/filters/radius_outlier_removal.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <vector>



boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "intensity");

    //创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 0, 0);

    //addPointCloud<>()完成对颜色处理器对象的传递
    viewer->addText("Boudary point clouds", 80, 80, "v1_text", 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

//    viewer->addSphere(cloud->points[0], cloud->points[cloud->size() -1], 'line');
    return (viewer);
}

void keep_box(std::vector<int> &box, pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output) {
    pcl::PassThrough<pcl::PointXYZI> passthrough;
    // passthrough.setInputCloud(input);//输入点云
    // passthrough.setFilterFieldName("y");//对z轴进行操作
    // passthrough.setFilterLimits(box[2], box[3]);//设置直通滤波器操作范围
    // passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*input);

    passthrough.setInputCloud(input);//输入点云
    
    passthrough.setFilterFieldName("x");//对z轴进行操作
    passthrough.setFilterLimits(box[0], box[1]);//设置直通滤波器操作范围

    passthrough.setFilterFieldName("y");//对z轴进行操作
    passthrough.setFilterLimits(box[2], box[3]);//设置直通滤波器操作范围

    // passthrough.setFilterFieldName("z");//对z轴进行操作
    // passthrough.setFilterLimits(box[4], box[5]);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
    passthrough.filter(*output);

    // passthrough.setInputCloud(input);//输入点云
    // passthrough.setFilterFieldName("z");//对z轴进行操作
    // passthrough.setFilterLimits(box[4], box[5]);//设置直通滤波器操作范围
    // passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*output);
}

void filter_box(std::vector<int> &box, pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output) {
    pcl::PassThrough<pcl::PointXYZI> passthrough;
  

    // passthrough.setInputCloud(input);//输入点云
    // passthrough.setFilterFieldName("y");//对z轴进行操作
    // passthrough.setFilterLimits(box[2], box[3]);//设置直通滤波器操作范围
    // passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*input);

    passthrough.setInputCloud(input);//输入点云
    
    passthrough.setFilterFieldName("x");//对z轴进行操作
    passthrough.setFilterLimits(box[0], box[1]);//设置直通滤波器操作范围

    passthrough.setFilterFieldName("y");//对z轴进行操作
    passthrough.setFilterLimits(box[2], box[3]);//设置直通滤波器操作范围

    // passthrough.setFilterFieldName("z");//对z轴进行操作
    // passthrough.setFilterLimits(box[4], box[5]);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    passthrough.filter(*output);

    // passthrough.setInputCloud(input);//输入点云
    // passthrough.setFilterFieldName("z");//对z轴进行操作
    // passthrough.setFilterLimits(box[4], box[5]);//设置直通滤波器操作范围
    // passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*output);
}

int main(int argc,char* argv[]) {
    // std::string yaml_file = "/data/scripts/data/pcl_vis_config.yaml";
    // YAML::Node vis_cfg = YAML::LoadFile(yaml_file);

    // std::string path = vis_cfg["source_cloud_path"].as<std::string>();
    // std::cout << "path: " << path << std::endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud);
    std::cout << "mapping cloud size:" << cloud->size() << std::endl;

    // 
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
    Eigen::AngleAxisf rotation_vector ( M_PI * 0, Eigen::Vector3f ( 0,0,1 ) ); 
    GlobalTransform.topLeftCorner(3,3) = rotation_vector.toRotationMatrix();
    GlobalTransform.topRightCorner(3,1) = Eigen::Vector3f(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*cloud, *result, GlobalTransform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keep(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new1(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<int> box(6);
    box = {5, 100, 0, 13, -10, 10};
    keep_box(box, cloud, cloud_keep);
    filter_box(box, cloud, cloud_filter);
    // 
    pcl::UniformSampling<pcl::PointXYZI> sor2;    
    sor2.setInputCloud(cloud_keep);             
    sor2.setRadiusSearch(0.05);      
    sor2.filter(*cloud_keep);       

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(cloud_keep);                 
    /****搜索半径设为radius，在此半径内点必须要有至少num个邻居时，此点才会被保留***/
    outrem.setRadiusSearch(0.16);              
    outrem.setMinNeighborsInRadius(12);        
    outrem.filter(*cloud_keep);   

    *cloud_new = *cloud_keep + *cloud_filter;

    std::vector<int> box1(6);
    box1 = {10, 20, 50, 60, -10, 10};
    keep_box(box1, cloud_new, cloud_keep);
    filter_box(box1, cloud_new, cloud_filter);
    // 
   
    sor2.setInputCloud(cloud_keep);             
    sor2.setRadiusSearch(0.05);      
    sor2.filter(*cloud_keep);       

    outrem.setInputCloud(cloud_keep);                 
    outrem.setRadiusSearch(0.16);              
    outrem.setMinNeighborsInRadius(8);        
    outrem.filter(*cloud_keep);   

    *cloud_new1 = *cloud_keep + *cloud_filter;

    std::cout << "filtered cloud size:" << cloud_keep->size() << std::endl;
    std::cout << "filtered cloud size:" << cloud_filter->size() << std::endl;

    std::vector<int> box2(6);
    box2 = {-3, 100, -50, 70, -10, 10};
    keep_box(box2, cloud_new1, cloud_new1);

    pcl::PassThrough<pcl::PointXYZI> passthrough;
    passthrough.setInputCloud(cloud_new1);//输入点云
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(-5, 5);//设置直通滤波器操作范围
    passthrough.filter(*cloud_new1);

    // std::vector<int> box1(6);
    // box1 = {5, 100, 50, 60, -10, 10};
    // keep_box(box1, cloud_new, cloud_keep);
    // filter_box(box1, cloud_new, cloud_filter);
    
    // sor2.setInputCloud(cloud_keep);             
    // sor2.setRadiusSearch(0.05);      
    // sor2.filter(*cloud_keep);  

    // outrem.setInputCloud(cloud_keep);                 
    // outrem.setRadiusSearch(0.16);              
    // outrem.setMinNeighborsInRadius(8);        
    // outrem.filter(*cloud_keep); 

    // *cloud_new1 = *cloud_keep + *cloud_filter;
    // std::cout << "filtered cloud size:" << cloud_new1->size() << std::endl;

    // pcl::PassThrough<pcl::PointXYZI> passthrough;
    // passthrough.setInputCloud(result);//输入点云
    // passthrough.setFilterFieldName("x");//对z轴进行操作
    // passthrough.setFilterLimits(-4, 4);//设置直通滤波器操作范围
    // // passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*result);

    // passthrough.setInputCloud(result);//输入点云
    // passthrough.setFilterFieldName("y");//对z轴进行操作
    // passthrough.setFilterLimits(-50, 70);//设置直通滤波器操作范围
    // // passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*result);

    // passthrough.setInputCloud(result);//输入点云
    // passthrough.setFilterFieldName("x");//对z轴进行操作
    // passthrough.setFilterLimits(-5, 170);//设置直通滤波器操作范围
    // // passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    // passthrough.filter(*result);

    // std::vector<int> box(6);
    // box = {-5, 100, -50, 70, -4, 4};
    // filter_box(box, result, result);

    pcl::io::savePCDFileBinary("/data/scripts/data/filteredGlobalMap.pcd", *cloud_new1);
    std::cout << "Saving map to pcd files completed\n" << std::endl;

    auto viewer = customColourVis(cloud_new1);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
    }
}
