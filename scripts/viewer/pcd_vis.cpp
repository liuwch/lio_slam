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
// #include <pcl/filters/radius_outlier_removal.h>


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

void radiusOutlierFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInput, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutput, float radius, int num) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(cloudInput);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(num);
    outrem.filter(*cloudOutput);
}

int main(int argc,char* argv[]) {
    // std::string yaml_file = "/data/scripts/data/pcl_vis_config.yaml";
    // YAML::Node vis_cfg = YAML::LoadFile(yaml_file);

    // std::string path = vis_cfg["source_cloud_path"].as<std::string>();
    // std::cout << "path: " << path << std::endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud);
    std::cout << "mapping cloud size:" << cloud->size() << std::endl;

    radiusOutlierFilter(cloud, cloud_filtered, 1.5, 30);
    std::cout << "radius filter size:" << cloud_filtered->size() << std::endl;


    auto viewer = customColourVis(cloud_filtered);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
    }
}
