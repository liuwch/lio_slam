#include <iostream>

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

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>



boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "z");

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

int main(int argc,char* argv[]) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud_src);
    std::cout << "src cloud size:" << cloud_src->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud_tgt);
    std::cout << "tgt cloud size:" << cloud_tgt->size() << std::endl;

    // 
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
    GlobalTransform << -0.997804, -0.0644857, -0.015098, 68.8291, 
                    0.0647312, -0.997768, -0.0163945, 29.1307,
                     -0.014007, -0.0173359, 0.999752, 0.68274,
                      0, 0, 0, 1;

 
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud (*cloud_src, *result, GlobalTransform);
    *result = *result + *cloud_tgt;



    auto viewer = customColourVis(result);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
    }
}
