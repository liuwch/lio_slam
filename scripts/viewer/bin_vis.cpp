#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "x");

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud);
    std::cout << "mapping cloud size:" << cloud->size() << std::endl;
    auto viewer = customColourVis(cloud);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10));
    }
}
