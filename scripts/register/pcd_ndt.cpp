#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    //加载房间的第一次扫描
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;
    //加载从新视角得到的房间的第二次扫描
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

  


    //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*src_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(target_cloud);
    approximate_voxel_filter.filter(*tgt_cloud);

    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(src_cloud);//输入点云
	passthrough.setFilterFieldName("z");//对z轴进行操作
	passthrough.setFilterLimits(1, 4);//设置直通滤波器操作范围
	// passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
	passthrough.filter(*src_cloud);

    pcl::PassThrough<pcl::PointXYZ> passthrough_y;
    passthrough_y.setInputCloud(src_cloud);//输入点云
	passthrough_y.setFilterFieldName("y");//对z轴进行操作
	passthrough_y.setFilterLimits(-35, 35);//设置直通滤波器操作范围
	// passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
	passthrough_y.filter(*src_cloud);


    pcl::PassThrough<pcl::PointXYZ> passthrough_tgt;
    passthrough_tgt.setInputCloud(tgt_cloud);//输入点云
	passthrough_tgt.setFilterFieldName("z");//对z轴进行操作
	passthrough_tgt.setFilterLimits(1, 4);//设置直通滤波器操作范围
	// passthrough_tgt.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
	passthrough_tgt.filter(*tgt_cloud);

    pcl::PassThrough<pcl::PointXYZ> passthrough_tgt_y;
    passthrough_tgt_y.setInputCloud(tgt_cloud);//输入点云
    passthrough_tgt_y.setFilterFieldName("y");//对z轴进行操作
	passthrough_tgt_y.setFilterLimits(-35, 35);//设置直通滤波器操作范围
	// passthrough_tgt.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
	passthrough_tgt_y.filter(*tgt_cloud);

    // std::cout << "Filtered cloud contains " << filtered_cloud->size()
    //     << " data points from room_scan2.pcd" << std::endl;
    std::cout << "--------------------------------NDT started-------------------------------" << std::endl;
    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(1.0);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(5);
    // 设置要配准的点云
    ndt.setInputCloud(src_cloud);
    //设置点云配准目标
    ndt.setInputTarget(tgt_cloud);

    std::cout << "--------------------------------transform started-------------------------------" << std::endl;
    //设置使用机器人测距法得到的初始对准估计结果
    // Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    // Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    std::cout << "src_cloud size : " << src_cloud->size() << std::endl;
    std::cout << "tgt_cloud size : " << tgt_cloud->size() << std::endl;

    Eigen::Matrix4f InitTransform = Eigen::Matrix4f::Identity ();
    Eigen::AngleAxisf rotation_vector ( M_PI*0.99, Eigen::Vector3f ( 0,0,1 ) ); 
    InitTransform.topLeftCorner(3,3) = rotation_vector.toRotationMatrix();
    InitTransform.topRightCorner(3,1) = Eigen::Vector3f(70, 30, 0);
    
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "align" << std::endl;
    ndt.align(*output_cloud, InitTransform);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;

    //使用创建的变换对未过滤的输入点云进行变换
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    //保存转换的输入点云
    std::cout << "--------------------------------saving qtruck_transformed.pcd------------------------------" << std::endl;
    pcl::io::savePCDFileASCII("qtruck_transformed.pcd", *output_cloud);
    // 初始化点云可视化界面
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);
    //对目标点云着色（红色）并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "target cloud");
    //对转换后的目标点云着色（绿色）并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "output cloud");
    // 启动可视化
    viewer_final->addCoordinateSystem(1.0);
    viewer_final->initCameraParameters();
    //等待直到可视化窗口关闭。
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}
