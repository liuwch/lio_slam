#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(argv[1], *cloud);
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
    return 0;
}


