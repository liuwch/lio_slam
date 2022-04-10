#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
 
#include <iostream>
#include <fstream>
 
using namespace pcl;
using namespace std;
 
namespace po = boost::program_options;
 
int main(int argc, char **argv){
	///The file to read from.
	string infile{argv[1]};
 
	///The file to output to.
	string outfile{argv[2]};
 
	// load point cloud
	fstream input(infile.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
 
	pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
 
	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
 
	cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << endl;
 
    //pcl::PCDWriter writer;
 
    // Save DoN features
    pcl::io::savePCDFileBinary(outfile, *points);
    //writer.write<PointXYZI> (outfile, *points, false);
}
