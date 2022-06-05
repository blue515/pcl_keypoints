#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>    // 包含相关头文件
#include <pcl/keypoints/iss_3d.h>
#include "resolution.h" // 用于计算模型分辨率
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("/home/blue/ply2pcd/char_inliers_passthrough.pcd", *cloud);
        //pcl::PCDReader reader;
        //reader.read ("/home/blue/ply2pcd/char_inliers_passthrough.pcd", *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * resolution);
	iss_detector.setNonMaxRadius(4 * resolution);
	iss_detector.setThreshold21(0.975);
	iss_detector.setThreshold32(0.975);
	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);
	iss_detector.setInputCloud(cloud);

	pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
	iss_detector.compute(*keys);
	std::cout << "key points size : " << keys->size() << std::endl;
        //pcl::PCDWriter writer;
        //writer.write ("/home/blue/ply2pcd/char_filter_iss.pcd",*keys, false);

        pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(keys, "cloud");
	//viewer.addPointCloudNormals<PointNT>(normals, level, scale, "normals");

	viewer.spin();

	system("pause");
	return 0;
}
