#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>     // 包含相关头文件
#include <pcl/keypoints/harris_3d.h>
#include "resolution.h"    // 用于计算模型分辨率
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("/home/blue/ply2pcd/water/water50_515_pxy_in.pcd", *cloud);
	std::cout << "original cloud size : " << cloud->size() << std::endl;

	double resolution = computeCloudResolution(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression(true);
	detector.setRadiusSearch(10 * resolution);
	detector.setThreshold(1E-6);
	detector.setSearchMethod(tree); // 不写也可以，默认构建kdtree
	detector.setInputCloud(cloud);
	detector.compute(*keypoints_temp);
	pcl::console::print_highlight("Detected %d points !\n", keypoints_temp->size());
	pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypoints_temp, *keys);

        pcl::PCDWriter writer;
        writer.write("/home/blue/ply2pcd/water/water50_515_keypoint_harris.pcd",*keys,false);
        //pcl::visualization::PCLVisualizer viewer;
        //viewer.addPointCloud(keys,"cloud");
        //viewer.spin();

	system("pause");
	return 0;
}
