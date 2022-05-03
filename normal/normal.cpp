// normal.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <exception>

#include <pcl/filters/extract_indices.h>
using namespace pcl;
double PointDistance(PointXYZRGB A, PointXYZRGB B)
{
	double d2 = pow(A.x - B.x, 2) + pow(A.y - B.y, 2) + pow(A.z - B.z, 2);
	double distance = sqrt(d2);
	return distance;

}

void cutCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,float distance)
{
	int index1 = 0, index2 = 1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_obstacles(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	while (index2 < cloud->size())
	{
		if (PointDistance(cloud->points[index1], cloud->points[index2]) < distance)
			inliers->indices.push_back(index2++);
		else
			index1 = index2++;
	}
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud);
}


int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("asd.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file rabbit.pcd\n");
		//return(-1);
	}
	std::cout << cloud->points.size() << std::endl;
	cutCloud(cloud, 0.5);
	//可视化
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);

	//viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
    std::cout << "Hello World!\n";
}
