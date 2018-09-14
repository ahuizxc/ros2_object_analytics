/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <vector>
#include <pcl/common/time.h>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
//#include <pcl/segmentation/impl/organized_connected_component_segmentation.hpp>
//#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <rcutils/logging_macros.h>
#include "object_analytics_node/const.hpp"
#include "object_analytics_node/segmenter/organized_multi_plane_segmenter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
namespace object_analytics_node
{
namespace segmenter
{
using pcl::Label;
using pcl::Normal;
using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PlanarRegion;

OrganizedMultiPlaneSegmenter::OrganizedMultiPlaneSegmenter()
  : conf_(AlgorithmConfig())
  , plane_comparator_(new pcl::PlaneCoefficientComparator<PointT, Normal>)
  , euclidean_comparator_(new pcl::EuclideanPlaneCoefficientComparator<PointT, Normal>)
  , edge_aware_comparator_(new pcl::EdgeAwarePlaneComparator<PointT, Normal>)
  , euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, Normal, Label>)
{
  applyConfig();
}

void OrganizedMultiPlaneSegmenter::segment(const PointCloudT::ConstPtr& cloud, PointCloudT::Ptr& cloud_segment,
                                           std::vector<PointIndices>& cluster_indices)
{
  double start = pcl::getTime();
  RCUTILS_LOG_DEBUG("Total original point size = %d", cloud->size());
  //pcl::copyPointCloud(*cloud, *cloud_segment);  // cloud_segment is same as cloud for this algorithm



  // std::vector<double> z_vector;
  // Eigen::Vector4f centroid;    
  // pcl::compute3DCentroid(*cloud, centroid);
  // std::cout << "center---x: " << centroid[0] << " y: " << centroid[1] << " z: " << centroid[2] << std::endl;
  // for (size_t i=0; i<=cloud->points.size(); i++)
  // {
  //   z_vector.push_back(cloud->points[i].z);
  // }
  // double sum = std::accumulate(std::begin(z_vector), std::end(z_vector), 0.0);  
  // double mean =  centroid[2]; //均值     
  // double accum  = 0.0;  
  // std::for_each (std::begin(z_vector), std::end(z_vector), [&](const double d) {  
  //     accum  += (d-mean)*(d-mean);  
  // });
  // double stdev = sqrt(accum/(z_vector.size()-1)); //方差 
  // double z_max = 0.5 * stdev + mean;
  // double z_min = -0.5 * stdev + mean;
  // for (size_t i=0;i<=cloud->points.size(); i++)
  // {
  //   pcl::PointXYZ point;
  //   point = cloud->points[i];
  //   if (point.z > z_max)
  //   {
  //     point.z = z_max;
  //   }
  //   if (point.z < z_min)
  //   {
  //     point.z = z_min;
  //   }
  //   point.z = mean;
  //   cloud_segment->points.push_back(point);
  // }
  // cloud_segment->height = cloud->height;
  // cloud_segment->width = cloud->width;
  // pcl::PassThrough<pcl::PointXYZ> pass;  
  // pass.setInputCloud (cloud);  
  // pass.setFilterFieldName ("z");  
  // pass.setFilterLimits (z_min, z_max);  
  // //pass.setFilterLimitsNegative (true);  
  // pass.filter (*cloud_segment);


  // pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
  // pcl::IndicesPtr indices (new std::vector <int>);
  // sor.setInputCloud (cloud);               //设置待滤波的点云
  // sor.setRadiusSearch (0.5);                               //设置在进行统计时考虑查询点临近点数
  // sor.setMinNeighborsInRadius(3);
  // std::cout << "filter begin!" << std::endl;                     //设置判断是否为离群点的阀值  
  // sor.filter (*cloud_segment);             //存储
  // std::cout << "filter end!" << std::endl;
  // cloud_segment->height = cloud->height;
  // cloud_segment->width = cloud->width;

  // PointCloud<Normal>::Ptr normal_cloud(new PointCloud<Normal>);
  // estimateNormal(cloud, normal_cloud);

  PointCloud<Label>::Ptr labels(new PointCloud<Label>);
  std::vector<PointIndices> label_indices;
  segmentObjects(cloud, cloud_segment, labels, label_indices, cluster_indices);

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Segmentation : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::estimateNormal(const PointCloudT::ConstPtr& cloud,
                                                  PointCloud<Normal>::Ptr& normal_cloud)
{
  double start = pcl::getTime();

  pcl::copyPointCloud(*cloud, *normal_cloud);

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Calc normal : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::segmentObjects(const PointCloudT::ConstPtr& cloud, PointCloudT::Ptr& cloud_segment, PointCloud<Label>::Ptr labels,
                                                  std::vector<PointIndices>& label_indices,
                                                  std::vector<PointIndices>& cluster_indices)
{
  double start = pcl::getTime();

  std::vector<bool> plane_labels;
  plane_labels.resize(cloud->points.size(), false);
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    pcl::Label lab;
    lab.label = i;
    labels->points.push_back(lab);
  }
  pcl::UniformSampling<PointT> uniform_sampling;
  PointCloudT::Ptr cloud_sampling(new PointCloudT);
  PointCloudT::Ptr cloud_filter(new PointCloudT);
  uniform_sampling.setInputCloud (cloud);
  uniform_sampling.setRadiusSearch (0.005f);
  uniform_sampling.filter (*cloud_sampling);
  pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud_sampling);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.2f, 15.0f);
  pass.filter(*cloud_filter);
  std::cout << "Model total points: " << cloud->points.size () << "; Selected Keypoints: " << cloud_filter->points.size () << std::endl;
  	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval<PointT> sor;
  if (cloud_filter->points.size() == 0)
  {
    return;
  }
  else{
	sor.setInputCloud(cloud_filter);
  }
	sor.setMeanK(20);
	sor.setStddevMulThresh(0.002f);
	sor.filter(*cloud_segment);
  std::cout << "downsampled points: " << cloud_filter->points.size () << "; removal points: " << cloud_segment->points.size () << std::endl;
  // PointCloud<Label> euclidean_labels;
  // pcl::OrganizedConnectedComponentSegmentation<PointT, Label> euclidean_segmentation(euclidean_cluster_comparator_);
  // // euclidean_segmentation.setProjectPoints(true);
  // std::cout << "height: " << cloud->height << ", width: " << cloud->width << std::endl;
  // euclidean_segmentation.setInputCloud(cloud_filter);
  // euclidean_segmentation.segment(euclidean_labels, cluster_indices);

  // auto func = [this](PointIndices indices) { return indices.indices.size() < this->object_minimum_points_; };
  // cluster_indices.erase(std::remove_if(cluster_indices.begin(), cluster_indices.end(), func), cluster_indices.end());

  double end = pcl::getTime();
  RCUTILS_LOG_DEBUG("Cluster : %f", double(end - start));
}

void OrganizedMultiPlaneSegmenter::applyConfig()
{
  plane_minimum_points_ = conf_.get<size_t>("PLANE_MINIMUM_POINTS", 2000);
  object_minimum_points_ = conf_.get<size_t>("OBJECT_MINIMUM_POINTS", 200);

  normal_estimation_.setNormalEstimationMethod(normal_estimation_.SIMPLE_3D_GRADIENT);
  normal_estimation_.setNormalEstimationMethod(normal_estimation_.COVARIANCE_MATRIX);
  normal_estimation_.setMaxDepthChangeFactor(conf_.get<float>("NORMAL_MAX_DEPTH_CHANGE", 0.02f));
  normal_estimation_.setNormalSmoothingSize(conf_.get<float>("NORMAL_SMOOTH_SIZE", 30.0f));

  // euclidean_cluster_comparator_->setDistanceThreshold(conf_.get<float>("EUCLIDEAN_DISTANCE_THRESHOLD", 0.02f), false);

  plane_segmentation_.setMinInliers(conf_.get<size_t>("MIN_PLANE_INLIERS", 1000));
  plane_segmentation_.setAngularThreshold(pcl::deg2rad(conf_.get<float>("NORMAL_ANGLE_THRESHOLD", 2.0f)));
  plane_segmentation_.setDistanceThreshold(conf_.get<float>("NORMAL_DISTANCE_THRESHOLD", 0.02f));

  const std::string comparator = conf_.get<std::string>("COMPARATOR", "PlaneCoefficientComparator");
  if (comparator == "PlaneCoefficientComparator")
  {
    plane_segmentation_.setComparator(plane_comparator_);
  }
  else if (comparator == "EuclideanPlaneCoefficientComparator")
  {
    plane_segmentation_.setComparator(euclidean_comparator_);
  }
  else if (comparator == "EdgeAwarePlaneComaprator")
  {
    plane_segmentation_.setComparator(edge_aware_comparator_);
  }
}
}  // namespace segmenter
}  // namespace object_analytics_node
