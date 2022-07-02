#pragma once

#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h> 
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>

class FillingExtractor {
    private:
    ros::Subscriber cloud_sub; 
    std::string range_axis;
    double range_min, range_max;
    double dist_thresh;
    bool set_negative;
    int min_red, min_green, min_blue;
    bool keep_organized;
    int mean_k;
    double stddev_thresh;
    double radius_search;

    public:
    FillingExtractor(ros::NodeHandle&);
    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    ros::Publisher distance_pub;
    ros::Publisher coeff_pub;
    sensor_msgs::PointCloud2 output;
    std::string input_topic;
    std::string output_topic;
    std::string distance_topic;

    std::vector<double> distances;
    double max;

    pcl::PointCloud<pcl::PointXYZRGB> sampleUniform(pcl::UniformSampling<pcl::PointXYZRGB>&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    pcl::PointCloud<pcl::PointXYZRGB> removeStatOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>&);
    pcl::PointCloud<pcl::PointXYZRGB> filterColor(pcl::ConditionalRemoval<pcl::PointXYZRGB>&, pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    pcl::PointCloud<pcl::PointXYZRGB> extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients::Ptr, pcl::PointIndices::Ptr, pcl::ExtractIndices<pcl::PointXYZRGB>&, pcl::SACSegmentation<pcl::PointXYZRGB>&);
    pcl::PointCloud<pcl::PointXYZRGB> filterField(pcl::ConditionalRemoval<pcl::PointXYZRGB>&, pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    void initializeSubscribers();
    void initializePublishers();  
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
};
