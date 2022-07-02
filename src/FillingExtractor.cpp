/**
 * @file FillingExtractor.cpp
 * @author Yunus Emre Karaoğlan
 * @brief Filling Extractor class functions definitions. 
 This class is used to extract the filling from a point cloud 
 that contains case, filling and a desk. The filling is extracted
    by using:
    -> Conditional Removal depends on range
    -> SAC Plane Segmentation using RANSAC
    -> Conditional Removal depends on color
    -> Statistical Outlier Removal
    -> Uniform Sampling
 * @version 0.1
 * @date 2022-07-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "filling_extraction/FillingExtractor.h"


// Calculate the distance between a point and a plane for given parameters
double pointDistance(double a, double b, double c, double d, double x, double y, double z)
{
    return std::abs(a*x + b*y + c*z + d) / std::sqrt(a*a + b*b + c*c);
}

// Constructor for FillingExtractor class, initializes all parameters, subscribers and publishers
FillingExtractor::FillingExtractor(ros::NodeHandle& nh) {
    this->nh = nh;
    this->initializeSubscribers();
    this->initializePublishers();
    nh.param<std::string>("/filling_extractor/range_condition/axis", this->range_axis, "z");
    nh.param<double>("/filling_extractor/range_condition/min_value", this->range_min, 0.0);
    nh.param<double>("/filling_extractor/range_condition/max_value", this->range_max, 0.9);
    nh.param<double>("/filling_extractor/plane_segmentation/distance_threshold", this->dist_thresh, 0.01);
    nh.param<bool>("/filling_extractor/plane_segmentation/set_negative", this->set_negative, true);
    nh.param<int>("/filling_extractor/color_filter/min_r", this->min_red, 200);
    nh.param<int>("/filling_extractor/color_filter/min_g", this->min_green, 200);
    nh.param<int>("/filling_extractor/color_filter/min_b", this->min_blue, 200);
    nh.param<bool>("/filling_extractor/color_filter/keep_organized", this->keep_organized, true);
    nh.param<int>("/filling_extractor/removing_statistical_outliers/mean_k", this->mean_k, 50);
    nh.param<double>("/filling_extractor/removing_statistical_outliers/stddev_thresh", this->stddev_thresh, 1.0);
    nh.param<double>("/filling_extractor/uniform_sampling/radius_search", this->radius_search, 0.02);
}

// Initialize subscriber for input point cloud 
void FillingExtractor::initializeSubscribers() {
    nh.param<std::string>("filling_extractor/input_topic", this->input_topic, "/filling_extractor/input");
    this->cloud_sub = this->nh.subscribe(this->input_topic, 1, &FillingExtractor::cloud_cb, this);
}

// Initialize publishers for output point cloud, distance and plane coefficients informations
void FillingExtractor::initializePublishers() {
    nh.param<std::string>("filling_extractor/output_topic", this->output_topic, "/filling_extractor/output");
    nh.param<std::string>("filling_extractor/distance_topic", this->distance_topic, "/filling_extractor/distance");
    this->cloud_pub = this->nh.advertise<sensor_msgs::PointCloud2>(this->output_topic, 1);
    this->distance_pub = this->nh.advertise<std_msgs::Float64>(this->distance_topic, 1);
    this->coeff_pub = this->nh.advertise<std_msgs::Float64MultiArray>("/filling_extractor/coeff", 1);
}


pcl::PointCloud<pcl::PointXYZRGB> FillingExtractor::filterField(pcl::ConditionalRemoval<pcl::PointXYZRGB>& condrem,
                                    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond, 
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_)
/**
 * @brief filtering the point cloud based on the range condition
 * 
 * @param condrem ConditionalRemoval object
 * @param range_cond Removal conditions
 * @param cloud_ Input point cloud
 * @return pcl::PointCloud<pcl::PointXYZRGB> 
 */
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud_);
    condrem.setKeepOrganized (this->keep_organized);
    condrem.filter (*(cloud_f));
    return *(cloud_f);
}

pcl::PointCloud<pcl::PointXYZRGB> FillingExtractor::extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
                                               pcl::ModelCoefficients::Ptr coefficients_, 
                                               pcl::PointIndices::Ptr indices_, 
                                               pcl::ExtractIndices<pcl::PointXYZRGB>& extract,
                                               pcl::SACSegmentation<pcl::PointXYZRGB>& seg_)
/**
 * @brief Plane Detection and Extraction
 * 
 * @param cloud_ input point cloud
 * @param coefficients_ Plane coefficients from SACSegmentation (a, b, c, d)
 * @param indices_ Point indices
 * @param extract ExtractIndices object
 * @param seg_ SACSegmentation object
 * @return pcl::PointCloud<pcl::PointXYZRGB> 
 */
{
    std_msgs::Float64MultiArray coeff_msg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setDistanceThreshold(this->dist_thresh);
    seg_.setInputCloud (cloud_);
    seg_.segment(*(indices_), *(coefficients_));
    for (int i = 0; i < 4; i++) {
        coeff_msg.data.push_back(coefficients_->values[i]);
    }
    this->coeff_pub.publish(coeff_msg);

    extract.setInputCloud (cloud_);
    extract.setIndices (indices_);
    extract.setNegative (this->set_negative);
    extract.filter (*(cloud_plane));
    return *(cloud_plane);
}

pcl::PointCloud<pcl::PointXYZRGB> FillingExtractor::filterColor(pcl::ConditionalRemoval<pcl::PointXYZRGB>& condrem,
                                    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond, 
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_)
/**
 * @brief filtering the point cloud based on the color condition 
 * 
 * @param condrem ConditionalRemoval object
 * @param color_cond Color conditions
 * @param cloud_ input point cloud
 * @return pcl::PointCloud<pcl::PointXYZRGB> 
 */
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    condrem.setCondition (color_cond);
    condrem.setInputCloud (cloud_);
    condrem.setKeepOrganized (this->keep_organized);
    condrem.filter (*(cloud_f_color));
    return *(cloud_f_color);
}

pcl::PointCloud<pcl::PointXYZRGB> FillingExtractor::removeStatOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
                                                                       pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>& sor_)
/**
 * @brief Remove statistical outliers to clean the point cloud
 * 
 * @param cloud_ Input point cloud
 * @param sor_ StatisticalOutlierRemoval object
 * @return pcl::PointCloud<pcl::PointXYZRGB> 
 */
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f_stat(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor_.setInputCloud (cloud_);
    sor_.setMeanK (this->mean_k);
    sor_.setStddevMulThresh (this->stddev_thresh);
    sor_.filter (*(cloud_f_stat));
    return *(cloud_f_stat); 
}

pcl::PointCloud<pcl::PointXYZRGB> FillingExtractor::sampleUniform(pcl::UniformSampling<pcl::PointXYZRGB>& uniform_,
                                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_)
/**
 * @brief Uniform sampling to clean and reduce the point cloud 
 * 
 * @param uniform_ UniformSampling object
 * @param cloud_ Input point cloud
 * @return pcl::PointCloud<pcl::PointXYZRGB> 
 */
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f_uniform(new pcl::PointCloud<pcl::PointXYZRGB>);
    uniform_.setInputCloud (cloud_);
    uniform_.setRadiusSearch (this->radius_search);
    uniform_.filter (*(cloud_f_uniform));
    return *(cloud_f_uniform);
}

void FillingExtractor::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    /**
 * @brief ROS PointCloud2 callback, extract the meaningful point cloud and publish it
 * 
 * @param input input PointCloud2 message
 */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_uniform(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_range;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_color;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    pcl::UniformSampling<pcl::PointXYZRGB> uniform;
    
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    pcl::fromROSMsg(*input, *cloud);
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> (this->range_axis, pcl::ComparisonOps::GT, this->range_min)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> (this->range_axis, pcl::ComparisonOps::LT, this->range_max)));

    *cloud_filtered = filterField(condrem_range, range_cond, cloud);

    *cloud_filtered = extractPlane(cloud_filtered, coefficients, inliers, extract, seg);

    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, this->min_red)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, this->min_green)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, this->min_blue)));

    *cloud_filtered_color = filterColor(condrem_color, color_cond, cloud_filtered);

    *cloud_filtered_sor = removeStatOutliers(cloud_filtered_color, sor);

    *cloud_filtered_uniform = sampleUniform(uniform, cloud_filtered_sor);

    for (const auto& point: *(cloud_filtered_uniform))
    {
        distances.push_back(pointDistance(coefficients->values[0],
                                        coefficients->values[1], 
                                        coefficients->values[2], 
                                        coefficients->values[3], 
                                        point.x, point.y, point.z));
    }
    std_msgs::Float64 max_distance;
    max_distance.data = *std::max_element(distances.begin(), distances.end());
    this->distance_pub.publish(max_distance);
    distances.clear();

    pcl::toROSMsg(*(cloud_filtered_uniform), this->output);
    output.header.frame_id = input->header.frame_id;
    output.header.stamp = input->header.stamp;
    this->cloud_pub.publish(output);
}

