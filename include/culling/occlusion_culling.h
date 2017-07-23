#ifndef OCCLUSION_H_
#define OCCLUSION_H_

#include <iostream>

#include "ros/ros.h"
#include <ros/package.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <geometry_msgs/Pose.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Custom classes
#include <culling/frustum_culling.h>
#include <culling/voxel_grid_occlusion_estimation.h>

class OcclusionCulling
{
public:
	// >>>>>>>>
    // Attributes
    // >>>>>>>>
    ros::NodeHandle  nh;
    std::string model;
    //     ros::Publisher original_pub;
    //     ros::Publisher visible_pub;
    ros::Publisher fov_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr occlusionFreeCloud;//I can add it to accumulate cloud if I want to extract visible surface from multiple locations
    pcl::PointCloud<pcl::PointXYZ>::Ptr FrustumCloud;//frustum cull

    pcl::PointCloud<pcl::PointXYZ> FreeCloud;
    float voxelRes, OriginalVoxelsSize;
    double id;
    pcl::VoxelGridOcclusionEstimationT voxelFilterOriginal;
    Eigen::Vector3i  max_b1, min_b1;
    visualization_msgs::Marker linesList1,linesList2,linesList3,linesList4;
    visualization_msgs::MarkerArray marker_array;
    pcl::FrustumCullingTT fc;
    double maxAccuracyError, minAccuracyError;

    // >>>>>>>>
    // Methods
    // >>>>>>>>
    //OcclusionCulling();
    //OcclusionCulling(std::string modelName);
    //OcclusionCulling(ros::NodeHandle & n, std::string modelName);
    OcclusionCulling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr);
    OcclusionCulling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPtr);
    
    ~OcclusionCulling(){};
    void initConfig(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr);
    void initConfig(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPtr, ros::NodeHandle nodeHandle);
    void visualizeRaycast(geometry_msgs::Pose location);
    
    pcl::PointCloud<pcl::PointXYZ> extractVisibleSurface(geometry_msgs::Pose location);
    //    float calcCoveragePercent(geometry_msgs::Pose location);
    float calcCoveragePercent(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
    double calcAvgAccuracy(pcl::PointCloud<pcl::PointXYZ> pointCloud);
    void visualizeFOV(geometry_msgs::Pose location);
    
    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[]);
    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[], float scale);


};

#endif 
