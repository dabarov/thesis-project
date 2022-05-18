#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <kinova_moveit/TargetObjectPcl.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>

ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pubParams;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  ros::WallTime start_, end_;
  double execution_time = 0.0;
  // ROS_INFO("Got callback\n");
  //  Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  // ROS_INFO("Initial points %i \n",  cloud->points.size());
  // pcl::ModelCoefficients coefficients;
  // pcl::PointIndices inliers;

  // Statistical outlier
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(20);
  sor.setStddevMulThresh(0.1);
  sor.filter(*cloud_filtered);

  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> sor2;
  sor2.setInputCloud(cloud_filtered);
  sor2.setLeafSize(0.01f, 0.01f, 0.01f);
  sor2.filter(*cloud_filtered1);
  cloud_filtered.swap(cloud_filtered1);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_filtered1);

  // ROS_INFO("Filtered\n");

  start_ = ros::WallTime::now();
  // Plane segmentaiton
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.005);

  // remove all planes
  int count = 0;
  int num_of_points = cloud_filtered1->points.size();
  if (num_of_points < 10)
  {
    ROS_INFO("No points");
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 cloud_inliers_pcl2;
    pcl::toPCLPointCloud2(*cloud_filtered, cloud_inliers_pcl2);
    pcl_conversions::fromPCL(cloud_inliers_pcl2, output);
    pub.publish(output);
    return;
  }
  int current_points = num_of_points;
  while (current_points > 0.100 * num_of_points)
  {
    seg.setInputCloud(cloud_filtered1);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      break;
    }

    // ROS_INFO("Segmentation Done\n");

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered1);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_p);
    current_points = cloud_p->points.size();
    cloud_filtered1.swap(cloud_p);
    count++;
    // ROS_INFO("Final points %i / %i \n",  current_points, num_of_points);
    // ROS_INFO("After swap %i / %i \n",  cloud_p->points.size(), cloud_filtered1->points.size());
  }
  end_ = ros::WallTime::now();
  // print results
  execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO("Segmentation time (ms): %f", execution_time);
  ROS_INFO("Final points %i / %i ", cloud_filtered1->points.size(), num_of_points);

  // Start KdTree
  // Creating the KdTree object for the search method of the extraction
  pcl::PCDWriter writer;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.01); // 2cm
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(2500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered1);
  ec.extract(cluster_indices);
  // std::cout << cluster_indices.empty() << " " << cluster_indices.size() << "\n";
  if (cluster_indices.empty())
  {
    ROS_INFO("No clusters!");
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 cloud_inliers_pcl2;
    pcl::toPCLPointCloud2(*cloud_filtered, cloud_inliers_pcl2);
    pcl_conversions::fromPCL(cloud_inliers_pcl2, output);
    pub.publish(output);
    return;
  }
  ROS_INFO("Number of clusters: %f", cluster_indices.empty());
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract_detected;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_detected(new pcl::PointCloud<pcl::PointXYZ>);

  // sensor_msgs::PointCloud2 output2;
  // pcl::PCLPointCloud2 cloud_inliers_pcl2_2;

  int j = 0;
  int flag = -1;
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &idx : it->indices)
    cloud_cluster->push_back((*cloud_filtered1)[idx]); //*
  cloud_cluster->width = cloud_cluster->size();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;

  std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
  // std::stringstream ss;
  // ss << "cloud_cluster_" << j << ".pcd";
  // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);

  start_ = ros::WallTime::now();
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud_cluster);
  feature_extractor.compute();
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  std::cout << "Bounding box" << min_point_AABB << " " << max_point_AABB << " \n";
  end_ = ros::WallTime::now();
  // print results
  execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO("Bounding box time (ms): %f", execution_time);

  start_ = ros::WallTime::now();
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_cluster, centroid);
  // std::cout << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<
  //   centroid[2] << " " <<   centroid[3] << " \n";
  ROS_INFO("Centroid: %f %f %f", centroid[0], centroid[1], centroid[2]);
  end_ = ros::WallTime::now();
  // print results
  execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO("Centroid time (ms): %f", execution_time);

  // Extracting Params
  geometry_msgs::Point centroidPoint;
  centroidPoint.x = centroid[0];
  centroidPoint.y = centroid[1];
  centroidPoint.z = centroid[2];

  geometry_msgs::Point boundingBoxAA;
  boundingBoxAA.x = min_point_AABB.x;
  boundingBoxAA.y = min_point_AABB.y;
  boundingBoxAA.z = min_point_AABB.z;

  geometry_msgs::Point boundingBoxBB;
  boundingBoxBB.x = max_point_AABB.x;
  boundingBoxBB.y = max_point_AABB.y;
  boundingBoxBB.z = max_point_AABB.z;

  kinova_moveit::TargetObjectPcl targetParams;
  targetParams.centroid = centroidPoint;
  targetParams.boundingBoxAA = boundingBoxAA;
  targetParams.boundingBoxBB = boundingBoxBB;

  pubParams.publish(targetParams);

  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z, 1.0));
  boxFilter.setMax(Eigen::Vector4f(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z, 1.0));
  boxFilter.setNegative(true);
  boxFilter.setInputCloud(cloud_filtered);
  boxFilter.filter(*bodyFiltered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 cloud_inliers_pcl2;
  pcl::toPCLPointCloud2(*bodyFiltered, cloud_inliers_pcl2);
  pcl_conversions::fromPCL(cloud_inliers_pcl2, output);
  pub.publish(output);

  // std::cout << output.header.frame_id << std::endl;
  // ROS_INFO("output2.header.frame_id %s",output.header.frame_id);
  sensor_msgs::PointCloud2 output2;
  pcl::PCLPointCloud2 cloud_inliers_pcl2_2;
  pcl::toPCLPointCloud2(*cloud_cluster, cloud_inliers_pcl2_2);
  pcl_conversions::fromPCL(cloud_inliers_pcl2_2, output2);

  // Publish the data
  output2.header.frame_id = "camera_depth_optical_frame";
  pub1.publish(output2);
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate rate(0.1); // frequency of operation
  ROS_INFO("Node_started\n");
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe ("kinect/depth_registered/points", 1, cloud_cb);
  // ROS_INFO("Sub_started\n");
  //  Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_segmentation_output", 1);
  pub1 = nh.advertise<sensor_msgs::PointCloud2>("output_clusters", 1);
  pubParams = nh.advertise<kinova_moveit::TargetObjectPcl>("kinova_moveit/pcl_params", 1);

  ros::spin();
}
