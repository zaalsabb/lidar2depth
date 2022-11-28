// standard cpp headers
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// opencv header files
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// pcl header files
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/auto_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h> 

using namespace std;

class Lidar2SubMap {

    public:

        Lidar2SubMap(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;

            ros::param::get("lidar2submap/num_scans", num_scans);
            ros::param::get("lidar2submap/voxel_size_cloud", voxel_size_cloud);
            ros::param::get("lidar2submap/submaps_dir", submaps_dir);

            if(boost::filesystem::exists(submaps_dir))
            {
                boost::filesystem::remove_all(submaps_dir);
            }
            boost::filesystem::create_directories(submaps_dir);

            sub = nh_.subscribe ("lidar", 1, &Lidar2SubMap::cloudCallback, this);

        };

        ros::NodeHandle nh_;

        ros::Subscriber sub;
        int num_scans = 100;
        int i_scans = 0;
        float voxel_size_cloud = 0.0f;
        int cloud_id = 1;   
        string submaps_dir;    
        vector<double> timestamps;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void cullCloud(float leaf_size);
        void saveTimeStamps();
};
