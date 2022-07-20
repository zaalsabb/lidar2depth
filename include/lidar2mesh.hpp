// standard cpp headers
#include <iostream>
#include <fstream>
// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <lidar2depth/Mesh.h>
#include <lidar2depth/Submap.h>
#include <lidar2depth/GetMap.h>
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

class Lidar2Mesh {

    public:

        Lidar2Mesh(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;

            ros::param::get("lidar2mesh/max_cloud_size", max_cloud_size);
            ros::param::get("lidar2mesh/num_scans", num_scans);
            ros::param::get("lidar2mesh/voxel_size_mesh", voxel_size_mesh);
            ros::param::get("lidar2mesh/voxel_size_cloud", voxel_size_cloud);
            ros::param::get("lidar2mesh/search_radius", search_radius);
            ros::param::get("lidar2mesh/max_nearest_neighbour", max_nearest_neighbour);
            ros::param::get("lidar2mesh/save_directory", save_directory);

            sub = nh_.subscribe ("lidar", 1, &Lidar2Mesh::cloudCallback, this);
            pub = nh_.advertise<lidar2depth::Mesh>("mesh", 1);

            srv = nh_.advertiseService("get_map", &Lidar2Mesh::getMapService, this);            

        };

        ros::NodeHandle nh_;

        ros::Subscriber sub;
        ros::Publisher pub;
        ros::ServiceServer srv;
        int max_cloud_size = 300000;
        int num_scans = 5;
        int i_scans = 0;
        float voxel_size_mesh = 0.1f;
        float voxel_size_cloud = 0.01f;
        float search_radius = 2.0f; 
        int max_nearest_neighbour = 100; 
        int cloud_id = 0;   
        string save_directory;    
        lidar2depth::Mesh msg_mesh;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void publishMesh();
        void cullCloud(float leaf_size);
        bool getMapService(lidar2depth::GetMap::Request& req, lidar2depth::GetMap::Response& res);
        int  getNumFiles (string fdir);
        void saveNumFiles (string fdir);
};
