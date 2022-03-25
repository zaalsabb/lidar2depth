// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
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
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
// #include <pcl/filters/voxel_grid.h> 

using namespace std;

class Lidar2Depth {

    public:

        Lidar2Depth(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;
            ros::param::get("lidar2depth/map_frame", map_frame);
            ros::param::get("lidar2depth/lidar_frame", lidar_frame);
            ros::param::get("lidar2depth/camera_frame", camera_frame);
            ros::param::get("lidar2depth/max_cloud_size", max_cloud_size);
            ros::param::get("lidar2depth/camera_intrinsics", camera_intrinsics);
            ros::param::get("lidar2depth/filter_kernel", ksize);
            ros::param::get("lidar2depth/image_width", w);
            ros::param::get("lidar2depth/image_height", h);
            ros::param::get("lidar2depth/frame_rate", frame_rate);

            if (getCameraInfoFromYAML(camera_intrinsics)==1){
                ROS_ERROR("Cannot open file %s", camera_intrinsics.c_str());
            }

            sub = nh_.subscribe ("lidar", 1, &Lidar2Depth::cloudCallback, this);

            image_transport::ImageTransport it(nh_);
            pub = it.advertise("depth", 1);   
            pub2 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);
            tf::TransformListener *listener = new tf::TransformListener(ros::Duration(10.0));

        };

        ros::NodeHandle nh_;

        string map_frame = "camera_init";
        string lidar_frame = "aft_mapped";
        string camera_frame = "camera_optical_frame";

        ros::Subscriber sub;
        ros::Publisher pub2;
        image_transport::Publisher pub;
        tf::TransformListener listener;

        tf::StampedTransform transform_map_lidar;
        tf::StampedTransform transform_camera_map;

        int max_cloud_size = 300000;
        int ksize = 0;

        string camera_intrinsics;
        float fx, fy, cx, cy, camera_w, camera_h, h, w;
        int frame_rate = 30;

        void projectToDepth();
        void cullCloud();
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        int getCameraInfoFromYAML(string filename);

};
