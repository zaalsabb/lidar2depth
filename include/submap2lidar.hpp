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
#include <pcl/registration/icp.h>

using namespace std;

class SubMap2Lidar {

    public:

        SubMap2Lidar(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;

            ros::param::get("submap2lidar/save_directory", save_directory);
            ros::param::get("submap2lidar/frame_rate", frame_rate);

            submaps_dir = save_directory+"/submaps";           

            pub = nh_.advertise<sensor_msgs::PointCloud2>("submap", 1);

            out_dir = save_directory+"/out";           
            if(boost::filesystem::exists(out_dir))
            {
                boost::filesystem::remove_all(out_dir);
            }
            boost::filesystem::create_directories(out_dir);

        };

        ros::NodeHandle nh_;

        ros::Publisher pub;
        float frame_rate = 1;
        int cloud_id = 1;   
        string save_directory;
        string submaps_dir;  
        string out_dir;  
        string pcd_file;

        void publishSubmap();
        void perform_icp();
        void save_map();

};
