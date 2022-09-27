// standard cpp headers
#include <iostream>
#include <fstream>
// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
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

class Map2Color {

    public:

        Map2Color(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;

            ros::param::get("map2color/pcd_file", pcd_file);
            // ros::param::get("map2color/poses_file", poses_file);
            ros::param::get( "map2color/camera_frame", camera_frame );
            ros::param::get( "map2color/map_frame", map_frame );
            ros::param::get( "map2color/scalar_field", scalar_field );
            ros::param::get( "map2color/depth_rescale", depth_rescale );

            ros::param::get( "r3live_vio/image_width", w );
            ros::param::get( "r3live_vio/image_height", h );
            ros::param::get( "r3live_vio/camera_intrinsic", K );         
            ros::param::get( "r3live_vio/camera_dist_coeffs", d ); 

            tf::TransformListener *listener = new tf::TransformListener(ros::Duration(10.0));

            sub = nh_.subscribe ("image", 1, &Map2Color::imageCallback, this);

            std::stringstream ss;
            ss << "Data Retrieved: \n";
            std::copy(K.begin(), K.end(), std::ostream_iterator<double>(ss, " "));
            ss << std::endl;
            
            fx = K[0];
            fy = K[4];
            cx = K[2];
            cy = K[5];            

            distortion_coefficients_mat = (cv::Mat_<double>(5, 1) << d[0],d[1],d[2],d[3],d[4]);
            camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            
            pcd_out = pcd_file.substr(0, pcd_file.size()-4) + "_out.pcd";

            depth_w = (int)(w*depth_rescale);
            depth_h = (int)(h*depth_rescale);

        };

        ros::NodeHandle nh_;
        string pcd_file; 
        string pcd_out;

        string map_frame = "";
        string camera_frame = "";
        bool scalar_field = false;

        // string poses_file; 
        int i_img = 1;
        ros::Subscriber sub;

        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients_mat;

        float fx, fy, cx, cy, h, w;
        std::vector< double > K;
        std::vector< double > d;

        float depth_rescale = 1.0f;
        int depth_w;
        int depth_h;

        bool get_new_img = true;
        tf::StampedTransform transform_camera_map;
        tf::TransformListener listener;
        sensor_msgs::Image image_msg;
        cv::Mat img_undist;
        ros::Time t_stamp;
        
        void projectToDepth();
        void imageCallback(const sensor_msgs::Image& image);
        void postProcess();
        void saveMap();
        int median(vector<int> vec);

};
