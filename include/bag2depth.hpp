// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
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
#include <lidar2depth/GetPoses.h>
#include <lidar2depth/GetStamps.h>

#include <algorithm>
#include <vector>

#include <chrono>
#include <thread>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#define foreach BOOST_FOREACH

using namespace std;

class Bag2Depth {

    public:

        Bag2Depth(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;
            ros::param::get("bag2depth/max_cloud_size", max_cloud_size);
            ros::param::get("bag2depth/scans_dir", scans_dir);
            ros::param::get("bag2depth/depth_rescale", depth_rescale );
            ros::param::get("bag2depth/future_slider", future_slider );   
            ros::param::get("bag2depth/sleep_init", sleep_init);         
            ros::param::get("bag2depth/depth_dir", depth_dir);         

            ros::param::get( "r3live_vio/image_width", w );
            ros::param::get( "r3live_vio/image_height", h );
            ros::param::get( "r3live_vio/camera_intrinsic", K );
            ros::param::get( "r3live_vio/camera_ext_R", camera_ext_R );
            ros::param::get( "r3live_vio/camera_ext_t", camera_ext_t );            
            ros::param::get( "r3live_vio/camera_dist_coeffs", d );            

            if(boost::filesystem::exists(depth_dir))
            {
                boost::filesystem::remove_all(depth_dir);
            }
            boost::filesystem::create_directories(depth_dir);

            image_transport::ImageTransport it(nh_);
            pub = it.advertise("depth", 1);   

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

            depth_w = (int)(w*depth_rescale);
            depth_h = (int)(h*depth_rescale);

        };

        ros::NodeHandle nh_;

        image_transport::Publisher pub;

        int max_cloud_size = 300000;
        float depth_rescale = 1.0f;
        int depth_w;
        int depth_h;
        double future_slider = 0.5f;
        float sleep_init = 0.0f;

        // maximum depth allowed in uint16 image
        float max_depth = 65.536;

        float fx, fy, cx, cy, w, h;
        std::vector< double > K, camera_ext_R, camera_ext_t;
        std::vector< double > d;

        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients_mat;
        
        string scans_dir;
        string depth_dir;

        void projectToDepth(const geometry_msgs::Pose pose);
        void cullCloud();
        void cloudCallback (int k);
        void readBag();
        void readPoses();
        void readLidarStamps();
        void sleep_for_secs();

};

