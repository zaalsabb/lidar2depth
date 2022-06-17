// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
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
            ros::param::get("lidar2depth/depth_width", w);
            ros::param::get("lidar2depth/depth_height", h);
            ros::param::get("lidar2depth/frame_rate", frame_rate);
            ros::param::get("lidar2depth/tf_available", tf_available);

            ros::param::get( "r3live_vio/image_width", camera_w );
            ros::param::get( "r3live_vio/image_height", camera_h );
            ros::param::get( "r3live_vio/camera_intrinsic", K );
            ros::param::get( "r3live_vio/camera_ext_R", camera_ext_R );
            ros::param::get( "r3live_vio/camera_ext_t", camera_ext_t );            
            ros::param::get( "r3live_vio/camera_dist_coeffs", d );            

            // if (getCameraInfoFromYAML(camera_intrinsics)==1){
            //     ROS_ERROR("Cannot open file %s", camera_intrinsics.c_str());
            // }
            sub = nh_.subscribe ("lidar", 1, &Lidar2Depth::cloudCallback, this);
            sub2 = nh_.subscribe ("calib_rotation", 1, &Lidar2Depth::calib_rotation_callback, this);
            sub3 = nh_.subscribe ("calib_translation", 1, &Lidar2Depth::calib_translation_callback, this);
            sub4 = nh_.subscribe ("color", 1, &Lidar2Depth::imageCallback, this);

            image_transport::ImageTransport it(nh_);
            pub = it.advertise("depth", 1);   
            pub2 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);
            pub3 = it.advertise("color_sync", 1);   

            tf::TransformListener *listener = new tf::TransformListener(ros::Duration(10.0));

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

            // t_now = ros::Time::now();
            tf::Matrix3x3 rotation(
                camera_ext_R[0], camera_ext_R[1], camera_ext_R[2],
                camera_ext_R[3], camera_ext_R[4], camera_ext_R[5],
                camera_ext_R[6], camera_ext_R[7], camera_ext_R[8]);

            tf::Quaternion q;
            rotation.getRotation(q);
            transform_camera_lidar.setOrigin(tf::Vector3(camera_ext_t[0],camera_ext_t[1],camera_ext_t[2]));
            transform_camera_lidar.setRotation(q);     
            transform_camera_lidar = transform_camera_lidar.inverse();;       
            // std::cout << "(" << q.x() <<"," << q.y()<<"," << q.z()<<"," << q.w() << ")" << std::endl;
            // transform_camera_lidar.setOrigin(tf::Vector3(0,0,0));
            // transform_camera_lidar.setRotation(tf::Quaternion(0.5, -0.5, 0.5, 0.5));             

        };

        ros::NodeHandle nh_;

        string map_frame = "";
        string lidar_frame = "";
        string camera_frame = "";

        ros::Subscriber sub;
        ros::Subscriber sub2;
        ros::Subscriber sub3;
        ros::Subscriber sub4;

        ros::Publisher pub2;

        image_transport::Publisher pub;
        image_transport::Publisher pub3;

        tf::TransformListener listener;

        tf::StampedTransform transform_lidar_map;
        tf::Transform transform_camera_lidar;
        tf::StampedTransform transform_camera_map;

        int max_cloud_size = 300000;
        int ksize = 0;

        string camera_intrinsics;
        float fx, fy, cx, cy, camera_w, camera_h, h, w;
        std::vector< double > K, camera_ext_R, camera_ext_t;
        std::vector< double > d;

        cv::Mat camera_matrix;
        cv::Mat distortion_coefficients_mat;

        int frame_rate = 30;
        bool get_new_img = true;
        bool tf_available = true;
        ros::Time t_now;
        sensor_msgs::Image image_msg;

        void projectToDepth();
        void cullCloud();
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void imageCallback(const sensor_msgs::Image& image);
        void calib_rotation_callback(const geometry_msgs::Twist& rotation);
        void calib_translation_callback(const geometry_msgs::Twist& translation);
        int getCameraInfoFromYAML(string filename);

};
