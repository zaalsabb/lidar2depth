#include "lidar2depth.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);

void Lidar2Depth::cullCloud(){
    if (cloud_map->points.size() > max_cloud_size)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (int i = 0; i < cloud_map->points.size()-max_cloud_size; i++)
        {
            inliers->indices.push_back(i);
        }

        extract.setInputCloud(cloud_map);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_map);
    }
}

void Lidar2Depth::projectToDepth(){

    try{
        listener.lookupTransform(camera_frame, map_frame, ros::Time(), transform_camera_map);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*cloud_map, *temp_cloud_camera, transform_camera_map);
    cv::Mat depth_image(cv::Size(w,h),CV_16UC1,cv::Scalar(0));
    for (int i=0; i<temp_cloud_camera->points.size(); i++){
        float Z = temp_cloud_camera->points[i].z;
        float X = temp_cloud_camera->points[i].x;
        float Y = temp_cloud_camera->points[i].y;
        float u = fx*X/Z + cx;
        float v = fy*Y/Z + cy;
        if (u>0 && u<depth_image.cols && v>0 && v<depth_image.rows){
            float Zi = depth_image.at<unsigned short>(v,u);
            if (Z>0 & (Z<Zi || Zi==0)){
                depth_image.at<unsigned short>(v,u) = Z*1000;
            }            
        }
    }
    if (ksize > 0){
        if (ksize%2 == 0){
            ksize += 1;
        }
        if (ksize == 3 || ksize == 5){
            depth_image.convertTo(depth_image, CV_32F);
        } else {
            depth_image.convertTo(depth_image, CV_8U);
        }

        cv::medianBlur(depth_image,depth_image,ksize);
        depth_image.convertTo(depth_image, CV_16UC1);  
    }  

    ros::Time time = ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image).toImageMsg();    

    pub.publish(msg);

    cullCloud();

}

void Lidar2Depth::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    try{
        listener.lookupTransform(map_frame, lidar_frame, ros::Time(), transform_map_lidar);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_map(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_lidar);

    pcl_ros::transformPointCloud(*temp_cloud_lidar, *temp_cloud_map, transform_map_lidar);

    *cloud_map += *temp_cloud_map;    

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_map,cloud_publish);
    cloud_publish.header = msg->header;
    cloud_publish.header.frame_id = map_frame;

    pub2.publish(cloud_publish);
}

int Lidar2Depth::getCameraInfoFromYAML(string filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return 1;
    }

    cv::Mat K;
    fs["camera_matrix"] >> K;    
    fs["image_height"] >> h;    
    fs["image_width"] >> w;    

    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);

    return 0;

}

int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "lidar2depth");

    ros::NodeHandle nh;

    Lidar2Depth lidar2depth(nh);

    ros::Rate rate(30.0);
    while (nh.ok()){

        lidar2depth.projectToDepth();

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;

}
