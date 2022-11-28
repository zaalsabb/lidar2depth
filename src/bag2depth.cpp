#include "bag2depth.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);
vector<double> camera_stamps;
vector<geometry_msgs::PoseStamped> camera_poses;
vector<double> lidar_stamps;
int curr_pose_index=0;

void Bag2Depth::cullCloud(){
    if (cloud_map->points.size() > max_cloud_size)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        reverse(camera_stamps.begin(),camera_stamps.end()); // first becomes last, reverses the vector

        for (int i = 0; i < cloud_map->points.size()-max_cloud_size; i++)
        {
            inliers->indices.push_back(i);
            camera_stamps.pop_back(); // pop last
        }

        extract.setInputCloud(cloud_map);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_map);               

        reverse(camera_stamps.begin(),camera_stamps.end()); // reverses it again, so the elements are in the same order as before
    }
}

void Bag2Depth::projectToDepth(const geometry_msgs::Pose pose){

    if (cloud_map->points.size()==0){
        return;
    }

    // std::cout << to_string(pose.orientation.x) << endl; 
    
    tf::Quaternion q = tf::Quaternion(pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z,
                                      pose.orientation.w);

    tf::Vector3 t = tf::Vector3(pose.position.x,
                                pose.position.y,
                                pose.position.z);

    tf::Transform transform_map_camera;
    transform_map_camera.setOrigin(t);    
    transform_map_camera.setRotation(q);    
    tf::Transform transform_camera_map = transform_map_camera.inverse();

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*cloud_map, *temp_cloud_camera, transform_camera_map);

    cv::Mat depth_image(cv::Size(depth_w,depth_h),CV_16UC1,cv::Scalar(0));
    for (int i=0; i<temp_cloud_camera->points.size(); i++){
        float Z = temp_cloud_camera->points[i].z;
        float X = temp_cloud_camera->points[i].x;
        float Y = temp_cloud_camera->points[i].y;
        float u = (fx*X/Z + cx)*(float)depth_w/(float)w;
        float v = (fy*Y/Z + cy)*(float)depth_h/(float)h;
        if (u>0 && u<depth_image.cols && v>0 && v<depth_image.rows){
            float Zi = depth_image.at<unsigned short>(v,u);
            if (Z>0 & (Z<Zi || Zi==0)){
                depth_image.at<unsigned short>(v,u) = Z*1000;
            }            
        }
    }

    cv::resize(depth_image, depth_image, cv::Size(w, h), cv::INTER_NEAREST);

    // ros::Time time = ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image).toImageMsg();    

    pub.publish(msg);

}

void Bag2Depth::cloudCallback (int k)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_lidar (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(scans_dir+"/"+to_string(k+1)+".pcd", *current_scan_lidar);

    *cloud_map += *current_scan_lidar; 
    double scan_time = lidar_stamps[k];

    for (size_t i = 0; i < current_scan_lidar->points.size(); i++) {
        camera_stamps.push_back(scan_time);
    }

    cullCloud();

}


void Bag2Depth::readBag(){

    for (int k = 0; k<lidar_stamps.size(); k++)
    {

        // add current scan to cloud
        cloudCallback(k);
        
        // find scans time
        double scan_time = camera_stamps[0]*future_slider + camera_stamps[camera_stamps.size()-1]*(1-future_slider);
        
        // find closest pose time to scans time
        double curr_pose_time = camera_poses[curr_pose_index].header.stamp.toSec();            
        if (scan_time > curr_pose_time){
            std::cout << "\nGenerating depth image " + to_string(curr_pose_index+1) << endl;
            projectToDepth(camera_poses[curr_pose_index].pose);
            curr_pose_index++;
        } 
        // std::cout << "lidar scans slider time:"+ to_string(scan_time) + ", depth image time:" +  to_string(curr_pose_time) << endl; 
    }

    // generating remaining poses after all lidar scans were processed
    while (curr_pose_index < camera_poses.size()+1){
        std::cout << "\nGenerating depth image " + to_string(curr_pose_index+1) << endl;
        projectToDepth(camera_poses[curr_pose_index].pose);
        curr_pose_index++;
    }

}

void Bag2Depth::readPoses(){

    ros::ServiceClient client = nh_.serviceClient<lidar2depth::GetPoses>("get_poses");
    client.waitForExistence();
    lidar2depth::GetPoses srv;
    client.call(srv);

    for (int i=0; i < srv.response.poses.size(); i++){
        camera_poses.push_back(srv.response.poses[i]);
    }
    ROS_INFO("Finished reading poses\n");

}

void Bag2Depth::readLidarStamps(){

    ros::ServiceClient client = nh_.serviceClient<lidar2depth::GetStamps>("get_timestamps");
    client.waitForExistence();
    lidar2depth::GetStamps srv;
    client.call(srv);

    for (int i=0; i < srv.response.timestamps.size(); i++){
        lidar_stamps.push_back(srv.response.timestamps[i].data.toSec());
    }
    ROS_INFO("Finished reading lidar timestamps\n");

}

int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "bag2depth");

    ros::NodeHandle nh;

    Bag2Depth bag2depth(nh);
    bag2depth.readPoses();
    bag2depth.readLidarStamps();    
    bag2depth.readBag();

    // ros::spin();
    
    return 0;

}
