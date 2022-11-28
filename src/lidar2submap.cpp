#include "lidar2submap.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

void Lidar2SubMap::cullCloud(float leaf_size){
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid; 
    voxelgrid.setInputCloud(cloud);
    voxelgrid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxelgrid.filter(*cloud);   
}

void Lidar2SubMap::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_lidar);

    *cloud += *temp_cloud_lidar;  
    i_scans++;

    if (i_scans == num_scans){
        double h_time = msg->header.stamp.toSec();
        timestamps.push_back(h_time);
        if (voxel_size_cloud > 0){
            cullCloud(voxel_size_cloud);
        }
        pcl::io::savePCDFileBinary(submaps_dir+"/"+to_string(cloud_id)+".pcd", *cloud);
        cout<<"saved scan "+to_string(cloud_id)<<endl;
        cloud_id++;
        cloud->clear();
        i_scans = 0;       
    }
}

void Lidar2SubMap::saveTimeStamps(){
    ofstream myfile;
    myfile.open(submaps_dir+"/timestamps.txt");
    for (int i=0; i<timestamps.size(); i++){
        myfile << to_string(timestamps[i]) + "\n";
    }
    myfile.close();
    cout<<"saved timestamps.txt"<<endl;
}


int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "lidar2submap");

    ros::NodeHandle nh;
    
    Lidar2SubMap lidar2submap(nh);
        
    while (ros::ok()) {
        ros::spinOnce();
    }    
    
    lidar2submap.saveTimeStamps();

    ros::spin();    
    return 0;

}
