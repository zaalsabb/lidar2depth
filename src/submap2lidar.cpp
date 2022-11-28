#include "submap2lidar.hpp"

using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr submap (new pcl::PointCloud<pcl::PointXYZ>);   

void SubMap2Lidar::perform_icp(){

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;    
    icp.setInputCloud(submap);
    icp.setInputTarget(cloud_map);
    icp.setMaximumIterations (500);
    icp.setTransformationEpsilon (1e-9);
    icp.setMaxCorrespondenceDistance (0.05);
    icp.setEuclideanFitnessEpsilon (1);
    icp.setRANSACOutlierRejectionThreshold (1.5);    
    icp.align(*submap);

    // if (icp.hasConverged())
    // {
    std::cout << "ICP score is " << icp.getFitnessScore() << std::endl;
    // std::cout << "Transformation matrix:" << std::endl;
    // std::cout << icp.getFinalTransformation() << std::endl;
    
    // save_map(); 
    // if (icp.getFitnessScore() < 15.0f){
    //     save_map();            
    // } else {
    //     std::cout << "ICP result rejected" << std::endl;
    // }
    // }

}

void SubMap2Lidar::save_map(){
    string pcd_file_out = out_dir+"/"+to_string(cloud_id)+".pcd";    
    // pcl::io::savePCDFileBinary(pcd_file_out, *cloud_map);
    // prev_cloud->points.resize(cloud_map->points.size());
    // for (int i=0; i<cloud_map->points.size(); i++){
    //     prev_cloud->points[i].x = cloud_map->points[i].x;
    //     prev_cloud->points[i].y = cloud_map->points[i].y;
    //     prev_cloud->points[i].z = cloud_map->points[i].z;
    // }
}

void SubMap2Lidar::publishSubmap ()
{
    pcl::io::loadPCDFile(pcd_file, *submap);

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid; 
    voxelgrid.setInputCloud(submap);
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelgrid.filter(*submap);

    if (cloud_id > 1){
        perform_icp();
    }

    *cloud_map += *submap;
    voxelgrid.setInputCloud(cloud_map);
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelgrid.filter(*cloud_map);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_map, msg);
    msg.header.frame_id = "world";

    pub.publish(msg);
    cout<<"published submap "+to_string(cloud_id)<<endl;

}


int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "submap2lidar");

    ros::NodeHandle nh;
    
    SubMap2Lidar submap2lidar(nh);
    ros::Rate rate(submap2lidar.frame_rate);
    while (nh.ok()){        
        submap2lidar.pcd_file = submap2lidar.submaps_dir+"/"+to_string(submap2lidar.cloud_id)+".pcd";
        if (boost::filesystem::exists(submap2lidar.pcd_file)){
            submap2lidar.publishSubmap();
            submap2lidar.cloud_id++;
        } else {
            cout<<"finished pubslishing submaps.."<<endl;
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }

    ros::spin();    
    return 0;

}
