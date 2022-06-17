#include "lidar2mesh.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


void Lidar2Mesh::cullCloud(){
    if (cloud->points.size() > max_cloud_size)
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (int i = 0; i < cloud->points.size()-max_cloud_size; i++)
        {
            inliers->indices.push_back(i);
        }

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);

        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid; 
        voxelgrid.setInputCloud(cloud);
        voxelgrid.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxelgrid.filter(*cloud);        
    }
}

void Lidar2Mesh::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_lidar);

    *cloud += *temp_cloud_lidar;  
    i_scans++;

    if (i_scans == num_scans){
        cout<<i_scans<<endl;
        cullCloud();
        publishMesh();
        cloud->clear();
        i_scans = 0;
    }

    

}

void Lidar2Mesh::publishMesh(){
    if (cloud->points.size()==0){
        return;
    }
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (10);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (search_radius);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (50);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    msg_mesh = lidar2depth::Mesh(); 
    for (int i=0; i<triangles.polygons.size(); i++){
        msg_mesh.triangles.push_back(triangles.polygons[i].vertices[0]);
        msg_mesh.triangles.push_back(triangles.polygons[i].vertices[1]);
        msg_mesh.triangles.push_back(triangles.polygons[i].vertices[2]);
    }

    for (int i=0; i<cloud->points.size(); i++){
        geometry_msgs::Point p = geometry_msgs::Point();
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;        
        msg_mesh.vertices.push_back(p);
    }

    pub.publish(msg_mesh);
}

int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "lidar2mesh");

    ros::NodeHandle nh;

    Lidar2Mesh lidar2mesh(nh);
    // while (nh.ok()){

    //     sleep(lidar2mesh.wait_period);

    //     lidar2mesh.publishMesh();

    //     ros::spinOnce();
    // }
    ros::spin();
    return 0;

}
