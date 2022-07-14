#include "lidar2mesh.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


void Lidar2Mesh::cullCloud(float leaf_size){
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid; 
    voxelgrid.setInputCloud(cloud);
    voxelgrid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxelgrid.filter(*cloud);   
}

void Lidar2Mesh::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_lidar(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_lidar);

    *cloud += *temp_cloud_lidar;  
    i_scans++;

    if (i_scans == num_scans){
        // cout<<i_scans<<endl;
        cullCloud(voxel_size_cloud);
        pcl::io::savePCDFileBinary(save_directory+"/cloud"+to_string(cloud_id)+".pcd", *cloud);
        cout<<"saved submap"<<endl;
        cloud_id++;
        // cullCloud(voxel_size_mesh);
        // publishMesh();
        cloud->clear();
        i_scans = 0;
    }

}

void Lidar2Mesh::publishMesh(){
    if (cloud->points.size()==0){
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->points.resize(cloud->size());
    for (size_t i = 0; i < cloud_xyz->points.size(); i++) {
        cloud_xyz->points[i].x = cloud->points[i].x;
        cloud_xyz->points[i].y = cloud->points[i].y;
        cloud_xyz->points[i].z = cloud->points[i].z;
    }

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_xyz);
    n.setInputCloud (cloud_xyz);
    n.setSearchMethod (tree);
    n.setKSearch (10);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_xyz, *normals, *cloud_with_normals);
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
    msg_mesh.triangles.resize(3*triangles.polygons.size());
    for (int i=0; i<triangles.polygons.size(); i++){
        msg_mesh.triangles[i*3    ] = triangles.polygons[i].vertices[0];
        msg_mesh.triangles[i*3 + 1] = triangles.polygons[i].vertices[1];
        msg_mesh.triangles[i*3 + 2] = triangles.polygons[i].vertices[2];
    }
    msg_mesh.vertices.resize(cloud->points.size());
    for (int i=0; i<cloud->points.size(); i++){
        msg_mesh.vertices[i].x = cloud->points[i].x;
        msg_mesh.vertices[i].y = cloud->points[i].y;
        msg_mesh.vertices[i].z = cloud->points[i].z;        
    }
    pub.publish(msg_mesh);
    pcl::io::save(save_directory+"/mesh.ply", triangles);
    cloud_xyz->clear();
}


bool Lidar2Mesh::getMapService(lidar2depth::GetMap::Request& req, lidar2depth::GetMap::Response& res){

    cout<<"Get Map Service Called.."<<endl;        

    if (cloud_id == 0){
        cout<<"No submaps have been saved yet.."<<endl;        
        return true;
    }

    res.submaps.resize(1);

    cloud->clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_lidar(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < cloud_id; i++){
        pcl::io::loadPCDFile(save_directory+"/cloud"+to_string(i)+".pcd", *temp_cloud_lidar);
        *cloud += *temp_cloud_lidar;  
    }
    cullCloud(voxel_size_cloud);
    pcl::io::savePCDFileBinary(save_directory+"/cloud.pcd", *cloud);

    pcl::toROSMsg(*cloud.get(),res.submaps[0].pointcloud );
    cullCloud(voxel_size_mesh);
    publishMesh();
        
    pcl::PolygonMesh triangles;
    pcl::io::loadPLYFile(save_directory+"/mesh.ply", triangles);
    res.submaps[0].mesh.triangles.resize(3*triangles.polygons.size());
    for (int j=0; j<triangles.polygons.size(); j++){
        res.submaps[0].mesh.triangles[j*3    ] = triangles.polygons[j].vertices[0];
        res.submaps[0].mesh.triangles[j*3 + 1] = triangles.polygons[j].vertices[1];
        res.submaps[0].mesh.triangles[j*3 + 2] = triangles.polygons[j].vertices[2];
    }
    pcl::PointCloud<pcl::PointXYZ> verts;
    pcl::fromPCLPointCloud2 (triangles.cloud, verts);
    res.submaps[0].mesh.vertices.resize(verts.points.size());
    for (int j=0; j<verts.points.size(); j++){
        res.submaps[0].mesh.vertices[j].x = verts.points[j].x;
        res.submaps[0].mesh.vertices[j].y = verts.points[j].y;
        res.submaps[0].mesh.vertices[j].z = verts.points[j].z;
    }
    cout<<"Meshing Finished. Sending Map..."<<endl;        
    return true;
}

int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "lidar2mesh");

    ros::NodeHandle nh;

    Lidar2Mesh lidar2mesh(nh);

    ros::spin();
    return 0;

}
