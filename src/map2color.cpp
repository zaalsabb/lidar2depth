#include "map2color.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZRGB>);
vector<float> cloud_depth_buffer;
vector<vector<int> > cloud_pixel_b;
vector<vector<int> > cloud_pixel_g;
vector<vector<int> > cloud_pixel_r;

void Map2Color::projectToDepth(){
    try{            
        listener.lookupTransform(camera_frame, map_frame, t_stamp, transform_camera_map);
    }
    catch (tf::TransformException ex){
        // std::cout << "cannot get tf frame" << endl;      
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_camera(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*cloud_map, *temp_cloud_camera, transform_camera_map);

    // build depth buffer
    cv::Mat depth_image(cv::Size(depth_w,depth_h),CV_64F,cv::Scalar(0));
    for (int i=0; i<temp_cloud_camera->points.size(); i++){
        float Z = temp_cloud_camera->points[i].z;
        float X = temp_cloud_camera->points[i].x;
        float Y = temp_cloud_camera->points[i].y;
        float u = (fx*X/Z + cx)*(float)depth_w/(float)w;
        float v = (fy*Y/Z + cy)*(float)depth_h/(float)h;
        if (u>0 && u<depth_image.cols && v>0 && v<depth_image.rows){
            float Zi = depth_image.at<float>(v,u);
            // use depth buffer to prevent
            if (Z>0 && (Z<Zi || Zi==0)){
                depth_image.at<float>(v,u) = Z;
            }
        }
    }

    // colorize point cloud map based on pixel values
    for (int i=0; i<temp_cloud_camera->points.size(); i++){
        float Z = temp_cloud_camera->points[i].z;
        float X = temp_cloud_camera->points[i].x;
        float Y = temp_cloud_camera->points[i].y;
        float u = (fx*X/Z + cx)*(float)depth_w/(float)w;
        float v = (fy*Y/Z + cy)*(float)depth_h/(float)h;
        if (u>0 && u<depth_image.cols && v>0 && v<depth_image.rows){
            float Zi = depth_image.at<float>(v,u);
            // use depth buffer to only colorize points with lowest depth values
            // if (Z==Zi){
            //     // colorize point with image that is closest to the point
            //     if (cloud_depth_buffer[i] == 0.0f || Z <= cloud_depth_buffer[i]){
            //         float u = fx*X/Z + cx;
            //         float v = fy*Y/Z + cy;                
            //         cv::Vec3b bgrPixel = img_undist.at<cv::Vec3b>(v, u);                    
            //         cloud_map->points[i].b = bgrPixel.val[0];
            //         cloud_map->points[i].g = bgrPixel.val[1];
            //         cloud_map->points[i].r = bgrPixel.val[2];
            //         cloud_depth_buffer[i] = Z;
            //     }                  
            // } 
            if (Z==Zi){
                // colorize point with image that is closest to the point
                float u = fx*X/Z + cx;
                float v = fy*Y/Z + cy;                
                cv::Vec3b bgrPixel = img_undist.at<cv::Vec3b>(v, u);                    
                cloud_pixel_b[i].push_back(bgrPixel.val[0]);
                cloud_pixel_g[i].push_back(bgrPixel.val[1]);
                cloud_pixel_r[i].push_back(bgrPixel.val[2]);

                cloud_map->points[i].b = median(cloud_pixel_b[i]);
                cloud_map->points[i].g = median(cloud_pixel_g[i]);
                cloud_map->points[i].r = median(cloud_pixel_r[i]);
                
                cloud_depth_buffer[i] = Z;
            }                         
        }
    }
}

int Map2Color::median(vector<int> vec)
{
    sort(vec.begin(), vec.end());
    return vec.at(floor(vec.size() / 2));
}

void Map2Color::imageCallback (const sensor_msgs::Image& msg)
{    
    if (get_new_img){
        get_new_img = false;
        std::cout << "colorizing pointcloud using image " + to_string(i_img) << endl;      
        cv::Mat img = cv_bridge::toCvCopy(msg, msg.encoding)->image;        
        cv::undistort(img, img_undist, camera_matrix, distortion_coefficients_mat);
        t_stamp = msg.header.stamp;
        projectToDepth();
        i_img++;
        get_new_img = true;
    }
}

void Map2Color::postProcess(){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    for (size_t i = 0; i < cloud_map->points.size(); i++) {
        if (cloud_depth_buffer[i] == 0.0f){
            inliers->indices.push_back(i);
        }
        // short red-green-blue colormap (https://www.particleincell.com/2014/colormap/)
        if (scalar_field){
            float s = (cloud_map->points[i].b + cloud_map->points[i].g + cloud_map->points[i].r)/3;
            s = s/255.0f;

            float a=(1-s)*3;	//invert and group
            int X=floor(a);	//this is the integer part
            int Y=floor(255*(a-X)); //fractional part from 0 to 255
            int r; int g; int b;
            switch(X)
            {
                case 0: r=255;   g=Y;     b=0;   break;
                case 1: r=255-Y; g=255;   b=0;   break;
                case 2: r=0;     g=255-Y; b=Y;   break;
                case 3: r=0;     g=0;     b=255; break;
            }

            cloud_map->points[i].b = b;
            cloud_map->points[i].g = g;
            cloud_map->points[i].r = r;
        }
    }
    extract.setInputCloud(cloud_map);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_map);
}

void Map2Color::saveMap(){

    pcl::io::savePCDFileBinary(pcd_out, *cloud_map);
}

int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "map2color");
    // std::cout << "test" << endl;

    ros::NodeHandle nh;

    Map2Color map2color(nh);
    // std::cout << "test2" << endl;
    pcl::io::loadPCDFile(map2color.pcd_file, *cloud_map);
    for (size_t i = 0; i < cloud_map->points.size(); i++) {
        cloud_depth_buffer.push_back(0);
        
        vector<int> pixel_temp;
        cloud_pixel_b.push_back(pixel_temp);
        cloud_pixel_g.push_back(pixel_temp);
        cloud_pixel_r.push_back(pixel_temp);
    }

    cout << "Press Ctrl+C to save pcd file..." <<endl;

    while (ros::ok()) {
        ros::spinOnce();
    }
    
    map2color.postProcess();
    map2color.saveMap();
    cout << "pcd file saved!" <<endl;

    ros::spin();
    return 0;

}