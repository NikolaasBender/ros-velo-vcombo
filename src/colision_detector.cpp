#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PtrCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBPtrCloud;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr NormPtrCloud;

double danger_dist, left_edge, right_edge, bottom_edge, top_edge, min_dist;

boost::shared_ptr<sensor_msgs::PointCloud2> ros_cloud_in (new sensor_msgs::PointCloud2());

// listen to l lidar
void callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    *ros_cloud_in = *source_cloud;
}

void slice_and_color(RGBPtrCloud source_cloud, RGBPtrCloud segmented_cloud, ros::Time timestamp){
    for(int i = 0; i < source_cloud->points.size(); i++){
        source_cloud->points[i].r = 255;
        source_cloud->points[i].g = 255;
        source_cloud->points[i].b = 255;
        if(source_cloud->points[i].x > left_edge &&
            source_cloud->points[i].x < right_edge &&
            source_cloud->points[i].z > bottom_edge &&
            source_cloud->points[i].z < top_edge){
                segmented_cloud->points.push_back(source_cloud->points[i]);
                if(source_cloud->points[i].y <= danger_dist && source_cloud->points[i].y > min_dist){
                    source_cloud->points[i].r = 255;
                    source_cloud->points[i].g = 0;
                    source_cloud->points[i].b = 0;
                    segmented_cloud->points[0].r = 255;
                    segmented_cloud->points[0].g = 0;
                    segmented_cloud->points[0].b = 0;
                    double dist = pcl::squaredEuclideanDistance(source_cloud->points[i], pcl::PointXYZ(0, 0, 0));
                    // write to csv
                    std::ofstream file;
                    file.open("/home/nick/dangers.csv", std::ios::app);
                    if(file.is_open()){
                        // std::cout << "file open" << std::endl;
                        file << std::to_string(timestamp.toSec()) << ",";
                        file << std::to_string(source_cloud->points[i].x) << ","; 
                        file << std::to_string(source_cloud->points[i].y) << ",";
                        file << std::to_string(source_cloud->points[i].z) << ",";
                        file << std::to_string(dist) << "\n";
                        file.close();
                    }else{
                        std::cout << "file not open" << std::endl;
                    }
                    
                }
            }
    }
}

int main(int argc, char** argv){
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;

    std::cout << "powered by crayola" << std::endl;

    std::string pc_topic, lidar_name;

    nh.getParam("top_edge", top_edge);
    nh.getParam("bottom_edge", bottom_edge);
    nh.getParam("left_edge", left_edge);
    nh.getParam("right_edge", right_edge);
    nh.getParam("danger_distance", danger_dist);
    nh.getParam("cloud_topic", pc_topic);
    nh.getParam("lidar_name", lidar_name);
    nh.getParam("minimum_distance", min_dist);


    // TODO: load transform from file

    // unified publisher
    ros::Publisher color_publisher = nh.advertise<sensor_msgs::PointCloud2>(lidar_name + "_pc_colored", 1);
    ros::Publisher danger_publisher = nh.advertise<sensor_msgs::PointCloud2>(lidar_name + "_pc_danger_zone", 1);

    // subscribe to front left
    ros::Subscriber lsub = nh.subscribe<sensor_msgs::PointCloud2>(pc_topic, 1, callback);

    RGBPtrCloud pc (new pcl::PointCloud<pcl::PointXYZRGB>());
    RGBPtrCloud danger_pc (new pcl::PointCloud<pcl::PointXYZRGB>());

    ros::Rate loop_rate(100);

    while(nh.ok()){
        pcl::fromROSMsg( *ros_cloud_in, *pc );

        slice_and_color(pc, danger_pc, ros_cloud_in->header.stamp);

        // convert back to pointcloud2 sensor msg
        sensor_msgs::PointCloud2 rgb_full_cloud;
        pcl::toROSMsg(*pc, rgb_full_cloud);
        rgb_full_cloud.header.stamp = ros::Time::now();
        rgb_full_cloud.header.frame_id = "vehicle";

        sensor_msgs::PointCloud2 danger_cloud;
        pcl::toROSMsg(*danger_pc, danger_cloud);
        danger_cloud.header.stamp = ros::Time::now();
        danger_cloud.header.frame_id = "vehicle";

        // publish combined and aligned cloud
        color_publisher.publish(rgb_full_cloud);
        danger_publisher.publish(danger_cloud);
        // std::cout << "published" << std::endl;
        danger_pc->clear();
        pc->clear();

        ros::spinOnce();
        loop_rate.sleep();
    }
    danger_pc->clear();
    pc->clear();
    ros_cloud_in.reset();
}
