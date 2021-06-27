#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
// #include <pcl/common/transforms.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <numeric>
#include <eigen3/Eigen/Dense>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double closeness_tol = 0.0001;

boost::shared_ptr<bool> Lup(new bool), Rup(new bool);

// start with a bogus tf, i will make this a saved and configurable thing later
Eigen::Matrix4f l_r_transform = Eigen::Matrix4f::Identity();


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> l_pc(new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> r_pc(new pcl::PointCloud<pcl::PointXYZ>);


// listen to l lidar
void l_callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    if(*Lup == 0){
        pcl::fromROSMsg( *source_cloud, *l_pc );
        *Lup = 1;
    }
}

// listen to r lidar
void r_callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    if(*Rup == 0){
        pcl::fromROSMsg(*source_cloud, *r_pc);
        *Rup = 1;
    }
}


double closeness(){
    float dist;
    // may have to change this
    float min = 100;
    int index = 0;
    std::vector<float> dists;
    // std::vector<int> indicies;
    for(int i = 0; i < l_pc->points.size(); i++){
        for(int j = 0; j < r_pc->points.size(); j++){
            dist = pcl::squaredEuclideanDistance(l_pc->points[i], r_pc->points[j]);
            if(dist < min){
                min = dist;
                index = j;
            }
        }
        // indicies.push_back(index);
        dists.push_back(min);
    }

    double mean_error = std::accumulate(dists.begin(), dists.end() ,0.0) / dists.size();
    

    return mean_error;
}


int main(int argc, char** argv){
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    // initialize "locks"
    *Lup = 0;
    *Rup = 0;

    // TODO: load transform from file

    // unified publisher
    ros::Publisher unified_publisher = nh.advertise<sensor_msgs::PointCloud2>("pc_unified", 1);
    // subscribe to front left
    ros::Subscriber lsub = nh.subscribe<sensor_msgs::PointCloud2>("/VelodyneFrontLeft", 1, l_callback);
    // subscribe to front right
    ros::Subscriber rsub = nh.subscribe<sensor_msgs::PointCloud2>("/VelodyneFrontRight", 1, r_callback);

    ros::Rate loop_rate(100);

    while(nh.ok()){
        // if both point clouds have been updated then run the combination
        if(*Lup && *Rup){
            // create final pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
            
            // stuff l pointcloud into final one
            combined_cloud = l_pc;

            // Transform right cloud to left
            pcl::transformPointCloud (*l_pc, *r_pc, l_r_transform);

            // check alignment of the pointclouds
            if(closeness() > closeness_tol){
                std::cout << "alignment changed" << std::endl;
                //bad alignment, do icp again -> assume that the tf is alright
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setInputSource(r_pc);
                icp.setInputTarget(l_pc);
                pcl::PointCloud<pcl::PointXYZ> r_pc_aligned;
                icp.align(r_pc_aligned);
                *combined_cloud += r_pc_aligned;
                l_r_transform = icp.getFinalTransformation();
                // TODO: save transform to file
            }else{
                // previous alignment is good, use that
                *combined_cloud += *r_pc;
            }

            // convert back to pointcloud2 sensor msg
            sensor_msgs::PointCloud2 combo_cloud;
            pcl::toROSMsg(*combined_cloud, combo_cloud);

            // publish combined and aligned cloud
            unified_publisher.publish(combo_cloud);
            // std::cout << "published" << std::endl;
            *Lup = 0;
            *Rup = 0;
        }
        /*else{
            // std::cout << "not ready" << std::endl;
        }*/
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
