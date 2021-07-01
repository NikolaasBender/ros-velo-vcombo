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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <cmath>
#include <pcl/features/normal_3d.h>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PtrCloud;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr NormPtrCloud;

double closeness_tol;
double cone_of_shame;

boost::shared_ptr<bool> Lup(new bool), Rup(new bool);

// start with a bogus tf, i will make this a saved and configurable thing later
Eigen::Matrix4f l_r_transform = Eigen::Matrix4f::Identity();


PtrCloud l_pc(new pcl::PointCloud<pcl::PointXYZ>);
PtrCloud r_pc(new pcl::PointCloud<pcl::PointXYZ>);


// listen to l lidar
void l_callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    if(*Lup == 0){
        l_pc->clear();
        pcl::fromROSMsg( *source_cloud, *l_pc );
        *Lup = 1;
    }
}

// listen to r lidar
void r_callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    if(*Rup == 0){
        r_pc->clear();
        pcl::fromROSMsg(*source_cloud, *r_pc);
        *Rup = 1;
    }
}


double closeness(PtrCloud cloud1, PtrCloud cloud2){
    std::cout << "checking closeness" << std::endl;
    float dist;
    // may have to change this
    float min = 100;
    int index = 0;
    std::vector<float> dists;
    // std::vector<int> indicies;
    for(int i = 0; i < cloud1->points.size(); i++){
        for(int j = 0; j < cloud2->points.size(); j++){
            dist = pcl::squaredEuclideanDistance(cloud1->points[i], cloud2->points[j]);
            if(dist < min){
                min = dist;
                index = j;
            }
        }
        // indicies.push_back(index);
        dists.push_back(min);
    }

    double mean_error = std::accumulate(dists.begin(), dists.end() ,0.0) / dists.size();

    std::cout << "closeness" << mean_error << std::endl;

    return mean_error;
}

void forward_slice(PtrCloud source_cloud, PtrCloud segmented_cloud){
    for(int i = 0; i < source_cloud->points.size(); i++){
        double theta = atan2(abs(source_cloud->points[i].x), abs(source_cloud->points[i].y));
        // if(theta <= cone_of_shame){
        //     segmented_cloud->points.push_back(source_cloud->points[i]);
        // }
        // filter only points within 30m
        if(theta <= cone_of_shame && source_cloud->points[i].y < 40){
            segmented_cloud->points.push_back(source_cloud->points[i]);
        }
    }
}


void normal_computation(PtrCloud source_cloud, NormPtrCloud target){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(source_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 30cm
    ne.setRadiusSearch(0.3);

    // Compute the features
    ne.compute(*target);
}


// this is a terrible naming scheme
void normal_icp(PtrCloud left_cloud, PtrCloud right_cloud){
    // ROS_INFO("aligning");
    //bad alignment, do icp again -> assume that the tf is alright
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(right_cloud);
    icp.setInputTarget(left_cloud);

    PointCloud r_pc_aligned;
    icp.align(r_pc_aligned);
    *left_cloud += r_pc_aligned;
    r_pc_aligned.clear();
    l_r_transform = icp.getFinalTransformation();
    PtrCloud dummy (new pcl::PointCloud<pcl::PointXYZ>());
    // ruin the right segmented cloud and blow it up to be the full right cloud
    pcl::transformPointCloud(*r_pc, *dummy, l_r_transform);
    *l_pc += *dummy;
    dummy->clear();
}


// this is a rally terrible naming scheme
void normal_normal_icp(PtrCloud left_cloud, PtrCloud right_cloud){
    NormPtrCloud l_cloud_normals (new pcl::PointCloud<pcl::PointNormal>());
    NormPtrCloud r_cloud_normals (new pcl::PointCloud<pcl::PointNormal>());

    normal_computation(left_cloud, l_cloud_normals);
    normal_computation(right_cloud, r_cloud_normals);
    
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setInputSource(r_cloud_normals);
    icp.setInputTarget(l_cloud_normals);               
    pcl::PointCloud<pcl::PointNormal> r_pc_aligned;
    // perform alignment
    icp.align(r_pc_aligned);
    // combine segmented clouds
    PointCloud dummy;
    pcl::copyPointCloud(r_pc_aligned, dummy);
    *left_cloud += dummy;
    r_pc_aligned.clear();
    dummy.clear();
    l_r_transform = icp.getFinalTransformation();
    pcl::transformPointCloud(*r_pc, dummy, l_r_transform);
    // combine full clouds
    *l_pc += dummy;
    dummy.clear();
}

void surface_finder(NormPtrCloud cloud, boost::shared_ptr<vector<pcl::PointNormal>> surfaces, pcl::PointNormal::Ptr starting_point){
    pcl::PointCloud<pcl::PointNormal> temp;
    for(int i = 0; i < cloud->points.size(); i++){
        if(cloud->points[i] != *starting_point && 
            cloud->points[i].n_x == starting_point->n_x &&
            cloud->points[i].n_y == starting_point->n_y &&
            cloud->points[i].n_z == starting_point->n_z){
                temp += cloud->points[i];
        }
    }

    
}

void flat_finder(NormPtrCloud left_cloud, NormPtrCloud right_cloud){


}

void tf_cap(){
    // check tf for wacky values
    // clip the values

}


int main(int argc, char** argv){
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    // initialize "locks"
    *Lup = 0;
    *Rup = 0;

    bool normals_icp = false;

    nh.getParam("tollerance", closeness_tol);
    nh.getParam("cone", cone_of_shame);
    nh.getParam("normals_icp", normals_icp);

    // TODO: load transform from file

    // unified publisher
    ros::Publisher unified_publisher = nh.advertise<sensor_msgs::PointCloud2>("pc_unified", 1);
    ros::Publisher cone_publisher = nh.advertise<sensor_msgs::PointCloud2>("pc_cone", 1);

    // subscribe to front left
    ros::Subscriber lsub = nh.subscribe<sensor_msgs::PointCloud2>("/VelodyneFrontLeft", 1, l_callback);
    // subscribe to front right
    ros::Subscriber rsub = nh.subscribe<sensor_msgs::PointCloud2>("/VelodyneFrontRight", 1, r_callback);

    ros::Rate loop_rate(100);

    while(nh.ok()){
        // if both point clouds have been updated then run the combination
        if(*Lup && *Rup){

            PtrCloud r_seg (new pcl::PointCloud<pcl::PointXYZ>());
            PtrCloud l_seg (new pcl::PointCloud<pcl::PointXYZ>());
            forward_slice(r_pc, r_seg);
            forward_slice(l_pc, l_seg);


            PtrCloud r_pcT (new pcl::PointCloud<pcl::PointXYZ>());
            // Transform right cloud to left
            // pcl::transformPointCloud(*l_pc, *r_pc, l_r_transform);
            pcl::transformPointCloud(*r_seg, *r_pcT, l_r_transform);
            // check alignment of the pointclouds
            if(closeness(l_seg, r_pcT) > closeness_tol){
                std::cout << "alignment changed" << std::endl;

                if(!normals_icp){
                    normal_icp(l_seg, r_pcT);
                }else{
                    normal_normal_icp(l_seg, r_pcT);
                }                

                // TODO: save transform to file
                ofstream tf_file;
                tf_file.open("tf.bin", ios::out | ios::binary);
                if(tf_file.is_open()){
                    tf_file << l_r_transform;
                    tf_file.close();
                }
            }else{
                std::cout << "good tf" << std::endl;
                // ROS_INFO("good tf");
                // previous alignment is good, use that
                r_pcT->clear();
                pcl::transformPointCloud(*r_pc, *r_pcT, l_r_transform);
                // combine full clouds
                *l_pc += *r_pcT;
                r_pcT->clear();
                pcl::transformPointCloud(*r_seg, *r_pcT, l_r_transform);
                // combine segmented clouds
                *l_seg += *r_pcT;
            }

            // convert back to pointcloud2 sensor msg
            sensor_msgs::PointCloud2 combo_cloud;
            pcl::toROSMsg(*l_pc, combo_cloud);
            combo_cloud.header.stamp = ros::Time::now();
            combo_cloud.header.frame_id = "vehicle";

            sensor_msgs::PointCloud2 seg_cloud;
            pcl::toROSMsg(*l_seg, seg_cloud);
            seg_cloud.header.stamp = ros::Time::now();
            seg_cloud.header.frame_id = "vehicle";

            // publish combined and aligned cloud
            unified_publisher.publish(combo_cloud);
            cone_publisher.publish(seg_cloud);
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
