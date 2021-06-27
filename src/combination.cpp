#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
// #include <pcl/common/transforms.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <sensor_msgs>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

boost::shared_ptr<bool> Lup(new bool), Rup(new bool);

geometry_msgs::TransformStamped l_transform;
geometry_msgs::TransformStamped r_transform;

// pcl::PointCloud<pcl::PointXYZ> l_trans_cloud;
// pcl::PointCloud<pcl::PointXYZ>::Ptr r_trans_cloud;

boost::shared_ptr<sensor_msgs::PointCloud2> l_trans_cloud(new sensor_msgs::PointCloud2);
boost::shared_ptr<sensor_msgs::PointCloud2> r_trans_cloud(new sensor_msgs::PointCloud2);


// listen to l lidar
void l_callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    if(*Lup == 0){
        l_trans_cloud->width = source_cloud->width;
        l_trans_cloud->height = source_cloud->height;
        l_trans_cloud->fields = source_cloud->fields;
        l_trans_cloud->point_step = source_cloud->point_step;
        l_trans_cloud->row_step = source_cloud->row_step;
        l_trans_cloud->row_step = source_cloud->is_dense;
        l_trans_cloud->is_bigendian = source_cloud->is_bigendian;
        tf2::doTransform(*source_cloud, *l_trans_cloud, l_transform); 
        *Lup = 1;
    }
}

// listen to r lidar
void r_callback(const sensor_msgs::PointCloud2ConstPtr& source_cloud){
    if(*Rup == 0){
        r_trans_cloud->width = source_cloud->width;
        r_trans_cloud->height = source_cloud->height;
        r_trans_cloud->fields = source_cloud->fields;
        r_trans_cloud->point_step = source_cloud->point_step;
        r_trans_cloud->row_step = source_cloud->row_step;
        r_trans_cloud->row_step = source_cloud->is_dense;
        r_trans_cloud->is_bigendian = source_cloud->is_bigendian;
        tf2::doTransform(*source_cloud, *r_trans_cloud, r_transform); 
        *Rup = 1;
    }
}


int main(int argc, char** argv){
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    // initialize "locks"
    *Lup = 0;
    *Rup = 0;

    tf2_ros::Buffer l_tfBuffer, r_tfBuffer;
    tf2_ros::TransformListener l_tfListener(l_tfBuffer);
    tf2_ros::TransformListener r_tfListener(r_tfBuffer);


    l_transform = l_tfBuffer.lookupTransform("vehicle", "VelodyneFrontLeft", ros::Time::now(), ros::Duration(3.0));
    r_transform = r_tfBuffer.lookupTransform("vehicle", "VelodyneFrontRight", ros::Time::now(), ros::Duration(3.0));

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

            sensor_msgs::PointCloud2 combo_cloud;
            combo_cloud.header.frame_id = "vehicle";
            combo_cloud.width = l_trans_cloud->width + r_trans_cloud->width;
            combo_cloud.height = r_trans_cloud->height;
            combo_cloud.fields = l_trans_cloud->fields;
            combo_cloud.is_bigendian = r_trans_cloud->is_bigendian;
            combo_cloud.is_dense = l_trans_cloud->is_dense;
            combo_cloud.point_step = r_trans_cloud->point_step;
            combo_cloud.row_step = l_trans_cloud->row_step;
            // concat the clouds
            combo_cloud.data = l_trans_cloud->data;
            combo_cloud.data.insert(combo_cloud.data.end(), r_trans_cloud->data.begin(), r_trans_cloud->data.end());
            unified_publisher.publish(combo_cloud);
            // std::cout << "published" << std::endl;
            combo_cloud.header.stamp = ros::Time::now();
            *Lup = 0;
            *Rup = 0;
        }else{
            // std::cout << "not ready" << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
