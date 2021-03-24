#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
 
 
main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_example");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/user/ros/kevin_cartographer_ws/2021-03-15-18-23-49.bag_points.ply", cloud) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
    sensor_msgs::PointCloud2 output;
    
    // Fill in the cloud data
    // cloud.width  = 100;
    // cloud.height = 1;
    // cloud.points.resize(cloud.width * cloud.height);
 
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    // {
    //     cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    // }
 
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";
 
    ros::Rate loop_rate(1);
    std::cout << "ply_publisher running..." << std::endl;
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    return 0;
}