#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static ros::Subscriber velo_cloud_sub;
static ros::Subscriber rs_cloud_sub;
static ros::Publisher velo_cloud_pub;
static ros::Publisher rs_cloud_pub;
// static double x_coord = 0.0;
static double y_coord = 0.0;
static double z_coord = 0.0;
static double d = 0.3;

void velo_cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // Create a container for the output data
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;

    // Convert input senor sensor msg to pcl format
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*input,cloud_in);

    for (unsigned int i = 0; i < cloud_in.size(); i++)
    {
        if (cloud_in[i].x>0)
        {
            // Create a XYZRGB point
            pcl::PointXYZRGB colored_point;
            double x = cloud_in[i].x;
            double y = cloud_in[i].y;
            double z = cloud_in[i].z;
            if ((fabs(z-z_coord)<d) && (fabs(y-y_coord)<d))
            {
                colored_point.r = 255;
                colored_point.g = 0;
                colored_point.b = 0;
                colored_point.a = 255;
            }
            else
            {
                colored_point.r = 0;
                colored_point.g = 255;
                colored_point.b = 0;
                colored_point.a = 255;
            }
            colored_point.x = x;
            colored_point.y = y;
            colored_point.z = z;
            cloud_out.push_back(colored_point);
        }
    }
    // Convert pcl to sensor msg
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_out,output);
    output.header = input->header;

    // Publishe the data
    velo_cloud_pub.publish(output);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc,argv,"find_points");
    ros::NodeHandle nh;

    // Create subscribers, publishers, services
    velo_cloud_sub = nh.subscribe("/mid/points", 1, velo_cloud_cb);
    velo_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velo/points",1);
    
    rs_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rs/points",1);

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}