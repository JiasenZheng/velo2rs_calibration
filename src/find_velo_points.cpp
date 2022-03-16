#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Services
#include "velo2rs/Position.h"
#include "velo2rs/Record.h"

static ros::Subscriber cloud_sub;
static ros::Publisher cloud_pub;
static ros::ServiceServer set_pos;
static ros::ServiceServer record_point;
static double y_coord = 0.0;
static double z_coord = 0.0;
static double d = 0.2;

static pcl::PointXYZ coord_o, coord_x, coord_z;



void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
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
                if (cloud_in[i].y > coord_o.y or fabs(cloud_in[i].y - coord_o.y)<0.01)
                {
                    if (cloud_in[i].z < coord_o.z)
                    {
                        coord_o.x = cloud_in[i].x;
                        coord_o.y = cloud_in[i].y;
                        coord_o.z = cloud_in[i].z;
                    }
                }
                if (cloud_in[i].y < coord_x.y or fabs(cloud_in[i].y - coord_x.y)<0.01)
                {
                    if (cloud_in[i].z < coord_x.z)
                    {
                        coord_x.x = cloud_in[i].x;
                        coord_x.y = cloud_in[i].y;
                        coord_x.z = cloud_in[i].z;
                    }
                }
                if (cloud_in[i].y > coord_z.y or fabs(cloud_in[i].y - coord_z.y)<0.01)
                {
                    if (cloud_in[i].z > coord_z.z)
                    {
                        coord_z.x = cloud_in[i].x;
                        coord_z.y = cloud_in[i].y;
                        coord_z.z = cloud_in[i].z;
                    }
                }
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
    cloud_pub.publish(output);
}

bool set_pos_cb(velo2rs::Position::Request &req,
                velo2rs::Position::Response &res)
{
    y_coord = req.horizontal_coord;
    z_coord = req.vertical_coord;
    d = req.dis;

    // Update the coordiante of 3 points
    coord_o.y = y_coord;
    coord_o.z = z_coord;
    coord_x.y = y_coord;
    coord_x.z = z_coord;
    coord_z.y = y_coord;
    coord_z.z = z_coord;

    return true;
}

bool record_point_cb(velo2rs::Record::Request &req,
                velo2rs::Record::Response &res)
{
    if (req.position == "o")
    {
        res.point_x = coord_o.x;
        res.point_y = coord_o.y;
        res.point_z = coord_o.z;
    }
    else if (req.position == "x")
    {
        res.point_x = coord_x.x;
        res.point_y = coord_x.y;
        res.point_z = coord_x.z;
    }
    else if (req.position == "z")
    {
        res.point_x = coord_z.x;
        res.point_y = coord_z.y;
        res.point_z = coord_z.z;     
    }
    else
    {
        return false;
    } 
    return true;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc,argv,"find_velo_points");
    ros::NodeHandle nh;

    // Create subscribers, publishers, services
    cloud_sub = nh.subscribe("/mid/points", 1, cloud_cb);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/velo/points",1);
    set_pos = nh.advertiseService("/set_pos",set_pos_cb);
    record_point = nh.advertiseService("/record_point",record_point_cb);

    // Initialize points
    coord_o.x = 0.0;
    coord_o.y = y_coord;
    coord_o.z = z_coord;
    coord_x.x = 0.0;
    coord_x.y = y_coord;
    coord_x.z = z_coord;
    coord_z.x = 0.0;
    coord_z.y = y_coord;
    coord_z.z = z_coord;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}