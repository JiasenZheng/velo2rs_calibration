#include <ros/ros.h>
#include <armadillo>
#include <cmath>

static arma::mat velo_o;
static arma::mat velo_x;
static arma::mat velo_z;
static arma::mat rs_o;
static arma::mat rs_x;
static arma::mat rs_z;

static arma::mat velo_x_dir, velo_y_dir, velo_z_dir;
static arma::mat rs_x_dir, rs_y_dir, rs_z_dir;

arma::mat normalize(const arma::mat &m)
{
    double x,y,z;
    x = m(0,0);
    y = m(1,0);
    z = m(2,0);
    double l = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    double x_bar, y_bar, z_bar;
    x_bar = x/l;
    y_bar = y/l;
    z_bar = z/l;
    arma::mat m_nor = arma::colvec({x_bar,y_bar,z_bar});
    return m_nor;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    // Get parameters
    double velo_o_x, velo_o_y, velo_o_z;
    ros::param::get("/velo/point_o/x", velo_o_x);
    ros::param::get("/velo/point_o/y", velo_o_y);
    ros::param::get("/velo/point_o/z", velo_o_z);
    velo_o = arma::colvec({velo_o_x,velo_o_y,velo_o_z});

    double velo_x_x, velo_x_y, velo_x_z;
    ros::param::get("/velo/point_x/x", velo_x_x);
    ros::param::get("/velo/point_x/y", velo_x_y);
    ros::param::get("/velo/point_x/z", velo_x_z);
    velo_x = arma::colvec({velo_x_x,velo_x_y,velo_x_z});

    double velo_z_x, velo_z_y, velo_z_z;
    ros::param::get("/velo/point_z/x", velo_z_x);
    ros::param::get("/velo/point_z/y", velo_z_y);
    ros::param::get("/velo/point_z/z", velo_z_z);
    velo_z = arma::colvec({velo_z_x,velo_z_y,velo_z_z});

    double rs_o_x, rs_o_y, rs_o_z;
    ros::param::get("/rs/point_o/x", rs_o_x);
    ros::param::get("/rs/point_o/y", rs_o_y);
    ros::param::get("/rs/point_o/z", rs_o_z);
    rs_o = arma::colvec({rs_o_x,rs_o_y,rs_o_z});

    double rs_x_x, rs_x_y, rs_x_z;
    ros::param::get("/rs/point_x/x", rs_x_x);
    ros::param::get("/rs/point_x/y", rs_x_y);
    ros::param::get("/rs/point_x/z", rs_x_z);
    rs_x = arma::colvec({rs_x_x,rs_x_y,rs_x_z});

    double rs_z_x, rs_z_y, rs_z_z;
    ros::param::get("/rs/point_z/x", rs_z_x);
    ros::param::get("/rs/point_z/y", rs_z_y);
    ros::param::get("/rs/point_z/z", rs_z_z);
    rs_z = arma::colvec({rs_z_x,rs_z_y,rs_z_z});

    // Calculate normalized directional vector (x,y,z)
    velo_x_dir = normalize(velo_x - velo_o);
 

}