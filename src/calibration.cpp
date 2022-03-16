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

static arma::mat Tvelo_obj;
static arma::mat Trs_obj;
static arma::mat Tvelo_rs;

static double x_coord, y_coord, z_coord, yaw, pitch, roll;

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
    velo_x_dir.print("velo_x: ");
    velo_z_dir = normalize(velo_z - velo_o);
    velo_z_dir.print("velo_z: ");
    velo_y_dir = arma::cross(velo_z_dir,velo_x_dir);
    velo_y_dir.print("velo_y: ");

    rs_x_dir = normalize(rs_x - rs_o);
    rs_x_dir.print("rs_x: ");
    rs_z_dir = normalize(rs_z - rs_o);
    rs_z_dir.print("rs_z: ");
    rs_y_dir = arma::cross(rs_z_dir,rs_x_dir);
    rs_y_dir.print("rs_y: ");

    // Construct T(velo->obj) and T(rs->obj)
    arma::mat velo_rot_pos = arma::join_horiz(velo_x_dir,velo_y_dir,velo_z_dir,velo_o);
    arma::mat bottom = arma::mat({0.0, 0.0, 0.0, 1.0});
    Tvelo_obj = arma::join_vert(velo_rot_pos,bottom);
    Tvelo_obj.print("Tvelo_obj: ");

    arma::mat rs_rot_pos = arma::join_horiz(rs_x_dir,rs_y_dir,rs_z_dir,rs_o);
    Trs_obj = arma::join_vert(rs_rot_pos,bottom);
    Trs_obj.print("Trs_obj: ");

    // Compute T(velo->rs)
    Tvelo_rs = Tvelo_obj*arma::inv(Trs_obj);
    Tvelo_rs.print("Tvelo_rs: ");

    // Get x, y, z
    x_coord = Tvelo_rs(0,3);
    y_coord = Tvelo_rs(1,3);
    z_coord = Tvelo_rs(2,3);
    ROS_INFO("x: %f, y: %f, z: %f",x_coord, y_coord, z_coord);

    // Compute yaw, pitch, and roll
    yaw = atan2(Tvelo_rs(1,0),Tvelo_rs(0,0));
    pitch = atan2(-Tvelo_rs(2,0),sqrt(pow(Tvelo_rs(2,1),2)+pow(Tvelo_rs(2,2),2)));
    roll = atan2(Tvelo_rs(2,1),Tvelo_rs(2,2));
    ROS_INFO("yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
 

}