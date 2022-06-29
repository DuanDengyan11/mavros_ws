#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <GeographicLib/Geoid.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <GeographicLib/Geocentric.hpp>
#include <tf/tf.h>
using namespace std;

class TaskManager
{
private:

    ros::NodeHandle nh;

    ros::Subscriber state_sub;
    ros::Subscriber local_pose_sub;
    ros::Subscriber global_pose_sub;
    ros::Publisher local_pos_pub;
    ros::Publisher global_pos_pub;
    ros::Publisher local_pos_raw_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_takeoff_client;
    ros::ServiceClient set_land_client;

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped local_pose;
    sensor_msgs::NavSatFix global_lla;
    mavros_msgs::CommandTOL takeoff;
    mavros_msgs::CommandTOL land;
    geometry_msgs::PoseStamped tempPose; //目标local_pos
    mavros_msgs::GlobalPositionTarget GeoPos; //目标lla

    bool FCUconnected = false;
    bool GPSconnected = false;
    bool is_takeoff = false; 

    static constexpr double DEG_TO_RAD = (M_PI / 180.0);

    // state 
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }
    // local_pos enu
    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        local_pose = *msg;
    }
    //global_pos lla
    void global_pose_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
        global_lla = *msg;
        GPSconnected = true;
    }

    double distance_pos(geometry_msgs::PoseStamped x1, geometry_msgs::PoseStamped x2);
    tf::Vector3 Vector3d2Vector3(Eigen::Vector3d input);
    Eigen::Vector3d Vector32Vector3d(tf::Vector3 input);
public:
    std::vector<geometry_msgs::PoseStamped> LocalWayPoints_matrix;
    std::vector<Eigen::Vector3d> lla_circle, lla_line;
    
    double takeoff_height, takeoff_height_tolerance;
    double length;
    double angle_section;
    double angle_cone;
    double safe_distance;
    double error_tolerance;

    TaskManager();
    ~TaskManager();
    void RosData(std::string &uav);
    bool check_FCUconnected();
    bool check_GPSconnected();
    bool istakeoff();
    bool isWaypoint(int i, bool change_yaw, double yaw_target);
    void get_att(double &yaw_d, geometry_msgs::Quaternion &q);
    Eigen::Vector3d get_ned_circle(double theta_d, double yaw_d, double R, bool change_yaw);
    Eigen::Vector3d get_ned_line(double dis_now, double yaw_d);
    Eigen::Vector3d trans_and_relative_pos(Eigen::Vector3d &pos, double yaw_d, double theta_d);

    Eigen::Vector3d lla2enu(const Eigen::Vector3d &lla1);
    void ned2lla(const Eigen::Vector3d &ned, const Eigen::Vector3d &lla0, Eigen::Vector3d &lla_new);
    double calcGPSDistance(Eigen::Vector3d &lla_new, Eigen::Vector3d &lla_old);
    Eigen::Vector3d get_lla();
    Eigen::Vector3d get_vel();
    bool is_GPS_success(Eigen::Vector3d &pos, double yaw_d, bool change_yaw, double yaw_target);
    bool CallLand();
    void SetAttOnly(geometry_msgs::Quaternion q);
    void publish_lla(Eigen::Vector3d &pos, double yaw_d, bool change_yaw, double yaw_target);
    bool check_mode();
    bool is_enu_success(Eigen::Vector3d &enu, double yaw_d, double theta_d, bool change_yaw, double yaw_target);
    void publish_enu(Eigen::Vector3d &enu, double yaw_d, double theta_d, bool change_yaw, double yaw_target);
    void Log_pos(Eigen::Vector3d &enu, Eigen::Vector3d &t_enu, Eigen::Vector3d &lla, Eigen::Vector3d &t_lla);
};
