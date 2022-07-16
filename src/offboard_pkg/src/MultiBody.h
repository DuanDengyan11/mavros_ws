#include "TaskManager.h"
#include <vector>
#include <fstream> 
#include <string> 
#include <iostream>
using namespace std;

class MultiBody
{
private:
    std::vector<TaskManager> uav;
    int nVehicle;
    int nCraft;
    int ninit_mission_required;
    int exist_load; 
    int lead_index = 0;
    double safe_distance;

    // waypoints
    int exist_waypoint;
    int nWayPoints;
    
    // gps_mission 
    int nSampleOrT;
    int change_yaw;
    int nedOrlla;
    double yaw_target;

    // line
    bool line_init = false;
    int exist_line;
    int nSample_line;
    double T_line;
    double dis_line;

    // circle
    bool circle_init = false;
    bool circle_Time_init = false;
    int exist_circle;
    int nSample_circle;
    double T_circle;
    double R;

    Eigen::Vector3d lla_lead0;
    double T_begin;
    double yaw_d;

    FILE *x_file;
    double distance[5];

    enum mission_type{circle, line};

    void init_uav();
    void GetRosData();
    void Init_lla_circle(double yaw_d, Eigen::Vector3d &lla_lead0);
    void Init_lla_line(double yaw_d, Eigen::Vector3d &lla_lead0);
    void Init_LocalWayPoints(XmlRpc::XmlRpcValue &waypoint);
    void Init_height(XmlRpc::XmlRpcValue &height_list);
    void Init_safe_error_dis(XmlRpc::XmlRpcValue &safe_error_dis_list);
    void Init_length_angle(XmlRpc::XmlRpcValue &length_angle_list);
    bool gps_mission_init(double &yaw_d, geometry_msgs::Quaternion &q_init, Eigen::Vector3d &lla_lead0, mission_type type);
    void delay(int num);
public:
    
    MultiBody();
    ~MultiBody();
    bool WaitAllFCU();
    bool WaitAllGPS();
    bool AllTakeoff();
    bool AllWayPoints();
    bool FlightCircle();
    bool AllLand();
    bool is_safe();
    bool FlightLine();
    bool check_all_mode();
    void Logging();
};
