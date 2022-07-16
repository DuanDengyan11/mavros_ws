#include "MultiBody.h"
#include <time.h>

MultiBody::MultiBody(/* args */)
{
    // init
    ros::NodeHandle private_nh("~");
    private_nh.param("nVehicle", nVehicle, 1);
    private_nh.param("exist_load", exist_load, 0);
    nCraft = nVehicle;
    ninit_mission_required = nCraft - 1;
    if (exist_load)
    {
        nCraft = nVehicle - 1;
        lead_index = nVehicle - 1;
        ninit_mission_required = nCraft;
    }
    init_uav();
    GetRosData();

    XmlRpc::XmlRpcValue safe_error_dis_list;
    private_nh.getParam("safe_error_dis", safe_error_dis_list);
    Init_safe_error_dis(safe_error_dis_list);

    // takeoff
    XmlRpc::XmlRpcValue height_list;
    private_nh.getParam("height", height_list);
    Init_height(height_list);

    // waypoints
    XmlRpc::XmlRpcValue waypoint_list;
    private_nh.param("exist_waypoint", exist_waypoint, 1);
    if (exist_waypoint)
    {
        private_nh.param("nWayPoints", nWayPoints, 1);
        private_nh.getParam("LocalWayPoints", waypoint_list);
        Init_LocalWayPoints(waypoint_list);
    }

    XmlRpc::XmlRpcValue length_angle_list;
    private_nh.param("change_yaw", change_yaw, 0);
    private_nh.getParam("length_angle", length_angle_list);
    Init_length_angle(length_angle_list);
    private_nh.param("nSampleOrT", nSampleOrT, 0);
    private_nh.param("nedOrlla", nedOrlla, 1);
    private_nh.param("yaw_target", yaw_target, 0.0);

    // line
    private_nh.param("exist_line", exist_line, 1);
    private_nh.param("nSample_line", nSample_line, 5);
    private_nh.param("T_line", T_line, 10.0);
    private_nh.param("dis_line", dis_line, 20.0);

    // circle
    private_nh.param("exist_circle", exist_circle, 1);
    private_nh.param("nSample_circle", nSample_circle, 13);
    private_nh.param("T_circle", T_circle, 60.0);
    private_nh.param("R", R, 5.0);

    // init for log based on system time
    time_t t;
    struct tm *tmp;
    char buf2[128];
    time(&t);
    tmp = localtime(&t);
    strftime(buf2, 64, "/home/nuc/FController/log/%Y-%m-%d %H:%M:%S.text", tmp);
    ROS_INFO("%s", buf2);
    x_file = fopen(buf2, "w");
}

void MultiBody::Logging()
{
    Eigen::Vector3d enu, t_enu, lla, t_lla;
    //记录数据，方便后续检查程序合理性
    //记录uav[i].local_pos.x/y/z uav[i].target_local_pos.x/y/z uav[i].global_lla.x/y/z uav[i].target_global_lla.x/y/z 
    for (size_t i = 0; i < nCraft; i++)
    {
        uav[i].Log_pos(enu, t_enu, lla, t_lla);
        fprintf(x_file,"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, ", enu.x(), enu.y(), enu.z(), t_enu.x(), t_enu.y(), t_enu.z());
        fprintf(x_file,"%.7f, %.7f, %.7f, %.7f, %.7f, %.7f, ", lla.x(), lla.y(), lla.z(), t_lla.x(), t_lla.y(), t_lla.z());
    }
    //记录安全距离
    int distance_num = sizeof(distance) / sizeof(distance[0]);
    for (size_t i = 0; i < distance_num; i++)
    {
        fprintf(x_file,"%.2f, ", distance[i]);
    }
    fprintf(x_file,"\n"); //换行
}

bool MultiBody::WaitAllFCU()
{
    static bool AllFCUConnect = false;
    size_t j = 0;
    if (!AllFCUConnect)
    {
        for (size_t i = 0; i < nVehicle; i++)
        {
            if (uav[i].check_FCUconnected())
            {
                j = j + 1;
            }
        }
        if (j == nVehicle)
        {
            AllFCUConnect = true;
        }
    }
    return AllFCUConnect;
}

bool MultiBody::WaitAllGPS()
{
    static bool AllGPSConnect = false;
    size_t j = 0;
    if (!AllGPSConnect)
    {
        for (size_t i = 0; i < nVehicle; i++)
        {
            if (uav[i].check_GPSconnected())
            {
                j = j + 1;
            }
        }
        if (j == nVehicle)
        {
            AllGPSConnect = true;
        }
    }
    return AllGPSConnect;
}

bool MultiBody::AllTakeoff()
{
    static bool allTakeoff = false;
    size_t j = 0;
    if (!allTakeoff)
    {
        for (size_t i = 0; i < nCraft; i++)
        {
            if (uav[i].istakeoff())
            {
                j = j + 1;
            }
        }
        ROS_INFO_ONCE("j, %ld, ncraft, %d", j, nCraft);
        if (j == nCraft)
        {
            allTakeoff = true;
            delay(20);
        }
    }
    return allTakeoff;
}

bool MultiBody::AllLand()
{
    static bool allLand = false;
    size_t j = 0;
    if (!allLand)
    {
        for (size_t i = 0; i < nCraft; i++)
        {
            if (uav[i].CallLand())
            {
                j = j + 1;
            }
        }
        if (j == nCraft)
        {
            allLand = true;
        }
    }
    return allLand;
}

bool MultiBody::AllWayPoints()
{
    static bool AllPoints = false;
    static size_t k = 0;

    if (!exist_waypoint)
    {
        AllPoints = true;
        return AllPoints;
    }

    double distance;
    if (exist_waypoint && !AllPoints)
    {
        size_t j = 0;
        for (size_t i = 0; i < nCraft; i++)
        {
            if (uav[i].isWaypoint(k, false, yaw_target))
            {
                j = j + 1;
            }
        }
        if (j == nCraft)
        {
            ROS_INFO("waypiont %ld success", k);
            k = k + 1;
        }
        if (k == nWayPoints)
        {
            AllPoints = true;
        }
    }
    return AllPoints;
}

bool MultiBody::gps_mission_init(double &yaw_d, geometry_msgs::Quaternion &q_init, Eigen::Vector3d &lla_lead0, mission_type type)
{
    bool gps_mission = false;
    Eigen::Vector3d lla, ned, enu;
    uav[lead_index].get_att(yaw_d, q_init);
    lla_lead0 = uav[lead_index].get_lla();

    if (nSampleOrT)
    {
        switch (type)
        {
        case line:
            Init_lla_line(yaw_d, lla_lead0);
            break;
        case circle:
            Init_lla_circle(yaw_d, lla_lead0);
            break;
        default:
            break;
        }
    }
    else
    {
        T_begin = ros::Time::now().toSec();
    }

    size_t j = 0;
    for (size_t i = 0; i < nCraft; i++)
    {
        if (i != lead_index)
        {
            switch (type)
            {
            case line:
                ned = uav[i].get_ned_line(0.0, yaw_d);
                break;
            case circle:
                ned = uav[i].get_ned_circle(0.0, yaw_d, R, change_yaw);
                break;
            default:
                break;
            }
            uav[i].ned2lla(ned, lla_lead0, lla);
            if (nedOrlla)
            {
                enu = uav[i].lla2enu(lla);
                if (uav[i].is_enu_success(enu, yaw_d, 0.0, false, yaw_target))
                {
                    j = j + 1;
                }
            }
            else
            {
                if (uav[i].is_GPS_success(lla, yaw_d, false, yaw_target))
                {
                    j = j + 1;
                }
            }
        }
    }
    if (j == (ninit_mission_required))
    {
        delay(5);
        gps_mission = true;
        ROS_INFO("init success");
    }

    return gps_mission;
}

bool MultiBody::FlightLine()
{
    static bool line_success = false;
    if (!exist_line)
    {
        line_success = true;
        return line_success;
    }
    geometry_msgs::Quaternion q_init;
    Eigen::Vector3d lla, ned, enu;
    if (!line_init)
    {
        if (gps_mission_init(yaw_d, q_init, lla_lead0, mission_type::line))
        {
            line_init = true;
        }
    }

    static size_t k = 0;
    double T_now = ros::Time::now().toSec();
    double dis = dis_line / T_line * (T_now - T_begin);
    if (exist_line && !line_success && line_init)
    {
        if (nSampleOrT)
        {
            size_t j = 0;
            for (size_t i = 0; i < nCraft; i++)
            {
                if (nedOrlla)
                {
                    enu = uav[i].lla2enu(uav[i].lla_line[k]);
                    if (uav[i].is_enu_success(enu, yaw_d, 0.0, change_yaw, yaw_target))
                    {
                        j = j + 1;
                    }
                }
                else
                {
                    if (uav[i].is_GPS_success(uav[i].lla_line[k], yaw_d, change_yaw, yaw_target))
                    {
                        j = j + 1;
                    }
                }
            }
            if (j == nCraft)
            {
                ROS_INFO("line %ld success, total %d", k, nSample_line);
                k = k + 1;
            }
            if (k == (nSample_line - 1))
            {
                delay(50);
                line_success = true;
            }
        }
        else
        {
            for (size_t i = 0; i < nCraft; i++)
            {
                ned = uav[i].get_ned_line(dis, yaw_d);
                uav[i].ned2lla(ned, lla_lead0, lla);
                if(nedOrlla)
                {
                    enu = uav[i].lla2enu(lla);
                    uav[i].publish_enu(enu, yaw_d, 0.0, change_yaw, yaw_target);
                }else
                {
                    uav[i].publish_lla(lla, yaw_d, change_yaw, yaw_target);
                }
            }
            if ((T_now - T_begin) > T_line)
            {
                delay(50);
                line_success = true;
            }
        }
    }
    return line_success;
}

bool MultiBody::FlightCircle()
{
    static bool circle_success = false;
    if (exist_circle == 0)
    {
        circle_success = true;
        return circle_success;
    }

    geometry_msgs::Quaternion q_init;
    Eigen::Vector3d lla, ned, enu;
    if (!circle_init)
    {
        if (gps_mission_init(yaw_d, q_init, lla_lead0, mission_type::circle))
        {
            circle_init = true;
        }
    }

    static size_t k = 0;
    double T_now = ros::Time::now().toSec();
    double f = 1 / T_circle; // frequency
    double theta_d ;

    if (exist_circle && !circle_success && circle_init)
    {
        if (nSampleOrT)
        {
            theta_d = 360.0 / (nSample_circle - 1) * (k + 1);
            size_t j = 0;
            for (size_t i = 0; i < nCraft; i++)
            {
                if(nedOrlla)
                {
                    enu = uav[i].lla2enu(uav[i].lla_circle[k]);
                    if (uav[i].is_enu_success(enu, yaw_d, theta_d, change_yaw, yaw_target))
                    {
                        j = j + 1;
                    }
                }else
                {
                    if (uav[i].is_GPS_success(uav[i].lla_circle[k], yaw_d, change_yaw, yaw_target))
                    {
                        j = j + 1;
                    }
                }
            }
            if (j == nCraft)
            {
                ROS_INFO("circle %ld success, total %d", k, nSample_circle);
                k = k + 1;
            }
            if (k == (nSample_circle - 1))
            {
                delay(50);
                circle_success = true;
            }
        }
        else
        {
            theta_d = 2 * M_PI * f * (T_now - T_begin) * (180.0 / M_PI);
            for (size_t i = 0; i < nCraft; i++)
            {
                ned = uav[i].get_ned_circle(theta_d, yaw_d, R, change_yaw);
                uav[i].ned2lla(ned, lla_lead0, lla);
                if(nedOrlla)
                {
                    enu = uav[i].lla2enu(lla);
                    uav[i].publish_enu(enu, yaw_d, theta_d, change_yaw, yaw_target);
                }else
                {
                    uav[i].publish_lla(lla, yaw_d, change_yaw, yaw_target);
                }
            }

            if ((T_now - T_begin) > T_circle)
            {
                delay(50);
                circle_success = true;
            }
        }
    }

    return circle_success;
}

void MultiBody::delay(int num)
{
    ros::Rate rate(5.0);
    for (size_t i = 0; i < num; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void MultiBody::Init_lla_circle(double yaw_d, Eigen::Vector3d &lla_lead0)
{
    double theta_d;
    Eigen::Vector3d ned, lla;
    for (size_t k = 0; k < (nSample_circle - 1); k++)
    {
        theta_d = 360.0 / (nSample_circle - 1) * (k + 1);
        for (size_t i = 0; i < nVehicle; i++)
        {
            ned = uav[i].get_ned_circle(theta_d, yaw_d, R, change_yaw);
            uav[i].ned2lla(ned, lla_lead0, lla);
            uav[i].lla_circle.push_back(lla);
        }
    }
}

void MultiBody::Init_lla_line(double yaw_d, Eigen::Vector3d &lla_lead0)
{
    double dis;
    Eigen::Vector3d ned, lla;
    for (size_t k = 0; k < (nSample_line - 1); k++)
    {
        dis = dis_line / nSample_line * (k + 1);
        for (size_t i = 0; i < nVehicle; i++)
        {
            ned = uav[i].get_ned_line(dis, yaw_d);
            uav[i].ned2lla(ned, lla_lead0, lla);
            uav[i].lla_line.push_back(lla);
        }
    }
}

bool MultiBody::is_safe()
{
    bool safe = true;
    Eigen::Vector3d local_lla[nVehicle];
    for (size_t i = 0; i < nCraft; i++)
    {
        local_lla[i] = uav[i].get_lla();
    }
    int k = 0;
    for (size_t i = 0; i < nCraft; i++)
    {
        for (size_t j = i + 1; j < nCraft; j++)
        {
            distance[k] = uav[i].calcGPSDistance(local_lla[i], local_lla[j]);
            //  ROS_INFO("distance %f", distance);
            //  ROS_INFO("uav %d %f, %f, %f", i, local_lla[i].x(), local_lla[i].y(), local_lla[i].z());
            //  ROS_INFO("uav %d %f, %f, %f", j, local_lla[j].x(), local_lla[j].y(), local_lla[j].z());
            if (distance[k] < (uav[i].safe_distance + uav[j].safe_distance))
            {
                safe = false;
            }
            k = k+1;
        }
    }
    return safe;
}

void MultiBody::Init_height(XmlRpc::XmlRpcValue &height_list)
{
    if (nCraft != height_list.size())
    {
        ROS_ERROR("nCraft is not equal to size of height_list");
    }
    for (size_t i = 0; i < height_list.size(); i++)
    {
        XmlRpc::XmlRpcValue data_list(height_list[i]);
        uav[i].takeoff_height = data_list[0];
        uav[i].takeoff_height_tolerance = data_list[1];
    }
}

void MultiBody::Init_length_angle(XmlRpc::XmlRpcValue &length_angle_list)
{
    if (nCraft != length_angle_list.size())
    {
        ROS_ERROR("nCraft is not equal to size of length_angle_list");
    }
    for (size_t i = 0; i < length_angle_list.size(); i++)
    {
        XmlRpc::XmlRpcValue data_list(length_angle_list[i]);
        uav[i].length = data_list[0];
        uav[i].angle_section = data_list[1];
        uav[i].angle_cone = data_list[2];
    }
}

void MultiBody::Init_safe_error_dis(XmlRpc::XmlRpcValue &safe_error_dis_list)
{
    if (nCraft != safe_error_dis_list.size())
    {
        ROS_ERROR("nCraft is not equal to size of safe_error_dis_list");
    }
    for (size_t i = 0; i < safe_error_dis_list.size(); i++)
    {
        XmlRpc::XmlRpcValue data_list(safe_error_dis_list[i]);
        uav[i].safe_distance = data_list[0];
        uav[i].error_tolerance = data_list[1];
    }
}

void MultiBody::Init_LocalWayPoints(XmlRpc::XmlRpcValue &waypoint)
{
    geometry_msgs::PoseStamped tempPose;
    double yaw;
    int index_row = waypoint.size();
    if (nWayPoints != index_row)
    {
        ROS_ERROR("nWayPoints is not equal to size of waypoint");
    }
    for (size_t i = 0; i < index_row; i++)
    {
        tempPose.header.seq = i;
        XmlRpc::XmlRpcValue data_list(waypoint[i]);
        int index_column = data_list.size();
        if (nCraft != (index_column / 4))
        {
            ROS_ERROR("nVehicle is not equal to column of waypoint /4");
        }
        else if (index_column % 4 != 0)
        {
            ROS_ERROR("column of waypoint should be divided by 4");
        }
        for (size_t j = 0; j < index_column / 4; j++)
        {
            tempPose.pose.position.x = data_list[0 + 4 * j];
            tempPose.pose.position.y = data_list[1 + 4 * j];
            tempPose.pose.position.z = data_list[2 + 4 * j];
            yaw = data_list[3 + 4 * j];
            yaw = yaw * (M_PI / 180.0);
            tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
            tf::quaternionTFToMsg(q, tempPose.pose.orientation);
            uav[j].LocalWayPoints_matrix.push_back(tempPose);
        }
    }
}

void MultiBody::init_uav()
{
    TaskManager uav0;
    for (size_t i = 0; i < nVehicle; i++)
    {
        uav.push_back(uav0);
    }
}

void MultiBody::GetRosData()
{
    std::string s0 = "/uav", s;
    for (size_t i = 0; i < nVehicle; i++)
    {
        if (nVehicle == 1)
        {
            s = "";
        }
        else
        {
            s = s0 + to_string(i);
        }
        uav[i].RosData(s);
    }
}

bool MultiBody::check_all_mode()
{
    for (size_t i = 0; i < nCraft; i++)
    {
        if (!uav[i].check_mode())
        {
            return false;
        }
    }
    return true;
}

MultiBody::~MultiBody()
{
}
