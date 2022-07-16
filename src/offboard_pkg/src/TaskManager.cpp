#include "TaskManager.h"

TaskManager::TaskManager()
{
}

void TaskManager::RosData(std::string &uav)
{
    state_sub = nh.subscribe<mavros_msgs::State>(uav + "/mavros/state", 10, &TaskManager::state_cb, this);
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav + "/mavros/local_position/pose", 10, &TaskManager::local_pose_cb, this);
    global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>(uav + "/mavros/global_position/raw/fix", 10, &TaskManager::global_pose_cb, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav + "/mavros/setpoint_position/local", 10);
    global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>(uav + "/mavros/setpoint_raw/global", 10);
    local_pos_raw_pub = nh.advertise<mavros_msgs::PositionTarget>(uav + "/mavros/setpoint_raw/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav + "/mavros/cmd/arming");
    set_takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>(uav + "/mavros/cmd/takeoff");
    set_land_client = nh.serviceClient<mavros_msgs::CommandTOL>(uav + "/mavros/cmd/land");
}

bool TaskManager::check_FCUconnected()
{
    if (current_state.connected)
    {
        FCUconnected = true;
    }
    return FCUconnected;
}

bool TaskManager::check_GPSconnected()
{
    return GPSconnected;
}

bool TaskManager::istakeoff()
{
    takeoff.request.altitude = takeoff_height;
    if (current_state.mode == "GUIDED")
    {
        ROS_INFO("local_height, %f, height , %f, error %f, tolerance %f", local_pose.pose.position.z, takeoff_height, local_pose.pose.position.z - takeoff_height, takeoff_height_tolerance);

        if (current_state.armed && !takeoff.response.success)
        {
            set_takeoff_client.call(takeoff);
        }
        if (abs(local_pose.pose.position.z - takeoff_height) < takeoff_height_tolerance)
        {
            is_takeoff = true;
        }
    }
    return is_takeoff;
}

bool TaskManager::check_mode()
{
    if (current_state.mode == "GUIDED")
    {
        return true;
    }
    return false;
}

void TaskManager::get_att(double &yaw_d, geometry_msgs::Quaternion &q)
{
    q = local_pose.pose.orientation;
    tf::Quaternion q1(local_pose.pose.orientation.x, local_pose.pose.orientation.y,
                      local_pose.pose.orientation.z, local_pose.pose.orientation.w);
    tf::Matrix3x3 m(q1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_d = 90 - yaw * 1.0 / DEG_TO_RAD;
}

void TaskManager::Log_pos(Eigen::Vector3d &enu, Eigen::Vector3d &t_enu, Eigen::Vector3d &lla, Eigen::Vector3d &t_lla)
{
    enu.x() = local_pose.pose.position.x;
    enu.y() = local_pose.pose.position.y;
    enu.z() = local_pose.pose.position.z;
    t_enu.x() = tempPose.pose.position.x;
    t_enu.y() = tempPose.pose.position.y;
    t_enu.z() = tempPose.pose.position.z;
    lla.x() = global_lla.latitude;
    lla.y() = global_lla.longitude;
    lla.z() = global_lla.altitude;
    t_lla.x() = GeoPos.latitude;
    t_lla.y() = GeoPos.longitude;
    t_lla.z() = GeoPos.altitude;
}

void TaskManager::SetAttOnly(geometry_msgs::Quaternion q)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation = q;
    pose.pose.position = local_pose.pose.position;
    local_pos_pub.publish(pose);
}

bool TaskManager::isWaypoint(int i, bool change_yaw, double yaw_target)
{
    // local_pos
    bool is_Waypoint = false;
    if (!change_yaw)
    {
        double yaw = yaw_target * DEG_TO_RAD;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        tf::quaternionTFToMsg(q, LocalWayPoints_matrix[i].pose.orientation);
    }
    local_pos_pub.publish(LocalWayPoints_matrix[i]);
    if (distance_pos(LocalWayPoints_matrix[i], local_pose) < error_tolerance)
    {
        is_Waypoint = true;
    }
    return is_Waypoint;
}

double TaskManager::distance_pos(geometry_msgs::PoseStamped x1, geometry_msgs::PoseStamped x2)
{
    return sqrt(pow(x1.pose.position.x - x2.pose.position.x, 2) + pow(x1.pose.position.y - x2.pose.position.y, 2) +
                pow(x1.pose.position.z - x2.pose.position.z, 2));
}

Eigen::Vector3d TaskManager::get_ned_circle(double theta_d, double yaw_d, double R, bool change_yaw)
{
    Eigen::Vector3d pos;
    // local coordinates relative to vehicle yaw
    pos.x() = R * std::sin(theta_d * DEG_TO_RAD);
    pos.y() = R * std::cos(theta_d * DEG_TO_RAD) - R;
    pos.z() = 0;
    if (change_yaw)
    {
        return trans_and_relative_pos(pos, yaw_d, theta_d);
    }
    else
    {
        return trans_and_relative_pos(pos, yaw_d, 0.0);
    }
}

Eigen::Vector3d TaskManager::get_ned_line(double dis_now, double yaw_d)
{
    Eigen::Vector3d pos;
    // local coordinates relative to vehicle yaw
    pos.x() = dis_now;
    pos.y() = 0;
    pos.z() = 0;

    return trans_and_relative_pos(pos, yaw_d, 0.0);
}

Eigen::Vector3d TaskManager::trans_and_relative_pos(Eigen::Vector3d &pos, double yaw_d, double theta_d)
{
    Eigen::Vector3d pos_rel, ned;
    tf::Matrix3x3 m;

    // ned
    m.setRPY(0, 0, -yaw_d * DEG_TO_RAD);
    ned = Vector32Vector3d(m.transpose() * Vector3d2Vector3(pos));

    m.setRPY(0, 0, (-yaw_d + theta_d) * DEG_TO_RAD);
    pos_rel.x() = length * sin(angle_section * DEG_TO_RAD) * cos(angle_cone * DEG_TO_RAD);
    pos_rel.y() = length * cos(angle_section * DEG_TO_RAD) * cos(angle_cone * DEG_TO_RAD);
    pos_rel.z() = -length * sin(angle_cone * DEG_TO_RAD);

    ned = Vector32Vector3d(m.transpose() * Vector3d2Vector3(pos_rel)) + ned;

    return ned;
}

void TaskManager::ned2lla(const Eigen::Vector3d &ned, const Eigen::Vector3d &lla0, Eigen::Vector3d &lla_new)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d ecef0, ecef;
    GeographicLib::Geocentric map(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
    // transformation matrix from ned to ecef
    const double sin_lat = std::sin(lla0.x() * DEG_TO_RAD);
    const double sin_lon = std::sin(lla0.y() * DEG_TO_RAD);
    const double cos_lat = std::cos(lla0.x() * DEG_TO_RAD);
    const double cos_lon = std::cos(lla0.y() * DEG_TO_RAD);
    R << -cos_lon * sin_lat, -sin_lon * sin_lat, cos_lat,
        -sin_lon, cos_lon, 0.0,
        -cos_lon * cos_lat, -sin_lon * cos_lat, -sin_lat;

    // ecef of lla0
    map.Forward(lla0.x(), lla0.y(), lla0.z(), ecef0.x(), ecef0.y(), ecef0.z());

    // ecef of the new point
    R.transposeInPlace();
    ecef = R * ned + ecef0;

    // ecef to lla
    map.Reverse(ecef.x(), ecef.y(), ecef.z(), lla_new.x(), lla_new.y(), lla_new.z());
}

Eigen::Vector3d TaskManager::lla2enu(const Eigen::Vector3d &lla1)
{
    Eigen::Vector3d ecef0, ecef1, delta_ecef, enu;
    Eigen::Matrix3d R;

    Eigen::Vector3d lla0;
    lla0.x() = global_lla.latitude;
    lla0.y() = global_lla.longitude;
    lla0.z() = global_lla.altitude;

    //为方便log
    GeoPos.latitude = lla1.x();
    GeoPos.longitude = lla1.y();
    GeoPos.altitude = lla1.z();

    GeographicLib::Geocentric map(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
    map.Forward(lla0.x(), lla0.y(), lla0.z(), ecef0.x(), ecef0.y(), ecef0.z());
    map.Forward(lla1.x(), lla1.y(), lla1.z(), ecef1.x(), ecef1.y(), ecef1.z());
    const double sin_lat = std::sin(lla0.x() * DEG_TO_RAD);
    const double sin_lon = std::sin(lla0.y() * DEG_TO_RAD);
    const double cos_lat = std::cos(lla0.x() * DEG_TO_RAD);
    const double cos_lon = std::cos(lla0.y() * DEG_TO_RAD);
    R << -cos_lon * sin_lat, -sin_lon * sin_lat, cos_lat,
        -sin_lon, cos_lon, 0.0,
        -cos_lon * cos_lat, -sin_lon * cos_lat, -sin_lat;
    delta_ecef.x() = ecef1.x() - ecef0.x();
    delta_ecef.y() = ecef1.y() - ecef0.y();
    delta_ecef.z() = ecef1.z() - ecef0.z();

    Eigen::Vector3d ned = R * delta_ecef;
    enu.x() = ned.y() + local_pose.pose.position.x;
    enu.y() = ned.x() + local_pose.pose.position.y;
    enu.z() = -ned.z() + local_pose.pose.position.z;
    return enu;
}

double TaskManager::calcGPSDistance(Eigen::Vector3d &lla_new, Eigen::Vector3d &lla_old)
{

    double latitude_new = lla_new.x();
    double longitude_new = lla_new.y();
    double altitude_new = lla_new.z();
    double latitude_old = lla_old.x();
    double longitude_old = lla_old.y();
    double altitude_old = lla_old.z();

    double lat_new = latitude_old * DEG_TO_RAD;
    double lat_old = latitude_new * DEG_TO_RAD;
    double lat_diff = (latitude_new - latitude_old) * DEG_TO_RAD;
    double lng_diff = (longitude_new - longitude_old) * DEG_TO_RAD;

    double a = sin(lat_diff / 2) * sin(lat_diff / 2) +
               cos(lat_new) * cos(lat_old) *
                   sin(lng_diff / 2) * sin(lng_diff / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double RADIO_TERRESTRE = 6372797.56085;
    double distance = RADIO_TERRESTRE * c;

    return sqrt(pow(distance, 2) + pow(altitude_new - altitude_old, 2));
}

Eigen::Vector3d TaskManager::get_lla()
{
    Eigen::Vector3d lla;
    lla.x() = global_lla.latitude;
    lla.y() = global_lla.longitude;
    lla.z() = global_lla.altitude;
    return lla;
}

bool TaskManager::is_GPS_success(Eigen::Vector3d &pos, double yaw_d, bool change_yaw, double yaw_target)
{
    bool GPS_succuss = false;
    GeographicLib::Geoid egm96("egm96-5");
    GeoPos.coordinate_frame = 5;
    if (change_yaw)
    {
        GeoPos.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
    }
    else
    {
        GeoPos.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
        // GeoPos.yaw = (90 - yaw_d) * DEG_TO_RAD;
        GeoPos.yaw = yaw_target * DEG_TO_RAD;
    }

    GeoPos.latitude = pos.x();
    GeoPos.longitude = pos.y();
    GeoPos.altitude = pos.z() + GeographicLib::Geoid::ELLIPSOIDTOGEOID * egm96(GeoPos.latitude, GeoPos.longitude);
    global_pos_pub.publish(GeoPos);

    Eigen::Vector3d lla_new = {global_lla.latitude, global_lla.longitude, global_lla.altitude};
    double distance1 = calcGPSDistance(lla_new, pos);
    if (distance1 < error_tolerance)
    {
        GPS_succuss = true;
    }
    return GPS_succuss;
}

bool TaskManager::is_enu_success(Eigen::Vector3d &enu, double yaw_d, double theta_d, bool change_yaw, double yaw_target)
{
    bool is_enu = false;
    tempPose.pose.position.x = enu.x();
    tempPose.pose.position.y = enu.y();
    tempPose.pose.position.z = local_pose.pose.position.z;

    double yaw;
    if (change_yaw)
    {
        yaw = (90 - yaw_d + theta_d) * DEG_TO_RAD;
        ;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        tf::quaternionTFToMsg(q, tempPose.pose.orientation);
    }
    else
    {
        yaw = yaw_target * DEG_TO_RAD;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        tf::quaternionTFToMsg(q, tempPose.pose.orientation);
    }

    local_pos_pub.publish(tempPose);

    if (distance_pos(tempPose, local_pose) < error_tolerance)
    {
        is_enu = true;
    }
    return is_enu;
}

void TaskManager::publish_lla(Eigen::Vector3d &pos, double yaw_d, bool change_yaw, double yaw_target)
{
    GeographicLib::Geoid egm96("egm96-5");
    GeoPos.coordinate_frame = 5;
    if (change_yaw)
    {
        GeoPos.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
    }
    else
    {
        GeoPos.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
        // GeoPos.yaw = (90 - yaw_d) * DEG_TO_RAD;
        GeoPos.yaw = yaw_target * DEG_TO_RAD;
    }

    GeoPos.latitude = pos.x();
    GeoPos.longitude = pos.y();
    GeoPos.altitude = pos.z() + GeographicLib::Geoid::ELLIPSOIDTOGEOID * egm96(GeoPos.latitude, GeoPos.longitude);
    global_pos_pub.publish(GeoPos);
}

void TaskManager::publish_enu(Eigen::Vector3d &enu, double yaw_d, double theta_d, bool change_yaw, double yaw_target)
{
    tempPose.pose.position.x = enu.x();
    tempPose.pose.position.y = enu.y();
    tempPose.pose.position.z = local_pose.pose.position.z;

    double yaw;
    if (change_yaw)
    {
        yaw = (90 - yaw_d + theta_d) * DEG_TO_RAD;
        ;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        tf::quaternionTFToMsg(q, tempPose.pose.orientation);
    }
    else
    {
        yaw = yaw_target * DEG_TO_RAD;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        tf::quaternionTFToMsg(q, tempPose.pose.orientation);
    }
    local_pos_pub.publish(tempPose);
}

tf::Vector3 TaskManager::Vector3d2Vector3(Eigen::Vector3d input)
{
    tf::Vector3 output = {input.x(), input.y(), input.z()};
    return output;
}

Eigen::Vector3d TaskManager::Vector32Vector3d(tf::Vector3 input)
{
    Eigen::Vector3d output = {input[0], input[1], input[2]};
    return output;
}

bool TaskManager::CallLand()
{
    bool island = false;
    if (set_land_client.call(land) && land.response.success)
    {
        island = true;
    }
    return island;
}

TaskManager::~TaskManager()
{
}
