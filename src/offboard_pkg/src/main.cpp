#include "MultiBody.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle offb_node;
    ros::Rate rate(5.0);

    MultiBody multi;
    // FCU connection
    while (ros::ok())
    {
        if (!multi.WaitAllFCU())
        {
            ROS_INFO_ONCE("waiting for FCU connection ...");
        }
        else
        {
            if (!multi.WaitAllGPS())
            {
                ROS_INFO_ONCE("waiting for GPS connection ...");
            }
            else
            {
                if (!multi.AllTakeoff())
                {
                    ROS_INFO_ONCE("takeoff ...");
                }
                else
                {
                    if (!multi.AllWayPoints())
                    {
                        ROS_INFO_ONCE("waypoints ...");
                    }
                    else
                    {
                        if (!multi.FlightLine())
                        {
                            ROS_INFO_ONCE("flight line ...");
                        }
                        else
                        {
                            if (!multi.FlightCircle())
                            {
                                ROS_INFO_ONCE("flight circle ...");
                            }
                            else
                            {
                                ROS_INFO_ONCE("All mission success ...");
                                break;
                            }
                        }
                    }
                    // if(!multi.check_all_mode())
                    // {
                    //     while (ros::ok())
                    //     {
                    //         ROS_INFO_ONCE("exit guided mode ...");
                    //         ros::spinOnce();
                    //         rate.sleep();
                    //     }
                    // }
                    if (!multi.is_safe())
                    {
                        ROS_ERROR("mission failed due to safe distance");
                        break;
                    }
                }
            }
        }
        multi.Logging(); //记录数据，方便分析
        ros::spinOnce();
        rate.sleep();
    }

    // land
    while (ros::ok())
    {
        multi.Logging(); //记录数据，方便分析
        if (!multi.AllLand())
        {
            ROS_INFO_ONCE("land ...");
        }
        else
        {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}