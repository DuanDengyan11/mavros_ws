

{
    gnome-terminal -- bash -c "killall gzserver; roslaunch px4 multi_uav_mavros_sitl.launch; exec bash"
}&
sleep 5s
{
    gnome-terminal -- bash -c "rosrun offboard_pkg px4_2; exec bash"
}



