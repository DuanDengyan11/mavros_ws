

{
    gnome-terminal -- bash -c "killall gzserver; roslaunch  px4 mavros_posix_sitl.launch; exec bash"
}&
sleep 5s
{
    gnome-terminal -- bash -c "rosrun offboard_pkg px4_1; exec bash"
}


