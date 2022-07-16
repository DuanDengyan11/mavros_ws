# kill all screen session
pkill screen

# gazebo
screen -d -m -S gazebo bash -c "killall gzserver; gazebo --verbose iris_arducopter_two.world; exec bash -i";
echo "gazebo ready"
sleep 5s

# sitl1
screen -d -m -S sitl1 bash -c "cd ~/FController/ardupilot/ArduCopter; ../Tools/autotest/sim_vehicle.py -f gazebo-iris -I0 --sysid=1; exec bash -i";
echo "sitl1 ready"
sleep 5s

# sitl2
screen -d -m -S sitl2 bash -c "cd ~/FController/ardupilot/ArduCopter; ../Tools/autotest/sim_vehicle.py -f gazebo-iris -I1 --sysid=2; exec bash -i";
echo "sitl2 ready"
sleep 5s

# mavros
screen -d -m -S mavros bash -c "roslaunch mavros apm_2_sim.launch; exec bash -i";
echo "mavros ready"
sleep 5s

# offboard
screen -d -m -S offboard bash -c "roslaunch offboard_pkg offboard_pkg2.launch; exec bash -i";
echo "offboard ready"

{
    gnome-terminal -- bash -c "screen -r sitl1; exec bash"
}&
{
    gnome-terminal -- bash -c "screen -r sitl2; exec bash"
}

