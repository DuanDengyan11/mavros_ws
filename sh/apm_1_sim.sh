# kill all screen session
pkill screen

screen -d -m -S gazebo bash -c "killall gzserver; gazebo --verbose iris_ardupilot.world; exec bash -i";
echo "gazebo ready"
sleep 5s

screen -d -m -S sitl bash -c "cd ~/FController/ardupilot/ArduCopter; ../Tools/autotest/sim_vehicle.py -f gazebo-iris; exec bash -i";
echo "sitl ready"
sleep 5s

screen -d -m -S mavros bash -c "roslaunch mavros apm_1_sim.launch; exec bash -i";
echo "mavros ready"
sleep 5s

screen -d -m -S offboard bash -c "roslaunch offboard_pkg offboard_pkg.launch; exec bash -i";
echo "offboard ready"

screen -r sitl


