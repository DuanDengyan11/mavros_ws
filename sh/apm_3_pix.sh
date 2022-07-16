# kill all screen session
pkill screen

# enable serial ports
sudo chmod -R 777 /dev/ttyUSB0
sudo chmod -R 777 /dev/ttyUSB1
sudo chmod -R 777 /dev/ttyUSB2
sudo chmod -R 777 /dev/ttyACM0
sleep 1s

screen -d -m -S mavros bash -c "roslaunch mavros apm_3_pix.launch; exec bash -i";
echo "mavros ready"
sleep 20s

screen -d -m -S offboard bash -c "roslaunch offboard_pkg offboard_pkg3.launch; exec bash -i";
echo "offboard ready"

rosrun mavros mavsys -n uav0/mavros  rate --all 10
rosrun mavros mavsys -n uav1/mavros  rate --all 10
rosrun mavros mavsys -n uav2/mavros  rate --all 10

