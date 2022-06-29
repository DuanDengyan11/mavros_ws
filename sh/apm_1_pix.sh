# kill all screen session
pkill screen

# enable serial ports
sudo chmod -R 777 /dev/ttyUSB0
sudo chmod -R 777 /dev/ttyUSB1
sudo chmod -R 777 /dev/ttyACM0
sleep 1s

screen -d -m -S mavros bash -c "roslaunch mavros apm_1_pix.launch; exec bash -i";
echo "mavros ready"
sleep 20s

screen -d -m -S offboard bash -c "roslaunch offboard_pkg offboard_pkg.launch; exec bash -i";
echo "offboard ready"

rosrun mavros mavsys rate --all 10
