set -e
echo "--------------------run perception------------"

source ../../../../devel/setup.bash

roslaunch sensorfusion perception_2020_03_25.launch 

sleep 10s
echo "--------------------run viedo publisher------------"
roslaunch video_publisher video.launch