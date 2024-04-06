catkin_make -DCATKIN_WHITELIST_PACKAGES="hc_lio" -j
source devel/setup.bash

read -p "choose a launch file
1. mid360.launch
2. velodyne.launch
3. m2dgr.launch
4. test.launch
5. kitti.launch" answer

if [ "$answer" = "1" ]; then
    echo "use mid360"
    roslaunch hc_lio mid360.launch
elif [ "$answer" = "2" ]; then
    echo "use velodyne"
    roslaunch hc_lio velodyne.launch
elif [ "$answer" = "3" ]; then
    echo "use m2dgr"
    roslaunch hc_lio m2dgr.launch
elif [ "$answer" = "4" ]; then
    echo "use test"
    roslaunch hc_lio test.launch
elif [ "$answer" = "5" ]; then
    echo "use kitti"
    roslaunch hc_lio kitti.launch
else
    echo "input is not invalid"
fi
