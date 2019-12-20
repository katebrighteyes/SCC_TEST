라이다 소스 다운로드

cd catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
cd ..
catkin_make
source ./devel/setup.bash
roslaunch hls_lfcd_lds_driver hlds_laser.launch

terminal 2

rviz

fixed frame : laser
topic : /scan
