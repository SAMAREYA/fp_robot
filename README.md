# fp_robot

kalo mau make repo ini masukin dulu ke folder workspace/src 

``` cd workspace/src ```

lalu jalankan line ini untuk clonning 

``` git clone git@github.com:SAMAREYA/fp_robot.git ```

untuk menjalankan package jalankan line berikut 

``` ros2 launch fp_robot main_launch.py ```




## Using SSH

```bash
mkdir -p robot_ws/src && cd robot_ws/src
git clone git@github.com:ITSRobocon/robot_2025.git
cd ../..
colcon build --packages-select robot2025_utils
source install/setup.bash
colcon build --packages-select robot2025_msgs
source install/setup.bash
colcon build --packages-select robot2025_hardware robot2025_hardware_py robot2025_description
source install/setup.bash


