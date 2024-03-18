#编译：
catkin build

#建图1：
roslaunch qd_driver mapping.launch

###mapping.launch 建图时 <param name="pcd_save/pcd_save_en" type="bool" value="false"/>
#                         value=true
#                 导航时  value=false  

source ./devel/setup.bash

#建图2：
roslaunch livox_ros_driver2 rviz_MID360.launch

roslaunch livox_ros_driver2 msg_MID360.launch

roslaunch fast_lio mapping_mid360.launch


roslaunch point_lio mapping_avia.launch
#3D To 2D:
roslaunch pcd2pgm run.launch

#导航：
roslaunch qd_driver nav.launch

roslaunch rm_local_planner rm_local_planner.launch

#启动仿真
roslaunch simple_meca_car gazebo.launch
```bash
#LIO-SAM保存用
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap
#FAST-LIO保存用
ros2 service call /map_save std_srvs/srv/Trigger



Set pcd_save_enable in launchfile to 1. All the scans (in global frame) will be accumulated and saved to the file FAST_LIO/PCD/scans.pcd after the FAST-LIO is terminated. pcl_viewer scans.pcd can visualize the point clouds.

source /opt/ros/noetic/setup.bash

source ./devel/setup.bash

```
Odometry    base_footprint

运行fast-lio 与mid-360驱动。 


rostopic echo /livox/lidar

roslaunch qd_driver nav.launch

#查看坐标系相对变换
rosrun tf view_frames



#啟動lio
roslaunch qd_driver mapping.launch

#单独编译fast-lio
catkin_make -DCATKIN_WHITELIST_PACKAGES=fast-lio
所有代码功能包最好单独编译.



#查看pcd地图

pcl_viewer scans.pcd

pcl_viewer map_filter.pcd
#pcd地图专成2D栅格地图
roslaunch pcd2pgm run.launch

#保存栅格地图
rosrun map_server map_saver -f test

#加载栅格地图
<node name="map_server" pkg="map_server" type="map_server" args="$(find mycar_nav)/map/$(arg map)"/>

catkin_make

#hdl_localization
roslaunch hdl_localization hdl_localization.launch


