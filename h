[1mdiff --git a/my_car_control/package.xml b/my_car_control/package.xml[m
[1mindex e67fa4e..88405a3 100644[m
[1m--- a/my_car_control/package.xml[m
[1m+++ b/my_car_control/package.xml[m
[36m@@ -7,22 +7,36 @@[m
   <maintainer email="user@example.com">davinci-mini</maintainer>[m
   <license>Apache-2.0</license>[m
 [m
[31m-  <!-- 依赖项 -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
[32m+[m[32m  <!-- ROS2 消息/服务依赖                                               -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
   <depend>rclpy</depend>[m
   <depend>geometry_msgs</depend>[m
   <depend>sensor_msgs</depend>[m
   <depend>nav_msgs</depend>[m
   <depend>nav2_msgs</depend>[m
   <depend>std_msgs</depend>[m
[32m+[m[32m  <depend>std_srvs</depend>                       <!-- 2025 补：control_node.py 用 std_srvs.srv.Empty -->[m
   <depend>tf2_ros</depend>[m
   <depend>tf2_geometry_msgs</depend>[m
   <depend>cv_bridge</depend>[m
   <depend>visualization_msgs</depend>[m
[32m+[m[32m  <depend>cartographer_ros_msgs</depend>          <!-- 2025 补：control_node.py 用 WriteState/FinishTrajectory -->[m
 [m
[31m-  <!-- 构建工具依赖 -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
[32m+[m[32m  <!-- bag 工具依赖（2025 新增 bag_to_csv 离线工具用）                  -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
[32m+[m[32m  <exec_depend>rosbag2_py</exec_depend>           <!-- bag_to_csv.py 读 ros2 bag -->[m
[32m+[m[32m  <exec_depend>rosidl_runtime_py</exec_depend>    <!-- bag_to_csv.py 反序列化消息 -->[m
[32m+[m
[32m+[m[32m  <!-- =============================================================== -->[m
[32m+[m[32m  <!-- 构建工具                                                          -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
   <buildtool_depend>ament_python</buildtool_depend>[m
 [m
[31m-  <!-- 测试依赖 -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
[32m+[m[32m  <!-- 测试依赖                                                          -->[m
[32m+[m[32m  <!-- =============================================================== -->[m
   <test_depend>ament_copyright</test_depend>[m
   <test_depend>ament_flake8</test_depend>[m
   <test_depend>ament_pep257</test_depend>[m
[36m@@ -32,4 +46,3 @@[m
     <build_type>ament_python</build_type>[m
   </export>[m
 </package>[m
[31m-[m
