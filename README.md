#  The 15th National College Student Smart Car Competition-Outdoor Optoelectronics Group

### 第十五届全国大学生智能汽车竞赛-室外光电组（ROS）

This rep based on Linux System: Ubuntu18.04 & ROS melodic version.

This rep stores the source code for Smart Car simulation.

* More information please check at '赛道和无人车三维模型.zip' file.
* Click [Preparation](https://blog.csdn.net/qq_37668436/article/details/107142166) for details about preparatory work.
* Click [Beginner Tutorial](https://www.guyuehome.com/6463) for framework of this project and some guidence.

* [Simple Project](https://github.com/xmy0916/racecar) is the source code of Beginner Tutorial.

More tutorials are as follow：

[详细使用文档](https://zacdeng.github.io)

[注意！] 有问题建议先查ROS wiki, 国内博客「古月居」也是个很好的参考

[智能车仿真教程一](https://www.guyuehome.com/9123)

[智能车仿真教程二](https://www.guyuehome.com/9251)

[智能车仿真教程三](https://www.guyuehome.com/9467)

[友链]([https://magicalsoso.github.io/2020/02/18/TwoWheelDiffChassisSLAM10-Navigation/#%E5%89%8D%E8%A8%80](https://magicalsoso.github.io/2020/02/18/TwoWheelDiffChassisSLAM10-Navigation/#前言))

- Racecar References：[Hypha-ROS/hypharos_racecar](https://github.com/Hypha-ROS/hypharos_racecar)
- AMCL：[amcl - ROS Wiki](http://wiki.ros.org/amcl)     [amcl - CSDN](https://blog.csdn.net/qq_29796781/article/details/80001355?tdsourcetag=s_pctim_aiomsg) 
- Global planner: [global_planner - ROS Wiki](http://wiki.ros.org/cn/navigation)
- TEB local planner：[teb_local_planner - ROS Wiki](http://wiki.ros.org/teb_local_planner)       [Reference1](https://blog.csdn.net/Fourier_Legend/article/details/89398485) and [Reference2](https://www.knightdusk.cn/2019/06/features-and-tuning-guide-for-teb-local-planner/)
- DWA local planner：[dwa_local_planner - ROS Wiki](http://wiki.ros.org/dwa_local_planner)    [Reference](https://blog.csdn.net/x_r_su/article/details/53393872)
- TF：[TF transfrom - CSDN](https://blog.csdn.net/zhanghm1995/article/details/84644984)

### Step

##### 适用包名：racecar_using_gazebo_odom

[注意！]

以下四步涉及的功能包为racecar_using_gazebo_odom压缩包，非racecar！！

(以下步骤为基本测试流程，使用gazebo_linkstate直接获取odometry信息，可以跑下全程时间在1分钟左右)

##### （1）打开.bashrc文件添加路径（也可以用echo source方法）

```
gedit ~/.bashrc
```

编辑后source

```
source ~/.bashrc
```

##### （2）启动gazebo + teleop control键盘控制

```
roslaunch racecar_gazebo racecar.launch
```

##### （3）启动 rviz +gazebo + teleop control键盘控制

```
roslaunch racecar_gazebo racecar_rviz.launch
```

##### （4）启动navigation功能

载入move_base相关配置，启动map_server

```
roslaunch racecar_gazebo racecar_runway_navigation.launch
```

启动 rviz+gazbo+navigation

```
roslaunch racecar_gazebo racecar_rviz_navigation.launch
```

启动纯追踪算法

```
rosrun racecar_gazebo path_pursuit.py
```

#### 适用包名：racecar

##### _（注意）整改后的launch_

- 由于赛方要求不能直接使用gazebo_linkstate得到的odometry信息，因此此方案添加由rf2o包中laser生成的odometry进行/odom信息发布

- 需要将rf2o_laser_odometry放在racecar同目录下（自己工作路径的/src下）

  （1）在终端中使用如下代码一键安装相关依赖

  ```
  rosdep install --from-paths src --ignore-src -r -y
  ```

  （2）转到工作空间下进行catkin_make

  ```
  catkin_make
  ```


- 添加robot_pose_ekf-master放在racecar同目录下（自己工作路径的/src下）

已写入相关launch文件中，直接启动下面两个launch文件即可

```
roslaunch racecar_gazebo racecar_runway_navigation.launch
```

```
roslaunch racecar_gazebo racecar_rviz_navigation.launch
```

* 为方便比赛时一键启动，统一添加至run_all.launch

  ```
  roslaunch racecar_gazebo run_all.launch
  ```

### Question

##### 1.Rviz中TF树无法正常加载，RobotStatus error：transform fail的情况

检查是否缺少相关组件，模型需要通过joint-state-publisher-gui获取当前车轮位置信息

安装gui模块：

```
sudo apt-get install ros-melodic-joint-state-publisher-gui
```

##### 2. For pkg : racecar——cmake时找不到OpenCV下的文件

CMake Error at /opt/ros/indigo/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
Could not find a package configuration file provided by “OpenCV” with any
of the following names:

OpenCVConfig.cmake

opencv-config.cmake

解决方法：

* 定位OpenCV安装位置（melodic默认在/usr/share/OpenCV/）得到你的opencv的路径

  （尽量选择ROS自带的OpenCV，一般是放在/usr/share/OpenCV/文件目录下）

```
locate OpenCVConfig.cmake
```

* 修改racecar_gazebo下的CMakeLists.txt文件

```
gedit ~/racecar_ws/src/racecar_gazebo/CMakeLists.txt
```

```
set(OpenCV_DIR /usr/share/OpenCV/)
```

##### 3. TF树相关问题

* /odom节点需要在amcl中进行配置（odom->base_footprint的tf变换）
* racecar.urdf.xacro文件中小车模型基准是base_footprint而非base_link，需要在对应文件中更改

```
grep -r base_link
```

在对应工作空间/src下用上述终端命令可查找和base_link相关文件及其路径（system中对应base_frame中的base_link也需要改为base_footprint，不要改tf树！！！不要改urdf！！！）

##### 4.源码包中path_pursuit.py文件中/pf/pose/odom接收不到信息

将/pf/pose/odom改为/vesc/odom暂时可行（供测试使用）

自己使用时需要将odom话题改为自己需要使用的odom话题

##### 5.加入amcl中tf树的构建

参考[博客]([https://magicalsoso.github.io/2020/02/18/TwoWheelDiffChassisSLAM10-Navigation/#amcl%E5%8A%9F%E8%83%BD%E5%8C%85%E4%B8%AD%E7%9A%84%E8%AF%9D%E9%A2%98%E5%92%8C%E6%9C%8D%E5%8A%A1](https://magicalsoso.github.io/2020/02/18/TwoWheelDiffChassisSLAM10-Navigation/#amcl功能包中的话题和服务))内容进行amcl配置

（1）map 和 odom 与 base_footprint 之间tf树均连通，但呈并列关系

修改 gazebo_odometry.py 文件中 cmd.header.frame_id 信息，使map - odom - base_footprint连通

错误如下：

![wrong](https://i.loli.net/2020/07/31/NqmYgJaDKI5VlML.png)

正确tf树：

![correct](https://i.loli.net/2020/07/31/NqmYgJaDKI5VlML.png)

##### 6.rf2o无法编译问题

- [注意！]

  遇到rf2o无法编译问题的解决方法：

  将laser_pose = laser_pose + pose_aux_2D改为

  ```
laser_pose = laser_pose + poses::CPose3D(pose_aux_2D)
  ```

### 说明

- 本包采用的小车模型为阿卡曼汽车模型，若采用差速小车模型需要自行更改

- 本包控制器为自行编写的解算控制过程，可能会有点乱😂有需要自行调整

