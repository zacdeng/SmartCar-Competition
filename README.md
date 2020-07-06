#  The 15th National College Student Smart Car Competition-Outdoor Optoelectronics Group

### 第十五届全国大学生智能汽车竞赛-室外光电组（ROS）

This rep based on Linux System: Ubuntu18.04 & ROS melodic version.

This rep stores the source code for Smart Car simulation.

* More information please check at '赛道和无人车三维模型.zip' file.

* Click [Beginner Tutorial](https://blog.csdn.net/qq_37668436/article/details/107142166) for details about preparatory work.

### Question

##### 1.Rviz中TF树无法正常加载，RobotStatus error：transform fail的情况

检查是否缺少相关组件，模型需要通过joint-state-publisher-gui获取当前车轮位置信息

安装gui模块：

```
sudo apt-get install ros-melodic-joint-state-publisher-gui
```

