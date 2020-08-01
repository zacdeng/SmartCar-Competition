#  gazebo障碍地图使用方法

### 测试gazebo地图是否正常加载

在racecar_gazebo中已编写test_world.launch文件。使用方法：

```
roslaunch racecar_gazebo test_world.launch
```

- 该launch文件仅加载机器人urdf以及gazebo地图，未加载控制器等
- 仅做测试地图使用

### 利用新地图跑racecar

更改方法：

- 打开racecar_gazebo中的racecar_rviz_navigation.launch文件

```
gedit ~/racecar_gazebo/launch/racecar_rviz_navigation.launch
```

- 修改其中<arg name="world_name" value="racetrack">

- 将value改为所需要的地图名称
- 启动地图

```
roslaunch racecar_gazebo racecar_rviz_navigation.launch
```

