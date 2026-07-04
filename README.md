# RF Robot V2

这是一个基于 ROS 2 的移动机器人导航与建图工作区，当前代码已经具备一套可运行的基础框架：仿真模型、静态/局部代价地图、全局路径规划、调度转发、地图管理，以及基于 Cartographer 的 2D 建图接入。

仓库里的核心思路不是把所有功能拆成单独进程，而是通过 `rf_main` 把多个自研 node 组合到一个主进程里运行，再配合 `robot_model` 提供 Gazebo 仿真入口。

## 当前项目已经实现了什么

- `rf_main`
  统一启动并托管以下节点：
  `rf_global_map`、`rf_local_map`、`rf_map_manager`、`rf_global_planner`、`rf_scheduler_node`、`rf_map_builder`。

- `rf_costmap`
  提供代价地图基础库，已经实现：
  `StaticLayer`、`ObstacleLayer`、`InflationLayer`、`MasterCostmap`、`CostmapPublisher`。

- `rf_global_map`
  维护全局代价地图，默认加载图层：
  `static_layer + obstacle_layer + inflation_layer`。
  通过服务控制启动/停止更新，并在启动时请求静态地图发布。

- `rf_local_map`
  维护局部滚动窗口代价地图，默认大小 `5m x 5m`、分辨率 `0.025m`，图层为：
  `obstacle_layer + inflation_layer`。

- `rf_map_manager`
  负责静态占据栅格地图的加载、发布、查询与导出。
  当前支持：
  从 `~/.rf_robot/map/occ_map.yaml` 和 `occ_map.pgm` 读取地图；
  发布 `/map`；
  提供 `get_map`、`dump_map`、`/publish_static_map` 服务。

- `rf_global_planner`
  已实现一个默认全局规划器 `DefaultPlanner`，在自定义 `rf_robot_msgs/msg/Costmap` 上进行基于网格的搜索。
  目前能力包括：
  `ComputePathToPose` action；
  `ComputePathThroughPoses` action；
  路径发布到 `/global_path`；
  支持从 TF 获取当前位置作为起点；
  支持目标点容差搜索。

- `rf_scheduler_node`
  订阅 `/goal_pose`，把收到的目标点转发为 `/compute_path_to_pose` action 请求。
  这一层更像一个轻量调度壳，当前主要负责把 RViz/上层输入接到全局规划器。

- `rf_map_builder`
  对 Cartographer 2D 建图进行了 ROS 2 侧封装，当前已实现：
  从 Lua 配置加载建图参数；
  通过 `/build_map` 服务启动 trajectory；
  订阅 `scan`、`points2`、`odom`、`imu`、`landmark`；
  发布 `submap_list`、`tracked_pose`、`scan_matched_points2`；
  发布 `map -> odom` TF；
  支持激光、点云、里程计、IMU、地标数据桥接到 Cartographer。

- `robot_model`
  提供 Gazebo 仿真资源，包含：
  `fishbot_gazebo.urdf`；
  `fish.world`；
  `gazebo.launch.py`。
  该 launch 会启动 Gazebo、`robot_state_publisher`、`ros_gz_bridge`、`rf_main` 和 `rviz2`。

- `rf_robot_msgs`
  已定义项目内部使用的消息、服务和 action，包括：
  `Costmap`、`SubmapList`、`LandmarkList`、
  `GetMap`、`DumpMap`、`ReqAck`、
  `ComputePathToPose`、`ComputePathThroughPoses`。

## 仓库结构

- `src/rf_main`
  主程序入口，组合多个节点并分三个 executor 线程运行。

- `src/rf_costmap`
  代价地图核心实现。

- `src/rf_global_map`
  全局代价地图节点。

- `src/rf_local_map`
  局部代价地图节点。

- `src/rf_map_manager`
  静态地图读写与发布。

- `src/rf_global_planner`
  全局规划 action 服务与默认规划器。

- `src/rf_scheduler_node`
  简单调度层，把 `/goal_pose` 转换为规划请求。

- `src/rf_map_builder`
  Cartographer 2D 建图接入层。

- `idl/rf_robot_msgs`
  自定义消息、服务、action。

- `src/robot_model`
  Gazebo 仿真模型与启动文件。

- `src/cartographer`
  引入的 Cartographer 源码。

- `src/elog`
  日志库。

## 节点关系

系统目前的大致数据流如下：

1. `rf_map_manager` 从磁盘加载静态地图并发布 `/map`。
2. `rf_global_map` 订阅 `/map`，生成 `/global_costmap` 和 `/global_costmap_raw`。
3. `rf_local_map` 基于 `/scan` 和 TF 生成 `/local_costmap` 与 `/local_costmap_raw`。
4. `rf_global_planner` 订阅 `/global_costmap_raw`，对外提供路径规划 action。
5. `rf_scheduler_node` 监听 `/goal_pose`，调用规划 action。
6. `rf_map_builder` 独立接入传感器与 TF，维护 Cartographer trajectory，发布位姿、子图和点云结果。

## 关键接口

### Topics

- 输入
  `scan`
  `points2`
  `odom`
  `imu`
  `landmark`
  `/goal_pose`

- 输出
  `/map`
  `/global_costmap`
  `/global_costmap_raw`
  `/local_costmap`
  `/local_costmap_raw`
  `/global_path`
  `submap_list`
  `tracked_pose`
  `scan_matched_points2`

### Services

- `/global_map_control`
  启停全局代价地图更新。

- `/local_map_control`
  启停局部代价地图更新。

- `/publish_static_map`
  触发静态地图发布。

- `get_map`
  获取当前缓存的静态地图。

- `dump_map`
  将 `nav_msgs/OccupancyGrid` 导出到磁盘。

- `/build_map`
  启动 Cartographer trajectory。

### Actions

- `/compute_path_to_pose`
  计算起点到目标点的全局路径。

- `/compute_path_through_poses`
  计算经过多个途经点的全局路径。

## 默认配置与行为

### 地图管理

- 静态地图目录固定为：
  `~/.rf_robot/map`

- 默认读取文件：
  `~/.rf_robot/map/occ_map.yaml`
  `~/.rf_robot/map/occ_map.pgm`

### Cartographer 配置

- 默认配置文件：
  `src/rf_map_builder/config/default_map_builder_cfg.lua`

- 运行时搜索路径顺序：
  `rf_map_builder` 包内 `share/.../config`
  `~/.rf_robot/config`

- 默认配置启用了：
  2D trajectory builder
  `tracking_frame = base_link`
  `odom_frame = odom`
  `use_odometry = true`
  `use_imu_data = true`
  `num_laser_scans = 1`

## 快速开始

### 1. 编译

```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 准备运行目录

```bash
mkdir -p ~/.rf_robot/map
mkdir -p ~/.rf_robot/config
```

如果需要全局静态地图，请提前准备：

```text
~/.rf_robot/map/occ_map.yaml
~/.rf_robot/map/occ_map.pgm
```

### 3. 启动仿真

```bash
ros2 launch robot_model gazebo.launch.py
```

这个 launch 默认会一起拉起：

- Gazebo
- `robot_state_publisher`
- `ros_gz_bridge`
- `rf_main`
- `rviz2`

### 4. 启动建图

```bash
ros2 service call /build_map rf_robot_msgs/srv/ReqAck "{trigger: 0}"
```

### 5. 启动地图更新

```bash
ros2 service call /global_map_control rf_robot_msgs/srv/ReqAck "{trigger: 0}"
ros2 service call /local_map_control rf_robot_msgs/srv/ReqAck "{trigger: 0}"
```

### 6. 发送目标点

可以向 `/goal_pose` 发布 `geometry_msgs/msg/PoseStamped`，调度节点会自动转发给全局规划器。

## 当前实现边界

从代码现状看，这个项目已经具备“建图 + 代价地图 + 全局规划 + 仿真入口”的基础闭环，但还不是完整导航栈。下面这些点在 README 里特别说明一下，后续扩展会更清楚：

- `rf_map_builder` 当前通过 `/build_map` 只做 trajectory 启动，尚未看到完整的结束轨迹、保存 pbstream、导出 occupancy map 的工作流接入。
- 全局规划器目前只有一个默认规划器，核心是基于代价地图网格搜索的 2D 路径规划。
- 当前没有看到局部路径跟踪器、速度控制器、恢复行为或行为树调度模块。
- `rf_scheduler_node` 现在更偏向“请求转发器”，还不是复杂任务编排器。
- 顶层统一启动主要通过 `robot_model/gazebo.launch.py` 和 `rf_main`，独立节点 launch 还比较少。

## 适合继续补强的方向

- 增加建图结果保存、加载与地图切换流程。
- 打通从 Cartographer 结果到静态地图落盘的自动链路。
- 增加局部规划与控制器，实现真正导航闭环。
- 为各包补充更完整的参数化配置与 launch 文件。
- 为关键模块补更多单元测试与集成测试。

## 备注

仓库内的 `src/cartographer` 与 `src/elog` 更像依赖源码或内置第三方组件，项目的主要自研逻辑集中在 `rf_*` 系列包和 `idl/rf_robot_msgs`。
