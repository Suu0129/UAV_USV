# UAV-USV Cooperative Simulation

基于 ROS 2 Humble、Gazebo Sim、PX4 和 Nav2 的无人机/无人船协同仿真工作区。

当前可运行功能包括：

- PX4 无人机起飞、巡航、追踪船体停机坪和降落
- 无人船向灯塔运动及 RViz 目标点导航
- 船载相机、LaserScan、TF、Nav2 MPPI 和局部避障
- AIS 动态目标、碰撞风险和 COLREGs 测试场景
- 海浪、灯塔、浮标、动态目标船等 Gazebo 环境

原始可运行实现保留在 `uav_usv_sim`，其他包是面向多人并行开发建立的模块边界。

## 1. 环境要求

项目当前按以下环境开发：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim 8
- Python 3 和 `pymavlink`
- PX4-Autopilot（运行无人机任务时需要）
- Nav2（运行无人船导航时需要）

更完整的依赖、Gazebo 开发库和常见问题见
[复现运行指南](src/uav_usv_sim/docs/复现运行指南.md)。

## 2. 获取代码

只需要运行项目时，直接克隆主仓库：

```bash
git clone https://github.com/Suu0129/UAV_USV.git
cd UAV_USV
```

参与开发但没有主仓库写入权限时，先在 GitHub 页面点击右上角 `Fork`，
再克隆自己账号下的仓库：

```bash
git clone https://github.com/你的用户名/UAV_USV.git
cd UAV_USV
git remote add upstream https://github.com/Suu0129/UAV_USV.git
```

## 3. 编译

普通编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果 `/home` 空间有限，推荐将构建结果放到 `/var/tmp`：

```bash
export UAV_USV_BUILD=/var/tmp/UAV_USV_build
export UAV_USV_INSTALL=/var/tmp/UAV_USV_install
export UAV_USV_LOG=/var/tmp/UAV_USV_log

source /opt/ros/humble/setup.bash
colcon --log-base "$UAV_USV_LOG" build \
  --build-base "$UAV_USV_BUILD" \
  --install-base "$UAV_USV_INSTALL" \
  --symlink-install
source "$UAV_USV_INSTALL/setup.bash"
```

每次打开新终端，都需要重新执行 ROS 和工作区的 `source` 命令。

## 4. 启动仿真

### 只启动海面世界和键盘控制

```bash
ros2 launch uav_usv_sim uav_usv_world_keyboard.launch.py
```

### 启动 PX4、无人机和无人船世界

先将路径改成自己电脑上的 PX4 源码目录：

```bash
export PX4_DIR=/你的路径/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py px4_dir:="$PX4_DIR"
```

然后在另一个已完成 `source` 的终端启动协同任务：

```bash
ros2 launch uav_usv_sim cooperative_lighthouse_mission.launch.py
```

也可以用一条命令启动完整演示：

```bash
export PX4_DIR=/你的路径/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_cooperation_demo.launch.py \
  px4_dir:="$PX4_DIR"
```

### 使用 RViz 和 Nav2 控制无人船

终端 1 启动仿真：

```bash
export PX4_DIR=/你的路径/PX4-Autopilot
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py \
  px4_dir:="$PX4_DIR"
```

终端 2 启动 Nav2：

```bash
ros2 launch uav_usv_sim boat_nav2_navigation.launch.py
```

在 RViz 顶部选择 `Nav2 Goal`，然后在栅格地图上单击并拖动，设置目标位置和目标朝向。

### 启动 COLREGs 测试场景

```bash
ros2 launch uav_usv_sim colregs_test_scenario.launch.py
```

## 5. 统一启动入口

`uav_usv_bringup` 暂时通过兼容 launch 调用原始实现：

```bash
ros2 launch uav_usv_bringup legacy_px4_sim.launch.py \
  px4_dir:="$PX4_DIR"
ros2 launch uav_usv_bringup legacy_boat_nav2.launch.py
ros2 launch uav_usv_bringup legacy_colregs_test.launch.py
```

后续各模块迁移完成后，统一入口仍由 `uav_usv_bringup` 管理。

## 6. 多人开发

公开仓库允许任何人查看和 Fork，但只有协作者能直接向主仓库推送。
不逐个邀请成员时，统一采用 `Fork + Pull Request`。

Fork 成员开发前先同步项目负责人的最新代码：

```bash
git switch main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
```

项目负责人在主仓库中使用 `git pull origin main` 即可。

不要直接在 `main` 分支开发。每项任务创建自己的分支：

```bash
git switch -c feature/模块名-功能名
```

例如：

```bash
git switch -c feature/perception-mid360
git switch -c feature/navigation-dynamic-costmap
git switch -c feature/gazebo-ocean-world
```

完成开发后：

```bash
./tools/check_workspace.sh
git add .
git commit -m "feat(perception): add MID-360 simulation"
git push -u origin feature/perception-mid360
```

随后在 GitHub 创建 Pull Request，由模块负责人或项目负责人检查后合并。
Pull Request 的目标仓库选择 `Suu0129/UAV_USV`，目标分支选择 `main`。

各包职责、分支规则和验收要求见 [协作开发指南](COLLABORATION.md)；
Topic、消息、坐标系和 TF 约定见 [接口约定](docs/INTERFACES.md)。

## 7. 主要包

| 包 | 职责 |
|---|---|
| `uav_usv_sim` | 当前完整可运行的原始仿真实现 |
| `uav_usv_interfaces` | 公共消息、服务和动作接口 |
| `uav_usv_description` | 船体、无人机、传感器和 TF 描述 |
| `uav_usv_gazebo` | Gazebo 世界、模型和仿真插件 |
| `uav_usv_usv_control` | 无人船控制 |
| `uav_usv_uav_control` | PX4 和无人机控制 |
| `uav_usv_perception` | 雷达、相机、AIS 和目标融合 |
| `uav_usv_localization` | 定位、里程计、SLAM 和 TF |
| `uav_usv_navigation` | Nav2、规划、代价地图和局部控制 |
| `uav_usv_colregs` | 碰撞风险和 COLREGs 决策 |
| `uav_usv_mission` | 无人机/无人船协同任务 |
| `uav_usv_bringup` | 总 launch 和系统配置 |
| `uav_usv_tests` | 接口与集成测试 |
