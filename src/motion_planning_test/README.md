# Motion Planning Test Package

## 功能描述

这个包实现了一个运动规划测试，使gantry_robot的末端执行器移动到stand的末端执行器位置，实现两个末端执行器的位姿对齐，并包含完整的碰撞检测功能。

## 主要特性

- ✅ 使用MoveIt进行运动规划
- ✅ 自动获取stand末端执行器的位姿（通过TF变换）
- ✅ 设置gantry_robot末端执行器目标位姿
- ✅ 碰撞检测（MoveIt自动处理）
- ✅ 位姿对齐误差计算和验证
- ✅ 详细的日志输出

## 文件结构

```
motion_planning_test/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS 2包配置
├── README.md                   # 说明文档
├── launch/
│   ├── ee_alignment_test.launch.py    # 测试节点启动文件（需要先启动MoveIt）
│   └── full_test.launch.py            # 完整测试启动文件（自动启动MoveIt和测试）
└── src/
    └── ee_alignment_test.cpp   # 末端执行器对齐测试实现
```

## 依赖项

- ROS 2 (Humble或更高版本)
- MoveIt 2
- TF2
- geometry_msgs

## 编译

在工作空间根目录下：

```bash
cd ~/Desktop/ws_sim
colcon build --packages-select motion_planning_test
source install/setup.bash
```

## 使用方法

### 方法1：完整启动（推荐）

这种方法会自动启动MoveIt环境和测试节点：

```bash
ros2 launch motion_planning_test full_test.launch.py
```

测试节点会在5秒后自动启动（等待MoveIt初始化完成）。

### 方法2：分步启动

如果你已经运行了MoveIt demo：

```bash
# 终端1：启动MoveIt环境
ros2 launch gantry_robot_moveit_config demo.launch.py

# 终端2：启动测试节点
ros2 launch motion_planning_test ee_alignment_test.launch.py
```

## 工作原理

1. **初始化**：节点创建MoveGroupInterface实例，连接到gantry_robot规划组

2. **获取目标位姿**：
   - 使用TF2查询stand末端执行器（cylinder3）的当前位姿
   - 相对于world坐标系获取位置和方向

3. **设置目标**：将获取的位姿设置为gantry_robot的目标位姿

4. **运动规划**：
   - MoveIt规划器计算从当前位置到目标位置的轨迹
   - 自动进行碰撞检测
   - 检查运动学约束

5. **执行运动**：如果规划成功，执行计算出的轨迹

6. **验证**：计算实际位置与目标位置的误差，验证对齐精度

## 配置参数

在`ee_alignment_test.cpp`中可以调整以下参数：

- `setPlanningTime(10.0)`：规划超时时间（秒）
- `setNumPlanningAttempts(10)`：规划尝试次数
- `setMaxVelocityScalingFactor(0.3)`：最大速度缩放因子（0-1）
- `setMaxAccelerationScalingFactor(0.3)`：最大加速度缩放因子（0-1）

## 输出信息

测试运行时会输出以下信息：

- 规划组信息（名称、末端执行器、参考坐标系）
- Stand末端执行器的位置和姿态
- 规划状态（成功/失败）
- 轨迹信息（路径点数量、规划时间）
- 执行状态
- 当前末端执行器位置
- 位置对齐误差

## 故障排查

### 规划失败

如果规划失败，可能的原因：

1. **目标位姿超出工作空间**
   - 解决：检查gantry_robot的运动范围限制
   
2. **存在碰撞**
   - 解决：在RViz中查看碰撞情况，调整目标位姿或环境
   
3. **运动学求解失败**
   - 解决：检查运动学配置文件

### TF变换失败

如果无法获取TF变换：

1. 确保所有机器人模型正确加载
2. 检查robot_state_publisher是否运行
3. 使用`ros2 run tf2_tools view_frames`查看TF树

### 对齐误差过大

如果位置对齐误差 > 1cm：

1. 增加规划尝试次数
2. 调整规划时间
3. 检查运动学配置的精度
4. 考虑使用笛卡尔路径规划

## 扩展功能

可以在此基础上添加：

- 自定义轨迹约束
- 多路径点规划
- 实时障碍物避让
- 力控制集成
- 视觉伺服

## 许可证

Apache-2.0

## 作者

ooofieee
