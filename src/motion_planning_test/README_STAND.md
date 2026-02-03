# Stand Random Pose Test

这个节点用于测试stand机器人的末端执行器（cylinder3）在以ipm为参考系的随机位姿中的可达性。

## 功能

- 生成以ipm为参考系的随机位姿
- 使用MoveIt进行运动规划
- 执行规划并统计成功率

## 位姿范围

### 位置范围（相对于ipm坐标系）
- X: -100mm 到 100mm
- Y: -100mm 到 100mm  
- Z: -100mm 到 0mm

### 姿态范围
- Roll: -45° 到 45°
- Pitch: -90° 到 0°
- Yaw: -90° 到 90°

## 使用方法

### 1. 启动MoveIt配置（需要先运行）
```bash
ros2 launch gantry_robot_moveit_config demo.launch.py
```

### 2. 启动随机位姿测试节点
```bash
# 使用默认参数（10次测试）
ros2 launch motion_planning_test stand_random_pose.launch.py

# 自定义测试次数
ros2 launch motion_planning_test stand_random_pose.launch.py test_count:=20

# 自定义规划组和参考系
ros2 launch motion_planning_test stand_random_pose.launch.py \
    planning_group:=stand \
    reference_frame:=ipm \
    end_effector:=cylinder3 \
    test_count:=10 \
    planning_time:=10.0
```

### 3. 或直接运行节点
```bash
ros2 run motion_planning_test stand_random_pose \
    --ros-args \
    -p planning_group:=stand \
    -p reference_frame:=ipm \
    -p end_effector_link:=cylinder3 \
    -p test_count:=10 \
    -p planning_time:=10.0
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planning_group` | `stand` | MoveIt规划组名称 |
| `reference_frame` | `ipm` | 生成随机位姿的参考坐标系 |
| `end_effector_link` | `cylinder3` | 末端执行器link名称 |
| `test_count` | `10` | 测试次数 |
| `planning_time` | `10.0` | 单次规划时间限制（秒） |

## 输出示例

```
========== Test 1/10 ==========
Generated random pose: pos[0.042, -0.078, -0.056] rpy[23.45°, -45.67°, 12.34°]
Planning...
✓ Planning succeeded!
Executing...
✓ Execution succeeded!

...

========== Test Summary ==========
Total tests: 10
Successful: 8 (80.0%)
Failed: 2
All tests completed!
```

## 注意事项

1. 确保已正确配置stand的MoveIt规划组（在SRDF中定义为`stand`）
2. 确保ipm坐标系已在URDF中定义并可用
3. 如果成功率较低，可以：
   - 调整位姿范围
   - 增加规划时间（planning_time）
   - 检查碰撞配置
