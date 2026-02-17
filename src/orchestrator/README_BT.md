# BehaviorTree-based Orchestrator

## 当前实现范围

已重写为 **Phase0 -> (暂时跳过视觉检查) -> Phase1** 的最小可运行 BT 编排。

### 组件

1. **BT 节点封装** (`include/orchestrator/bt_mtc_nodes.hpp`)
    - `ExecutePhase0`: 调用 `controller::Phase0Node` 执行真实 MTC phase0
    - `ExecutePhase1`: 调用 `controller::Phase1Node` 执行真实 MTC phase1
    - `PublishPhaseNode`: 发布当前 phase 到 `/orchestrator/phase`

2. **任务流程定义** (`config/gantry_task.xml`)
    - 只保留 phase0 和 phase1
    - phase0 与 phase1 之间不做视觉检查

3. **主节点** (`src/bt_orchestrator_node.cpp`)
    - 安全初始化 BehaviorTree（避免在构造函数中 `shared_from_this()`）
    - 周期性 tick 行为树直到成功/失败

## 编译

```bash
cd /home/ooofieee/Desktop/ws_sim
colcon build --packages-select orchestrator --symlink-install
source install/setup.bash
```

## 运行

### 推荐：一键启动（包含 MoveIt 上下文）

```bash
ros2 launch orchestrator bt_orchestrator.launch.py
```

说明：该 launch **只启动** `bt_orchestrator_node`，不启动 MoveIt/控制器/RViz。

### 基本运行

```bash
ros2 run orchestrator bt_orchestrator_node
```

仅在你已经单独启动了 MoveIt（含 `move_group` 与 robot description/semantic）时使用。

例如先在另一个终端启动（按你的实际场景选择）：

```bash
ros2 launch gantry_robot_moveit_config_sim demo.launch.py
```

### 使用自定义 XML 文件

```bash
ros2 run orchestrator bt_orchestrator_node --ros-args \
  -p bt_xml_file:=/path/to/custom.xml
```

### 调整 tick 频率

```bash
ros2 run orchestrator bt_orchestrator_node --ros-args \
  -p tick_rate_ms:=50
```

## 视觉检查后续接入建议

后续可在 `gantry_task.xml` 的 phase0 和 phase1 之间插入 `VisionCheck` 节点，并在 `bt_mtc_nodes.hpp` 中新增对应服务调用逻辑。

## 发布的 Topics

- `/orchestrator/phase` (std_msgs/Int8): 当前执行阶段
  - 0: INIT/ALIGN
  - 1: PUSH
    - 视觉检查接入后可扩展 2/3 等后续阶段
