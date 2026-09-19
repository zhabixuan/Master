# path_planner 优化说明

> 涉及文件：
> - [src/liom_local_planner/include/liom_local_planner/path_planner.h](../../src/liom_local_planner/include/liom_local_planner/path_planner.h)
> - [src/liom_local_planner/src/path_planner.cpp](../../src/liom_local_planner/src/path_planner.cpp)
>
> 未参考 `coarse_path_planner`。规划算法本身（混合 A* + 2D Dijkstra 启发式 + Reeds-Shepp one-shot）保持不变，只优化了热路径上的多余开销、消除了内存分配、修复了若干健壮性问题并清理了死代码。

---

## 1. 去掉主循环里的冗余碰撞检测

**改动**

在 `Plan()` 的主循环里，`ExpandNextNode()` 内部已经对整条 kinematic path 的每个位姿（包括最后一个位姿 `path.back()`，也就是 `next_node->pose`）做过了 `CheckPoseCollision`，所以循环里紧接着的那次检查是重复的，已删除：

```cpp
// 删除前
if (!ExpandNextNode(current_node, i, next_node)) { continue; }
if (closed_set_.count(next_node->index) > 0) { continue; }
if (env_->CheckPoseCollision(0.0, next_node->pose)) { continue; }   // ← 冗余
next_node->set_cost(...);

// 删除后
if (!ExpandNextNode(current_node, i, next_node)) { continue; }
if (closed_set_.count(next_node->index) > 0) { continue; }
next_node->set_cost(...);
```

**为什么**

`ExpandNextNode()` 中：

```cpp
auto path = GenerateKinematicPath(node->pose, is_forward, steering);
...
for (auto &pose : path) {
  if (env_->CheckPoseCollision(0.0, pose)) { return false; }   // 已检查 path.back()
}
next_node = std::make_shared<Node3d>(path.back(), XYbounds_, *config_);
```

`next_node->pose == path.back()`，因此再查一次是纯粹浪费。

**效果**

每次节点扩展都省掉一次 `CheckPoseCollision`。而一次 `CheckPoseCollision` 内部会对车辆的每个碰撞圆盘调用 `CheckBoxCollision`，每次 `CheckBoxCollision` 又包含多边形遍历 + R-tree 空间查询。这是搜索循环里最贵的操作之一，去掉后扩展速度直接受益。

---

## 2. 消除 2D 启发式里的 `shared_ptr` 堆分配

**改动**

把 `Node2d` 的栅格索引与包围盒计算抽成**静态函数**，使启发式搜索可以只拿 `uint64_t` 索引查表，不再为每次查询/每个邻居 `new` 一个对象：

```cpp
// path_planner.h —— Node2d 改为：构造只算一次索引，索引/包围盒逻辑下沉为静态函数
struct Node2d {
    int x_grid, y_grid;
    uint64_t index = 0;
    double f_cost = inf;

    static uint64_t GridIndex(int x_grid, int y_grid);
    static uint64_t GridIndex(math::Pose ps, const std::vector<double>& XYbounds, const PlannerConfig& config);
    static math::AABox2d GenerateBox(int x_grid, int y_grid, const std::vector<double>& XYbounds, const PlannerConfig& config);

    Node2d(math::Pose ps, const std::vector<double>& XYbounds, const PlannerConfig& config)
        : Node2d(static_cast<int>((ps.x - XYbounds[0]) / config.grid_xy_resolution),
                 static_cast<int>((ps.y - XYbounds[2]) / config.grid_xy_resolution)) {}
    Node2d(int x_grd, int y_grd) : x_grid(x_grd), y_grid(y_grd) { index = GridIndex(x_grd, y_grd); }
};
```

`Calculate2DCost` 重写为直接对索引运算，并按「边界 → closed → open → 碰撞」的顺序短路，**只有全新的栅格才分配对象、才做 R-tree 碰撞查询**：

```cpp
double PathPlanner::Calculate2DCost(std::shared_ptr<Node3d> node_3d) {
  const uint64_t target_index = Node2d::GridIndex(node_3d->pose, XYbounds_, *config_);

  auto closed_node = grid_closed_set_.find(target_index);
  if (closed_node != grid_closed_set_.end()) {
    return closed_node->second->f_cost * config_->grid_xy_resolution;
  }

  while (!grid_open_pq_.empty()) {
    const uint64_t current_index = grid_open_pq_.top().first;
    grid_open_pq_.pop();
    auto current_it = grid_open_set_.find(current_index);
    if (current_it == grid_open_set_.end()) { continue; }   // 惰性删除的过期条目
    std::shared_ptr<Node2d> current_node = current_it->second;
    grid_open_set_.erase(current_it);                        // 关闭时即从 open 中移除
    grid_closed_set_.emplace(current_index, current_node);
    ...
    for (int i = 0; i < 8; ++i) {
      const int next_x = current_node_x + grid_directions[i][0];
      const int next_y = current_node_y + grid_directions[i][1];
      if (next_x < 0 || next_x > max_grid_x_ || next_y < 0 || next_y > max_grid_y_) continue;
      const uint64_t next_index = Node2d::GridIndex(next_x, next_y);
      if (grid_closed_set_.count(next_index) > 0) continue;      // 已关闭，不查碰撞
      const double next_f_cost = current_node_f_cost + grid_direction_costs[i];
      auto opened_it = grid_open_set_.find(next_index);
      if (opened_it != grid_open_set_.end()) {                   // 已打开，只更新，不查碰撞
        if (opened_it->second->f_cost > next_f_cost) { ... }
        continue;
      }
      if (GridCellCollides(next_x, next_y)) continue;            // 全新栅格才查碰撞
      auto next_node = std::make_shared<Node2d>(next_x, next_y); // 全新栅格才分配
      ...
    }
    if (current_index == target_index) return current_node_f_cost * config_->grid_xy_resolution;
  }
  return inf;
}
```

**为什么**

原实现中：

- 每次调用 `Calculate2DCost`（即每次扩展一个 3D 节点都要算启发式）都会 `std::make_shared<Node2d>(node_3d->pose, ...)` 一次；
- 每个栅格邻居（每个关闭节点 8 个方向）都会无条件 `std::make_shared<Node2d>` 一次；
- 每个邻居都会**先**做 `GridCheckConstraints`（含 R-tree 碰撞查询），**再**判断 closed/open。

这三点都是纯浪费：查询只用到索引、邻居对象在「已关闭/已打开」时根本不需要新建、碰撞状态在静态环境下不会变所以重复查询无意义。

**效果**

- 启发式部分的内存分配次数从「每次调用 1 次 + 每个邻居 1 次」降到「只有首次进入 open set 的新栅格 1 次」；
- 碰撞查询只对全新栅格执行，已打开/已关闭栅格不再重复做 R-tree 查询；
- `grid_open_set_` 在栅格关闭时即 `erase`，不再无限累积（原来所有被打开过的栅格永远留在 open set 里）。

---

## 3. 把 `tan(steering)` 提到循环外

**改动**

```cpp
// 改动前：每次循环内都算一次 std::tan(steering)
for (int i = 0; i < forward_num_; i++) {
  math::Pose next_pose;
  next_pose.theta = last_pose.theta + step_size / wheel_base * std::tan(steering);
  ...
}

// 改动后：整个函数只算一次
const double dtheta = step_size / config_->vehicle.wheel_base * std::tan(steering);
for (int i = 0; i < forward_num_; i++) {
  const double last_theta = last_pose.theta;
  const double next_theta = last_theta + dtheta;
  const double mid_theta = (last_theta + next_theta) / 2.0;
  last_pose.x += step_size * std::cos(mid_theta);
  last_pose.y += step_size * std::sin(mid_theta);
  last_pose.theta = math::NormalizeAngle(next_theta);
  path[i + 1] = last_pose;
}
```

**为什么**

`steering`、`step_size`、`wheel_base` 在一次 `GenerateKinematicPath` 调用内都是常量，`tan(steering)` 却每步重算了一遍。同时去掉了每步一个临时 `next_pose` 的构造。

**效果**

`forward_num_` 通常大于 1（由 `arc_length / step_size` 决定），`tan` 从「每步 1 次」降到「每次调用 1 次」；`GenerateKinematicPath` 在 `ExpandNextNode` 和最终 `TraversePath` 里都会被反复调用，收益随路径长度累积。

---

## 4. one-shot 频率的除零保护与范围约束

**改动**

```cpp
// 改动前
double scaled_heu_cost = (current_node->f_cost - current_node->g_cost) / dist_start_to_goal;
int oneshot_freq = static_cast<int>(min_oneshot_freq + scaled_heu_cost * (max_oneshot_freq - min_oneshot_freq));

// 改动后
double scaled_heu_cost = dist_start_to_goal > 1e-6
    ? (current_node->f_cost - current_node->g_cost) / dist_start_to_goal
    : 1.0;
int oneshot_freq = static_cast<int>(std::clamp(
    min_oneshot_freq + scaled_heu_cost * (max_oneshot_freq - min_oneshot_freq),
    static_cast<double>(min_oneshot_freq), static_cast<double>(max_oneshot_freq)));
```

**为什么**

- `start` 与 `goal` 重合（或极近）时 `dist_start_to_goal ≈ 0`，原代码会除零得到 inf/nan，`oneshot_freq` 变成未定义行为；
- 启发式（障碍物感知的 2D 距离）恒 ≥ 直线距离，因此 `scaled_heu_cost` 常大于 1，原式会让 `oneshot_freq` 超过 `max_oneshot_freq`（100），与变量命名语义不符。`std::clamp` 把频率约束在 `[min, max]` 内。

**效果**

边界输入下不再产生未定义行为；one-shot 尝试频率被限制在预期区间，行为更可预测。

---

## 5. `GridCheckConstraints` 的类型修正与职责内聚

**改动**

`GridCheckConstraints(std::shared_ptr<Node2d>)` 改为 `GridCellCollides(int x_grid, int y_grid)`：

```cpp
bool PathPlanner::GridCellCollides(int x_grid, int y_grid) const {
  if (x_grid < 0 || x_grid > max_grid_x_ || y_grid < 0 || y_grid > max_grid_y_) {
    return true;
  }
  return env_->CheckBoxCollision(0.0, Node2d::GenerateBox(x_grid, y_grid, XYbounds_, *config_));
}
```

**为什么**

- 原函数把 `int` 的 `x_grid/y_grid` 赋给 `const double`，是无意义的隐式类型提升；
- 改为纯整数入参后，不再依赖一个完整 `Node2d` 对象，配合第 2 点实现了「先索引判断、后碰撞查询」。

**效果**

去掉无意义的 double 转换，接口更贴合「判断某个栅格是否不可用」的语义，同时支持无分配调用。

---

## 6. 清理死代码与拼写修正

**改动**

- 删除无法编译的可视化调试块 `#ifdef VISUALIZE_NODE_EXPANSION` / `#ifdef VISUALIZE_GRID_MAP`（引用了不存在的 `node`、`origin_`、已注释掉的 `is_closed`）；
- 删除不再使用的 include：`liom_local_planner/time.h`、`liom_local_planner/visualization/plot.h`；新增 `#include <algorithm>`（`std::clamp`）、`#include <cstdlib>`（`std::abs(int)`）；
- 删除未使用的死变量 `best_explored_num` 及其连带注释；
- 修正参数名拼写 `currend_node` → `current_node`。

**为什么**

这些 `#ifdef` 块即使手动打开也无法编译（引用未定义符号），属于纯噪声；`best_explored_num` 赋值后从未使用，会触发 `-Wunused-variable`。

**效果**

代码更干净，消除编译告警；不影响任何运行行为。

---

## 验证

- 语法检查：用项目原有编译标志 `-fsyntax-only` 通过（无错误、无新增告警）。
- 完整构建：`colcon build --packages-select liom_local_planner` 成功（`Finished <<< liom_local_planner`），仅剩的告警来自无关的 `lightweight_nlp_problem.cpp`。

---

## 未改动的部分（保持语义一致）

- 混合 A* 的节点扩展、`forward_num_`/`arc_length` 计算、代价函数 `EvaluateExpandCost` 的权重结构；
- 2D Dijkstra 启发式的传播方向与距离度量（8 邻域、`1`/`M_SQRT2` 代价、`grid_xy_resolution` 缩放）；
- Reeds-Shepp one-shot 生成逻辑；
- `Node3d` 的位索引打包方式。

以上优化聚焦于**去掉热路径上的冗余计算与堆分配**，规划结果应与优化前一致，仅运行更快、更省内存。
