# 论文对齐修复说明（解决 RViz 仿真结果不稳定）

> 涉及文件：
> - [src/liom_local_planner/src/liom_local_planner.cpp](../../src/liom_local_planner/src/liom_local_planner.cpp)（规划器主流程）
> - [src/liom_local_planner/include/liom_local_planner/liom_local_planner.h](../../src/liom_local_planner/include/liom_local_planner/liom_local_planner.h)
> - [src/liom_local_planner/src/lightweight_nlp_problem.cpp](../../src/liom_local_planner/src/lightweight_nlp_problem.cpp)（IPOPT NLP 问题）
> - [src/liom_local_planner/include/liom_local_planner/planner_config.h](../../src/liom_local_planner/include/liom_local_planner/planner_config.h)
>
> 未参考 `coarse_path_planner`，未查看 `dont_need` 目录。本轮目标不是性能，而是**行为正确性**：逐条对照论文《Optimization-Based Trajectory Planning for Autonomous Parking With Irregularly Placed Obstacles: A Lightweight Iterative Framework》（Bai Li et al., IEEE TITS 2022），找出代码实现与论文不一致、或实现不合理的地方并修复。

---

## 根因：只实现了 STC 单趟方法，缺失论文的核心贡献——迭代走廊优化框架（Alg. 3）

论文把两类方法区分得很清楚：

- **STC（SFC-trajectory construction）**：只根据初始猜测建一次安全飞行走廊（SFC），解一次 OCP。论文明确指出 STC **依赖初始猜测**：当初始猜测较差时，生成的走廊要么漏掉了自由空间、要么根本没覆盖任何运动学可行的轨迹，于是求解器在错误的走廊里求解，结果差甚至不可行。
- **轻量迭代框架（Alg. 3）**：建走廊 → 解盒约束 OCP → 用解出的轨迹计算不可行度 `ψ_infeasibility` → 从当前解重建走廊 → 以当前解热启动再解，循环直到 `ψ_infeasibility < ε_tol`。这才是论文的贡献点，也是它能从坏初值恢复的原因。

**原代码的问题**：`Plan()` 里只做了「建一次走廊 → 解一次」，正是 STC。这就是为什么 RViz 里「有时候混合 A* 能生成初始路径、但 IPOPT 优化不出轨迹 / 结果很差 / 有时候又可以」——完全取决于初始猜测是否恰好落在「好走廊」里。

**修复**：把 `Plan()` 重构为论文 Alg. 3 的迭代循环：

```cpp
double infeasibility = inf;
bool converged = false;
for (int iter = 0; iter < config_->opti_iter_max; iter++) {
  if (!BuildCorridors(guess, constraints)) { ... return false; }
  if (!problem_->Solve(config_->opti_w_penalty0, constraints, guess, result, infeasibility)) { ... return false; }
  if (infeasibility <= config_->opti_varepsilon_tol) { converged = true; break; }
  guess = result;  // 热启动下一轮
}
```

- 把原来内联的走廊生成代码抽成 `BuildCorridors()`（`const` 方法，每次迭代清空上一轮的走廊 marker 再重铺）。
- 新增配置 `opti_iter_max`（默认 5）：迭代轮数上限；不可行度阈值复用已有的 `opti_varepsilon_tol`。
- 每轮用上一轮的解热启动（`guess = result`），走廊基于最新轨迹重建，逐步把走廊收敛到真正可行的轨迹附近。

**效果**：坏初值不再「一次性定生死」。即便第一轮在差走廊里解出的轨迹不可行，后续轮会基于该轨迹重建更贴近的走廊再解，从而让 IPOPT 稳定收敛，消除「时好时坏」的随机性。

---

## 二、时间步长 off-by-one：`dt = T/nfe` → `dt = T/(nfe-1)`

`eval_infeasibility()` 里前向欧拉离散的步长原为：

```cpp
T dt = x[0] / nfe_;
```

但变量布局是 `tf + [x y θ v φ a ω 圆盘]×nrows + θ_end`，其中 `nrows = nfe - 2`，即链 `start → state_1 → ... → state_{nfe-2} → goal` 一共含 **`nfe-1` 个前向欧拉步**。用 `T/nfe` 会让真实视野被压缩成 `T·(nfe-1)/nfe`，与初始猜测的 `tf` 不匹配。

```cpp
// 改动后
T dt = x[0] / (nfe_ - 1);
```

**效果**：离散动力学与初始猜测时间尺度一致，运动学残差不再是系统性偏大，`ψ_infeasibility` 的度量才准确，收敛判据才可靠。

---

## 三、目标函数补齐论文式(10)的 `v²` 因子

论文式(10)：`J = w·∫(a² + v²·ω²)dτ + T`。原代码把 `v²` 漏掉了：

```cpp
// 改动前
obj_value += config_.opti_w_a * a*a + config_.opti_w_omega * omega*omega;

// 改动后
obj_value += config_.opti_w_a * a*a + config_.opti_w_omega * v*v * omega*omega;
```

**效果**：横摆角速度的惩罚改为与车速耦合——高速时抑制大横摆（更符合物理直觉），低速时（倒车/换挡）允许大转向，避免低速段被 `ω` 项过度惩罚导致轨迹被压得过于保守。

---

## 四、初始猜测里零速度处的 `phi` 置 NaN 守卫

`ResamplePath()` 用运动学关系 `θ̇ = v·tanφ/LW` 反解 `φ = atan((Δθ·LW)/(v·dt))`。换挡/尖点（cusp）处 `v≈0`，分母趋零，`atan(±inf/NaN)` 会把 `inf/NaN` 直接灌进 IPOPT 初值点，是「混合 A* 有解但 IPOPT 直接失败」的一个直接触发点。

```cpp
if (std::abs(result.states[i].v) > 1e-6) {
  result.states[i].phi = std::clamp(..., atan((Δθ * LW) / (v * dt)), ...);
} else {
  result.states[i].phi = 0.0;
}
```

**效果**：初值点不再含 `NaN/inf`，IPOPT 从干净的起点出发，避免因坏初值立即失败。

---

## 五、过短的拼接后缀回退到混合 A* 重规划

`StitchPreviousSolution()` 在车辆接近走完上一条轨迹时，裁剪出的后缀可能只剩 1~2 个状态。此时 NLP 内部 `nrows = nfe - 2 ≤ 0`，会造成负尺寸数组/异常。加守卫：

```cpp
if(!CheckGuessFeasibility(guess) || guess.states.size() < static_cast<size_t>(config_->min_nfe)) {
  // 走 hybrid A* 完整重规划
}
```

**效果**：保证进入求解器的离散状态数始终 ≥ `min_nfe`，消除边界情况下的崩溃/异常，并让过短的拼接结果自然退回完整重规划。

---

## 验证

- 完整构建 `colcon build --packages-select liom_local_planner` 成功，**无源码 warning、无 error**（唯一的 stderr 为 Boost `<boost/detail/no_exceptions_support.hpp>` 弃用提示，来自第三方头文件，与本项目无关）。
- `eval_infeasibility` 的 `template<class T>` 同时以 `double`/`adouble` 实例化，编译通过，说明 `dt = T/(nfe-1)` 与 `v*v*omega*omega` 对两种类型都成立。

---

## 未改动的部分

- 自行车运动学模型、前向欧拉离散形式、软约束结构与权重；
- 优化变量布局 `tf + [x y θ v φ a ω 圆盘坐标]×nrows + θ_end` 与全部下标映射（除 `dt` 步长修正）；
- IPOPT 参数与 `opti_w_penalty0`（`w_penalty` 初值）；
- 走廊生成 `GenerateCorridorBox` 的盒扩张策略（属于 environment）。

本轮只修「与论文不一致 / 导致行为不稳定的逻辑」，不涉及纯性能优化。
