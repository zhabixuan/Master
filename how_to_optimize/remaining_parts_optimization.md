# liom_local_planner 剩余部分优化说明

> 涉及文件：
> - [src/liom_local_planner/src/lightweight_nlp_problem.cpp](../../src/liom_local_planner/src/lightweight_nlp_problem.cpp)（IPOPT NLP 问题）
> - [src/liom_local_planner/src/liom_local_planner.cpp](../../src/liom_local_planner/src/liom_local_planner.cpp)（规划器主流程）
> - [src/liom_local_planner/include/liom_local_planner/optimizer_interface.h](../../src/liom_local_planner/include/liom_local_planner/optimizer_interface.h)
>
> 未参考 `coarse_path_planner`，未查看 `dont_need` 目录。优化聚焦于 **IPOPT 求解器热路径的 ADOL-C tape 瘦身**、**去掉走廊生成/初值构造里的堆分配**、**清理热路径日志与死代码**。规划与求解的数学表达式保持不变。

---

## 一、lightweight_nlp_problem.cpp

### 1. 把 `pow(d, 2)` 全部改写为 `d*d`（收益最大）

**改动**

`eval_infeasibility()` 是软约束残差平方和，被 `eval_obj` 在 IPOPT 的 `eval_f` 和 ADOL-C tape 生成时反复调用。原实现里每一个平方项都写成了 `pow(表达式, 2)`，本次全部改写为 `d*d`，并把每项中间量抽取为局部变量：

```cpp
// 改动前
infeasibility += pow(x[1 + (0) * nrows_ + i] - x[1 + (0) * nrows_ + i-1]
                     - dt * x[1 + (3) * nrows_ + i-1] * cos(x[1 + (2) * nrows_ + i-1]), 2) + ...;

// 改动后
const T v_prev  = x[1 + 3 * nrows_ + i-1];
const T th_prev = x[1 + 2 * nrows_ + i-1];
d_x   = x[1 + 0 * nrows_ + i] - x[1 + 0 * nrows_ + i-1] - dt * v_prev * cos(th_prev);
...
infeasibility += d_x*d_x + d_y*d_y + d_th*d_th + d_v*d_v + d_phi*d_phi;
```

**为什么**

`eval_infeasibility` 是 `template<class T>`，同时以 `double` 和 `adouble` 实例化。对 `adouble`，`pow(x, 2)` 会在 tape 上展开成 `exp(2*log(x))` 这种多操作码的复合运算，而 `x*x` 只是一次乘法。`eval_infeasibility` 里约 30 处平方项，`eval_obj` 每次调用都会走一遍，tape 生成时也要 trace 一遍——这直接决定 IPOPT 每次迭代里目标函数、梯度、Hessian 的 tape 求值成本。

同时把 `v_prev`/`th_prev`/`c_last`/`s_last` 等重复子表达式提出来，进一步减少 tape 上的冗余操作数（例如盘耦合项里 `cos(theta)`/`sin(theta)` 原来每个圆盘各算一次，现在每行只算一次）。

**效果**

- 目标函数 tape 的活跃操作数显著下降（去掉约 30 组 `pow/exp/log`，换成单次乘法）；
- IPOPT 每步的 `eval_f`、稀疏 Hessian 求值都直接受益，收敛迭代次数越多收益越大；
- 数学表达式完全等价（平方还是平方），求解结果不变。

### 2. `get_starting_point` 去掉 `GetDiscPositions` 的堆分配

**改动**

```cpp
// 改动前
auto x0_disc = config_.vehicle.GetDiscPositions(guess_.states[i+1].x,
                                                guess_.states[i+1].y,
                                                guess_.states[i+1].theta);
x0_disc_mat.row(i) = Eigen::Map<Eigen::VectorXd>(x0_disc.data(), x0_disc.size());

// 改动后
const auto &st = guess_.states[i+1];
const double c = std::cos(st.theta);
const double s = std::sin(st.theta);
for (int j = 0; j < config_.vehicle.n_disc; j++) {
    x0_disc_mat(i, 2 * j)     = st.x + config_.vehicle.disc_coefficients[j] * c;
    x0_disc_mat(i, 2 * j + 1) = st.y + config_.vehicle.disc_coefficients[j] * s;
}
```

**为什么**

`GetDiscPositions` 每次调用都 `new` 一个 `std::vector`，`get_starting_point` 又会被 `generate_tapes` 内部再调用一次，总共 `2 * nrows_` 次分配。这里其实只是把圆盘坐标写进已经映射好的 Eigen 块里，完全可以直接用 `disc_coefficients` 内联计算。

**效果**

每次 Solve 省掉 `2 * nrows_` 次堆分配和对应的 `Eigen::Map` 临时对象；初值点数值不变。

### 3. `Solve` 的 `std::cout` 改为 RCLCPP 日志

**改动**

```cpp
// 改动前
std::cout << "wall_t: " << GetCurrentTimestamp() - solver_st
          << ", status: " << status
          << ", infeasibility: " << infeasibility
          << ", tf: " << result.tf << std::endl;

// 改动后
RCLCPP_INFO(rclcpp::get_logger("lightweight_nlp_problem"),
    "wall_t: %.3f, status: %d, infeasibility: %.6f, tf: %.3f",
    GetCurrentTimestamp() - solver_st, static_cast<int>(status), infeasibility, result.tf);
```

**为什么**

该文件其余地方都用 `RCLCPP_*`，只有这一处直接 `std::cout`，既不经过 ROS 日志体系、也不受日志级别控制。统一为 RCLCPP 后行为一致，且信息（耗时/状态/不可行度/终止时间）仍完整保留。

### 4. 消除未使用参数的编译告警

`eval_constraints` 的四个参数在 `m == 0`（无硬约束）时均未使用，加上 `(void)` 显式标记；`<cmath>` 显式 include。

---

## 二、liom_local_planner.cpp

### 1. 走廊生成循环内联圆盘位置，去掉 `GetDiscPositions` 堆分配

**改动**

```cpp
// 改动前
auto disc_pos = config_->vehicle.GetDiscPositions(guess.states[i].x,
                                                  guess.states[i].y,
                                                  guess.states[i].theta);
for(int j = 0; j < config_->vehicle.n_disc; j++) {
  if (!env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], ...)) { ... }
  ...
}

// 改动后
const double c = std::cos(guess.states[i].theta);
const double s = std::sin(guess.states[i].theta);
for(int j = 0; j < config_->vehicle.n_disc; j++) {
  const double disc_x = guess.states[i].x + config_->vehicle.disc_coefficients[j] * c;
  const double disc_y = guess.states[i].y + config_->vehicle.disc_coefficients[j] * s;
  if (!env_->GenerateCorridorBox(0.0, disc_x, disc_y, ...)) { ... }
  ...
}
```

**为什么**

这是 `Plan()` 里对每个 guess 状态（通常几十到上百个）执行的循环，`GetDiscPositions` 每状态一次堆分配。圆盘坐标只用一次即可，直接内联即可，同时把 `cos/sin(theta)` 提到 j 循环外（原实现每个圆盘各算一遍）。

**效果**

每状态省一次 `std::vector` 分配，且 `cos/sin` 从每圆盘一次降为每状态一次。

### 2. 删除走廊生成内层循环的热路径日志

**改动**

删除了 `for` 双层循环内每次成功生成一个走廊盒都打的一条：

```cpp
RCLCPP_INFO(logger, "%d th corridor box indexed at %zu generation had generated!", j, i);
```

**为什么**

这条日志在 `n_disc × 状态数` 的循环内，一次规划会打出几十到几百行；真正有用的汇总信息（成功/失败、总耗时）在循环外已有。RCLCPP 即使日志级别过滤掉，也会先做参数格式化，属于纯开销。

### 3. 删除一次性调试日志

删除 `Plan()` 里三条无信息的临时日志：

```cpp
RCLCPP_INFO(logger, "Calling visualization::Plot...");
RCLCPP_INFO(logger, "Calling visualization::Trigger...");
RCLCPP_INFO(logger, "coarse path had published!---------------");
```

保留 `coarse path generation time` 这条有用的计时日志；`Plot`/`Trigger` 的实际发布调用保留。

### 4. `std::vector<bool>` → `std::vector<uint8_t>`

**改动**

`ResamplePath()` 中档位标志 `gears` 从 `std::vector<bool>` 改为 `std::vector<uint8_t>`，并把 `gears.back()` 的赋值改为显式下标：

```cpp
std::vector<uint8_t> gears(path.size());
...
if (gears.size() > 1) {
  const size_t last = gears.size() - 1;
  gears[last] = gears[last - 1];
}
```

**为什么**

`std::vector<bool>` 是位压缩特化，`operator[]` 返回的是代理对象而非 `bool&`，是著名的易踩坑点；同时位运算访问反而比普通字节更慢。这里只有「赋值 + 比较」两种用法，改成 `uint8_t` 语义不变、更安全、更直观，也顺带消除 `-Wstringop-overflow` 对 `vector::back()` 的误报。

### 5. 删除约 40 行已废弃的注释代码

`GenerateOptimalTimeProfileSegment()` 在 `return` 之后残留一整段被注释掉的旧版速度曲线实现（约 40 行），已整体删除。

### 6. 修复 `size_t` / `int` 有符号比较告警

`ResamplePath()` 和 `GenerateOptimalTimeProfileSegment()` 里 `for (size_t i = 0; i < nfe - 1; i++)` 这类把 `size_t` 与 `int`（`nfe`、`N`）比较的循环，统一改为 `int` 下标，消除 `-Wsign-compare`。

---

## 三、optimizer_interface.h

`IOptimizer::Solve` 默认实现（始终返回 `false` 的占位）给未使用参数加上 `(void)` 标记，消除 `-Wunused-parameter`。

---

## 验证

- 完整构建：`colcon build --packages-select liom_local_planner` 成功，**无 warning、无 error**（此前 `lightweight_nlp_problem.cpp` 与 `liom_local_planner.cpp` 各有若干未使用参数/符号比较告警，现已全部清零）。
- ADOL-C 的 `eval_infeasibility` 模板同时以 `double` 和 `adouble` 实例化，编译通过，说明 `pow→*` 改写对两种类型都成立。

---

## 未改动的部分（保持语义一致）

- 自行车运动学模型、前向欧拉离散、软约束（罚函数）的结构与权重；
- 优化变量布局 `tf + [x y θ v φ a ω 圆盘坐标]×nrows + θ_end` 与所有下标映射；
- IPOPT 参数（`print_level`、`max_iter`、`mumps` 等）与收敛判据；
- 时间最优速度曲线 `GenerateOptimalTimeProfileSegment` 的加速/匀速/减速算法本身；
- 走廊生成 `GenerateCorridorBox` 的扩张策略（属于 environment，上一轮已优化）。

以上改动只针对**热路径的冗余计算、堆分配、日志与死代码**，规划/求解的输入输出应与此前完全一致，仅运行更快、日志更干净、编译更干净。
