# Eigen + OSQP 关节速度 QP

独立 C++17 库和命令行程序，使用 Eigen 及 OSQP 1.x 的原生 API，无 ROS、Python 或 OsqpEigen 运行时依赖。输入雅可比矩阵、期望末端速度和约束，输出满足约束容差的关节速度。

## 数学模型

令 `x = dq` 为关节速度，`J ∈ R^(m×n)`，`v ∈ R^m`：

```text
minimize  1/2 (J*x - v)^T W (J*x - v)
        + λ/2 ||x - x_ref||²

subject to
  dq_min <= x <= dq_max
  q_min  <= q + dt*x <= q_max                 （可选）
  a_min  <= (x - dq_previous)/dt <= a_max      （可选）
  b_min  <= C*x <= b_max                      （可选）
```

`W` 是非负对角权重矩阵，默认单位阵；`λ > 0` 默认 `1e-4`；参考速度 `x_ref` 默认零。正则项让冗余、秩亏雅可比下的问题仍具有唯一最优解。期望末端速度是软目标，受限时可以存在跟踪误差。若需要将某些末端速度分量设为等式约束，可通过 `C` 的相应行及相等的上下界表达。

转换到 OSQP 的标准形式：

```text
minimize  1/2 x^T P x + c^T x
subject to l <= A*x <= u

P = J^T W J + λ I
c = -J^T W v - λ x_ref
A = [ I; C ]

l_joint = max(dq_min, (q_min-q)/dt, dq_previous + dt*a_min)
u_joint = min(dq_max, (q_max-q)/dt, dq_previous + dt*a_max)
l = [ l_joint; b_min ]
u = [ u_joint; b_max ]
```

仅对已启用的约束计算交集。位置约束使用一步欧拉预测；加速度约束限制相邻两个速度指令的差分。`dt` 以秒为单位；旋转关节的位置、速度、加速度通常分别用 rad、rad/s、rad/s²。`J` 和 `v` 必须采用一致的坐标系、单位和分量顺序；常用六维顺序是 `[vx, vy, vz, wx, wy, wz]`。可通过 `task_weights` 调整平移和旋转分量的权重。

## 构建和运行

需要 CMake ≥ 3.18、C/C++ 编译器及 Eigen ≥ 3.3。Ubuntu 可先安装：

```bash
sudo apt install build-essential cmake libeigen3-dev
```

从仓库根目录执行：

```bash
cmake -S joint_velocity_qp -B build/joint_velocity_qp -DCMAKE_BUILD_TYPE=Release
cmake --build build/joint_velocity_qp -j2
ctest --test-dir build/joint_velocity_qp --output-on-failure

# 七关节、六维任务示例，包含位置、速度、加速度和线性约束
./build/joint_velocity_qp/joint_velocity_qp_cli --demo

# 从标准输入读取 J、v 和速度上下限
./build/joint_velocity_qp/joint_velocity_qp_cli < joint_velocity_qp/examples/input.txt
```

若未找到已安装的 OSQP 1.x，CMake 默认联网下载固定版本 OSQP 1.0.0、QDLDL 0.1.8 并校验 SHA-256。只下载到构建目录，不安装系统包。OSQP 必须采用双精度构建，即 `OSQP_USE_FLOAT=OFF`。

使用已有 OSQP 安装、关闭下载：

```bash
cmake -S joint_velocity_qp -B build/joint_velocity_qp \
  -DCMAKE_BUILD_TYPE=Release \
  -DJOINT_VELOCITY_QP_FETCH_OSQP=OFF \
  -Dosqp_DIR=/path/to/osqp/lib/cmake/osqp
```

本实现采用 [OSQP 1.x API](https://osqp.org/docs/interfaces/C.html)，不能直接与 0.6.x API 混用。

## 命令行输入输出

标准输入由空白分隔，顺序为：

```text
m n
J 的 m 行，每行 n 个数
v 的 m 个数
dq_min 的 n 个数
dq_max 的 n 个数
```

例如 [examples/input.txt](examples/input.txt)：

```text
2 3
1 0 1
0 1 1
1 0.5
-0.2 -0.2 -0.2
0.2 0.2 0.2
```

三个关节均达到速度上限，输出约为：

```text
0.2 0.2 0.2
```

标准输出仅包含求解成功后的 `n` 个关节速度；标准错误输出状态、迭代次数、跟踪误差和最大约束违反量。退出码 `0` 表示成功，`1` 表示求解失败或不可行，`2` 表示输入无效。上下界支持 `-inf` / `inf`；输入不能包含注释。命令行限制 `m*n <= 1000000`；额外约束通过 C++ API 设置。

## C++ 接口

```cpp
#include <joint_velocity_qp/solver.hpp>
#include <iostream>

namespace qp = joint_velocity_qp;

void example(const Eigen::MatrixXd& J, const Eigen::VectorXd& v) {
  const Eigen::Index n = J.cols();
  qp::Problem p;
  p.jacobian = J;
  p.desired_velocity = v;
  p.velocity_limits = {Eigen::VectorXd::Constant(n, -0.5),
                       Eigen::VectorXd::Constant(n,  0.5)};

  qp::Options options;
  options.regularization = 1e-4;
  const qp::Result result = qp::solve(p, options);
  if (result.ok()) {
    const Eigen::VectorXd dq = result.joint_velocity;
    std::cout << dq.transpose() << '\n';
  } else {
    std::cerr << qp::statusName(result.status) << ": " << result.message << '\n';
  }
}
```

如需启用额外约束，在 `solve()` 前设置：

```cpp
p.dt = 0.001;
p.position_limits = qp::PositionLimits{q, {q_min, q_max}};
p.acceleration_limits = qp::AccelerationLimits{dq_previous, {a_min, a_max}};
p.linear_constraints = qp::LinearConstraints{C, {b_min, b_max}};

// 可选：每个任务分量的非负权重、关节参考速度
p.task_weights = task_weights;
p.reference_velocity = dq_ref;
```

在其他 CMake 项目中复用：

```cmake
add_subdirectory(/path/to/kaixuan/joint_velocity_qp joint_velocity_qp_build)
target_link_libraries(your_target PRIVATE joint_velocity_qp::joint_velocity_qp)
```

实现位于 [src/solver.cpp](src/solver.cpp)，公开接口位于 [include/joint_velocity_qp/solver.hpp](include/joint_velocity_qp/solver.hpp)。

## 求解状态和验证范围

- 输入维度错误、NaN、非法权重、非正时间步或正则系数：返回 `InvalidInput`。
- 速度、位置和加速度约束交集为空，或 OSQP 判定原问题不可行：返回 `Infeasible`。
- 迭代超限或精度不足：返回 `IterationLimit` / `Inaccurate`。
- 仅接受 `OSQP_SOLVED`；之后独立复核所有组装后的 `l <= A*dq <= u`，超过 `constraint_tolerance` 返回 `ConstraintViolation`。
- 失败结果中的速度向量为空；调用方必须检查 `result.ok()`。程序不会将零速度当作不可行问题的替代解。

默认 OSQP 绝对、相对容差均为 `1e-7`，额外约束复核的绝对容差为 `1e-6`，作用于转换后的各行约束值。返回的是数值容差内的可行解；正则项及受限任务可能产生非零跟踪误差。

测试覆盖带权最小二乘解析解、速度饱和、奇异雅可比、位置与加速度约束、耦合不等式、等式、不可行约束、非法输入和迭代超限。

每次 `solve()` 都建立并释放 OSQP 工作区，适合作为独立求解程序及集成起点；高频控制时可进一步复用工作区、固定稀疏结构并更新数据以减少分配和分解开销。当前没有实体机器人运动测试。
