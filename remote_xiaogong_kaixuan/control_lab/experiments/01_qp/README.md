# 阶段 01：二维 QP

本阶段使用无量纲二维变量，学习目标函数、约束和数值验收。它独立于机械臂仿真，
没有机器人控制输出。对应实现是 `kaixuan_control_lab/qp2d.py`，实验入口是
`kaixuan_control_lab/run_qp2d.py`。以下命令均从 `control_lab/` 执行。

## 运行与产物

```bash
conda activate mujoco_test
python -m pip install -r requirements-qp.txt
python -m unittest discover -s experiments/01_qp/tests -v
python -m kaixuan_control_lab.run_qp2d
```

本机环境已迁移到 Conda；新机器可先从本目录运行 `conda env create -f environment.yml`。

图使用 Agg 后端，无窗口也能运行。可用 `--output artifacts/qp01_my_run` 指定新目录；
为避免混合两轮结果，目录已经存在时会拒绝运行。

默认输出到 `artifacts/qp2d_<UTC时间>/`：

| 文件 | 内容 |
| --- | --- |
| `geometry.png` | 内点、斜边、顶点、负向边界四种情况的可行域与代价等高线 |
| `sweep.png` | 目标连续移动时，解与拉格朗日乘子的变化 |
| `admm.png` | 教学 ADMM 的原始/对偶残差及坐标收敛 |
| `solutions.csv` | 235 个样本的解、独立参考、误差、活跃约束、乘子、KKT、求解状态与耗时 |
| `admm_trace.csv` | 自写教学 ADMM 每次迭代的数据；不是 OSQP 内部迭代日志 |
| `failure_cases.json` | 不可行、迭代超限的状态与被拒绝结果 |
| `summary.json` | 验收结果、参数、种子、精确依赖版本、源码哈希、耗时分位数 |

235 个样本包括：101 个连续目标、6 个特定目标和 128 个固定种子的随机目标。
每个样本建立新的 OSQP 工作区，未使用 warm start。耗时包含 Python 包装的单独统计，
也记录求解器 setup/solve/run 耗时；这组数据不能用于声明机器人实时能力。

## 1. 变量、目标和约束

决策变量是 `z=[z1,z2]`，给定目标为 `r=[r1,r2]`：

\[
\min_z f(z)=\frac12(z-r)^TH(z-r),\quad H=\operatorname{diag}(2,1),
\qquad -1\le z_1,z_2\le1,\quad z_1+z_2\le0.8.
\]

`f=(z1-r1)²+0.5*(z2-r2)²`，第一维偏离相同距离时惩罚更大。
梯度是 `H(z-r)`，Hessian 是 H。H 的特征值为 2 和 1，严格正定，
因此目标严格凸；约束是半空间的交集，也是凸集。只要可行，最优解唯一。

无约束解是 z=r。r 可行时无需改变；不可行时寻找 H 度量下离 r 最近的可行点。
分别裁剪两维不能处理 `z1+z2<=0.8` 这样的耦合约束。

## 2. 写成 OSQP 输入

展开并省略与 z 无关的常数 `0.5*rᵀHr`：

\[
\min_z \frac12z^TPz+c^Tz,\qquad P=H,\quad c=-Hr.
\]

本实现把所有限制写成单侧形式 `Cz<=d`：

```text
C = [ 1  0 ]       d = [ 1   ]
    [-1  0 ]           [ 1   ]
    [ 0  1 ]           [ 1   ]
    [ 0 -1 ]           [ 1   ]
    [ 1  1 ]           [ 0.8 ]
```

传入 OSQP 的是 `A=C, l=[-inf]*5, u=d`，P、A 使用 CSC 稀疏矩阵。
OSQP Python 参数名 `q` 表示这里的线性代价 c，与机器人关节位置 q 无关。
CSV 的 objective 使用完整 f(z)，不是省略常数后的求解器目标。

## 3. KKT 与独立参考解

令 `lambda>=0`，拉格朗日函数是 `L=f(z)+lambdaᵀ(Cz-d)`：

```text
驻点：    H(z-r) + Cᵀlambda = 0
原始可行：Cz-d <= 0
对偶可行：lambda >= 0
互补：    lambda_i*(Cz-d)_i = 0
```

对本凸问题，满足 KKT 的候选就是最优解。独立参考实现不调用任何优化器：
枚举 0、1、2 条线性独立的活跃约束，解下式，再检查全部约束和乘子符号：

```text
[ H    C_Iᵀ ] [ z        ] = [ H*r ]
[ C_I  0    ] [ lambda_I ]   [ d_I ]
```

二维最优点的法向量组合可由至多两条独立法向量表示。奇异组合跳过，
其余约束的乘子设为零；通过 KKT 的候选按原始目标值选择。

手算 `r=(1,1)`：只有斜边活跃，得到
`2*(z1-1)+lambda=0`、`z2-1+lambda=0`、`z1+z2=0.8`，
解为 `lambda=0.8, z=(0.6,0.2)`。它更愿意牺牲惩罚较小的第二维。

`r=(2,1)` 时最优点为 `(1,-0.2)`，`z1_upper` 和 `sum_upper` 的乘子分别为 0.8 和 1.2。
注意几何上活跃只表示约束取等号，不保证乘子严格为正；`r=(2,1.8)` 是一个零乘子边界例子。
乘子还表示该约束上界微小放宽时最优代价的一阶变化率为 `-lambda`。

## 4. ADMM 在迭代什么

引入辅助变量 v，把 `Cz<=d` 拆成 `Cz=v`、`v<=d`。下面是本实验的
未松弛、固定 rho 教学迭代，取 `rho=1, sigma=1e-6`：

```text
(H + sigma*I + rho*CᵀC) z_next
    = sigma*z + H*r + Cᵀ(rho*v-y)
v_next = min(C*z_next + y/rho, d)     # 逐元素投影
y_next = y + rho*(C*z_next-v_next)
```

z 步处理二次目标，v 步处理可行范围，y 累积两边不一致的量。
rho 和 sigma 是算法参数；它们不改变本实验最终要优化的 H 和 r。

图中原始残差是 `||Cz-v||_inf`，反映两份变量的一致性；对偶残差是
`||H(z-r)+Cᵀy||_inf`，反映驻点条件。迭代中的 z 可以暂时违反原约束，
不能把尚未收敛的中间值当作可用输出。
该实现用于观察收敛；OSQP 还包含缩放、松弛、自适应 rho 和稀疏线性求解等处理。

OSQP 自报的原始残差与直接计算的 `max(Cz-d,0)` 是不同的量，CSV 分别记录。
独立验收另外计算原始违反量、驻点、对偶违反量和互补残差，全部采用无穷范数。

## 5. 验收与失败行为

- 所有可行样本只接受精确的 `OSQP_SOLVED` 状态；`solved inaccurate` 不接受。
- 解与乘子必须维度正确、数值有限，并通过独立 KKT 绝对容差 `1e-7`。
- 与枚举参考解的最大坐标误差必须小于 `1e-6`。
- 将和约束改为 `z1+z2<=-2.5`；盒约束决定其最小值为 -2，问题不可行。
- 把可行问题的 `max_iter` 设为 1，验证迭代超限路径。
- 两种失败都保留状态、残差和耗时，同时令公开解为 `None`，不会用零向量代替。
- 教学 ADMM 在 300 次迭代后到达 `(0.6,0.2)`，误差及原始/对偶残差小于 `1e-7`。

单元测试还使用手算解、64 个随机目标，以及伪造的“solved 但数值不合格”结果，
检查不能仅凭求解器状态放行。验收失败时实验命令返回非零退出码。

本阶段完成后，再按 ROADMAP 阶段 02 做多关节末端直线跟踪与外力扰动实验。

参考：[OSQP 算法与残差](https://osqp.org/docs/solver/index.html)、
[Python 接口](https://osqp.org/docs/interfaces/python.html)、
[状态码](https://osqp.org/docs/interfaces/status_values.html)。
