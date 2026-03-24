# Whole-Body MPC for Loco-Manipulation: 代码结构与功能笔记

这份笔记结合论文 [2511.19709v1.pdf](./2511.19709v1.pdf) 和仓库代码，对 `wb-mpc-locoman` 的整体结构、核心模块、算法实现方式和阅读顺序做一个系统说明。

## 1. 这个仓库本质上在做什么

这个仓库不是完整的机器人软件栈，而是论文中核心 MPC 建模和求解部分的研究代码实现。

论文的核心思想是：

- 直接在全身动力学上做 torque-level MPC。
- 不再采用“简化模型规划 + 低层 whole-body controller 跟踪”的传统两层结构。
- 用 `Pinocchio` 提供刚体动力学和运动学符号表达。
- 用 `CasADi` 建立 NLP 并自动求导。
- 用 `Fatrop` 作为主求解器实现实时求解。

从代码实现角度看，这个仓库主要完成四件事：

1. 加载机器人模型和约束。
2. 构建不同动力学层级的 OCP。
3. 用非线性优化求解 MPC。
4. 把预测出的状态、接触力、力矩做可视化，或者导出成可部署的求解器函数。

## 2. 仓库目录结构

### 根目录关键文件

- `main.py`
  - 运行入口，配置机器人、目标、步态、求解器，然后反复求解 MPC。
- `args.py`
  - 定义不同动力学模型和不同求解器的配置。
- `README.md`
  - 项目简介和基础使用说明。
- `2511.19709v1.pdf`
  - 对应论文。
- `environment.yaml`
  - Conda 环境定义。

### `optimization/`

这是仓库最核心的目录，负责把动力学模型组装成一个最优控制问题。

- `ocp.py`
  - 抽象基类，封装所有 OCP 的公共逻辑：
  - 参数定义
  - 公共目标函数
  - 公共约束
  - warm start
  - solver 初始化
  - codegen 接口
- `ocp_factory.py`
  - 根据字符串创建对应的 OCP 类实例。
- `ocp_whole_body_rnea.py`
  - 论文主方法，对应 whole-body inverse dynamics MPC。
- `ocp_whole_body_aba.py`
  - forward dynamics 基线。
- `ocp_whole_body_acc.py`
  - acceleration-level whole-body 模型。
- `ocp_centroidal_vel.py`
  - centroidal momentum + velocity 输入模型。
- `ocp_centroidal_acc.py`
  - centroidal acceleration 模型。

### `dynamics/`

这个目录负责提供各类动力学和状态流形运算，供 `optimization/` 调用。

- `dynamics.py`
  - 基类，提供通用工具：
  - RNEA torque 计算
  - contact Jacobian torque 估计
  - frame 位置和速度
  - base 位姿提取
- `dynamics_whole_body_torque.py`
  - whole-body torque dynamics，包括：
  - `rnea_dynamics()`
  - `aba_dynamics()`
- `dynamics_whole_body_acc.py`
  - whole-body acceleration 模型。
- `dynamics_centroidal_vel.py`
  - centroidal velocity 模型。
- `dynamics_centroidal_acc.py`
  - centroidal acceleration 模型。

### `utils/`

- `robot.py`
  - 负责加载 URDF/SRDF 和构建机器人对象。
- `gait_sequence.py`
  - 固定步态调度和摆腿 z 向速度样条。
- `visualization.py`
  - 用 MeshCat 绘制接触力和末端力。
- `debug_fatrop.py`
  - 用于检查 Fatrop 稀疏结构识别的调试脚本。

### `robots/`

机器人模型资源目录。

- `b2_description/`
  - Unitree B2 纯四足模型。
- `b2_z1_description/`
  - Unitree B2 + Z1 机械臂模型。

包含：

- `urdf/`
- `srdf/`
- `meshes/`

### `codegen/`

用于把 CasADi/Fatrop 求解函数导出并编译成共享库，便于更接近论文里的部署方式。

---

## 3. 从入口脚本看整个执行流程

入口文件是 `main.py`。

### 3.1 机器人和任务配置

在 `main.py` 里首先设置：

- 使用哪台机器人
- 使用哪种动力学模型
- base 速度目标
- 机械臂末端速度目标
- 机械臂末端力目标
- 预测时域节点数
- torque 限制只在哪几个节点施加
- 步态类型和摆动参数
- 求解器类型

例如当前默认配置：

- 机器人：`B2_Z1(reference_pose="standing_with_arm_up", arm_joints=4)`
- 动力学：`whole_body_rnea`
- 求解器：`fatrop`

这与论文描述一致：锁定 Z1 最后两个关节，只保留 4 个 arm joints，形成 22 DoF 的 whole-body 模型。

### 3.2 初始化机器人模型

`robot.set_gait_sequence(gait_type, gait_period)` 会根据 `trot / walk / stand` 创建固定 gait schedule。

机器人对象里会包含：

- Pinocchio model/data
- 参考姿态 `q0`
- 足端 frame id
- arm 末端 frame id
- 关节位置、速度、力矩上界

### 3.3 创建 OCP

`make_ocp(...)` 会根据 `dynamics` 字符串，在 `optimization/ocp_factory.py` 中选择合适的 OCP 子类。

创建后会自动执行：

1. `setup_problem()`
2. `set_weights()`

也就是：

- 建立决策变量
- 建立参数
- 建立目标
- 建立约束
- 配置权重

### 3.4 进入 MPC 循环

`mpc_loop()` 做的事情是：

1. 更新当前初始状态和 gait schedule。
2. 调用求解器求当前 horizon 的最优解。
3. 记录解算时间和约束违反。
4. 从预测结果中取下一步状态，作为下一次 MPC 的初始状态。

这本质上是 receding horizon 控制的最简实现。

注意：

- 这里没有接真实仿真器或真实机器人。
- 它直接把预测的下一状态作为“下一时刻测量状态”。
- 所以它更像论文方法的求解和验证入口，而不是完整闭环控制系统。

## 4. OCP 基类 `optimization/ocp.py` 的作用

这个文件是整个项目的骨架。

### 4.1 公共参数

所有 OCP 共享这些参数：

- `x_init`
  - 当前时刻状态。
- `dt_min`, `dt_max`
  - 自适应时间步的首末步长。
- `contact_schedule`, `swing_schedule`
  - 固定步态调度。
- `n_contacts`
  - 当前支撑脚数量。
- `swing_period`, `swing_height`, `swing_vel_limits`
  - 摆腿样条参数。
- `Q_diag`, `R_diag`
  - 状态和输入的权重对角线。
- `base_vel_des`
  - 机身速度参考。
- `arm_vel_des`
  - 机械臂末端速度参考。
- `arm_force_des`
  - 机械臂末端力参考。

### 4.2 自适应时间步

论文中强调使用几何级数时间步来兼顾近端高分辨率和远端长时域规划。

代码中：

- `ratio = dt_max / dt_min`
- `gamma = ratio ** (1 / (nodes - 1))`
- `dts[i] = dt_min * gamma**i`

这就是论文里 adaptive time-step scheme 的直接实现。

### 4.3 公共目标函数

公共目标函数是标准二次型：

- 惩罚状态偏差
- 惩罚输入偏差
- 终端状态继续惩罚

也就是论文 Eq. (3) 的实现。

### 4.4 公共约束

`setup_constraints()` 里封装了几类对所有 OCP 都成立的约束：

- 初始状态约束
- 接触脚摩擦锥约束
- 摆动脚零接触力约束
- 接触脚速度约束
- 摆动脚 z 向样条速度约束
- 机械臂末端力约束
- 机械臂末端速度约束
- 关节位置限制
- 关节速度限制

这些约束基本对应论文 IV-B 中的 contact、swing、arm 和 state bounds。

其中一个很重要的实现细节是：

- 脚在摆动相时，xy 方向速度不做参考跟踪，只约束 z 方向样条速度。
- 这意味着 footstep location 是优化涌现出来的，而不是外部预先指定的。

这和论文的描述一致：固定的是 gait schedule，不是足端落点。

## 5. 论文主方法：`whole_body_rnea`

如果只看一份代码，优先看：

- `optimization/ocp_whole_body_rnea.py`
- `dynamics/dynamics_whole_body_torque.py`

这两份文件最直接体现论文主方法。

### 5.1 状态和输入定义

论文中的 whole-body 状态是：

- `x = [q, v]`

代码里也是：

- `nx = nq + nv`
- `ndx_opt = 2 * nv`

注意这里没有直接优化 quaternion 本身，而是优化切空间增量 `dq`，再通过 Pinocchio 的 `integrate()` 回到流形。

这正对应论文对“在 SO(3) 上用局部增量而非直接优化四元数”的描述。

输入在代码里被拆成：

- 加速度 `a`
- 接触力 `forces`
- 前几个节点的关节力矩 `tau_j`

更具体地说：

- 前 `tau_nodes` 个节点：`[a, forces, tau]`
- 后面的节点：`[a, forces]`

这正是论文中“只在前几步保留 torque decision variables，并施加 torque bounds”的实现。

### 5.2 离散状态更新

这里采用显式 Euler：

- `dq_next = dq + v * dt`
- `dv_next = dv + a * dt`

对应论文 Eq. (4)。

### 5.3 RNEA 路径约束

整个论文最关键的一条约束就是：

- 用 RNEA 把 whole-body inverse dynamics 作为 path constraint 加进每个节点。

代码里：

- `tau_rnea = self.dyn.rnea_dynamics()(q, v, a, forces)`
- `tau_rnea[:6] == 0`

含义是：

- 浮动基底不应有外加驱动力矩。
- 系统必须满足 floating-base dynamics。

在前几个节点还额外要求：

- `tau_rnea[6:] == tau_j`

也就是显式把关节力矩与 inverse dynamics 一致化。

### 5.4 为什么这是论文主方法

论文强调：

- 相比 forward dynamics transcription，inverse dynamics transcription 更快、更鲁棒。
- 相比 centroidal model，它更真实地考虑了 arm 和 limb 的质量分布以及关节驱动约束。

这一点在代码结构上很清楚：

- `whole_body_rnea` 被作为默认主要模型。
- `centroidal_*` 和 `whole_body_aba` 更多是 benchmark 对照组。

## 6. 其他动力学模型在做什么

论文不只实现了主方法，还实现了几套对照模型。

### 6.1 `whole_body_aba`

对应 forward dynamics 版本。

做法是：

- 把关节力矩 `tau_j` 作为输入。
- 用 ABA 计算加速度 `a = aba(q, v, tau_j, forces)`。
- 再做 Euler 离散推进。

它代表：

- 用前向动力学做 whole-body MPC 的常规方式。

论文用它来证明：

- RNEA 作为 path constraint 的 formulation 更适合实时求解。

### 6.2 `whole_body_acc`

这个版本的输入是加速度和接触力。

如果 `include_base=True`：

- 直接把 base acceleration 也作为输入变量。

否则：

- 只优化关节加速度。
- 再通过 base dynamics 求出 base acceleration。

这个模型更接近“全身加速度一致性约束”。

### 6.3 `centroidal_vel`

这是 reduced-order 模型。

状态是：

- `h`：按质量归一化后的 centroidal momentum 变量
- `q`：关节构型

输入是：

- 速度 `v`
- 接触力 `forces`

如果不把 base velocity 作为输入，就用 centroidal momentum matrix `A(q)` 反推出 base velocity。

更准确地说，这里的 `h` 不是未经缩放的物理动量，而是与 `A(q)v / mass` 一致的量。代码中：

- `com_dynamics()` 返回的是 `(sum forces + gravity terms) / mass`
- `dynamics_gaps()` 写成 `A(q) v - h * mass = 0`
- 目标状态前 6 维直接用 `base_vel_des`

因此把它理解为“质量归一化的 centroidal momentum”或“类 centroidal 速度变量”更准确。

这套模型保留了全身运动学，但动力学层面只建模 centroidal 量的演化，而不是完整刚体动力学。

### 6.4 `centroidal_acc`

和上一个类似，但输入是加速度而不是速度。

它利用：

- `A(q)`
- `A_dot(q, v)`

构建 centroidal acceleration constraints。

### 6.5 为什么这些模型都保留在仓库里

因为论文做了系统 benchmark，比较了：

- whole-body inverse dynamics
- whole-body forward dynamics
- centroidal velocity
- centroidal acceleration

所以这个仓库不仅仅是“论文主方法代码”，也是一个统一的 benchmark 框架。

## 7. `dynamics/` 目录里每类函数的工程意义

### 7.1 `state_integrate()` 和 `state_difference()`

这两个函数非常关键。

原因是 floating-base 机器人含有 quaternion，状态空间不在欧氏空间里。代码没有直接优化绝对位姿，而是：

- 在切空间里优化 `dx`
- 用 Pinocchio `integrate()` 生成真实状态
- 用 Pinocchio `difference()` 比较状态误差

这避免了：

- 直接对 quaternion 优化
- 手动加单位范数约束

这是机器人非线性优化里非常标准且正确的做法。

### 7.2 `rnea_dynamics()`

这个函数将末端外力映射到对应 joint 的外力表达，然后调用 Pinocchio 的 `rnea()` 计算整机广义力。

输出是：

- `tau_rnea`，维度包含 base 的 6 维和关节部分。

在论文主方法里它就是“动力学一致性约束”的核心。

### 7.3 `aba_dynamics()`

给定：

- `q`
- `v`
- `tau_j`
- 接触力

计算：

- `a`

这是 forward dynamics 版本的核心。

### 7.4 `tau_estimate()`

这个函数用接触 Jacobian 和重力项粗略估计关节力矩。

它主要用于：

- centroidal 模型和部分简化模型中施加 torque limits。

注意这里不是完整 inverse dynamics，只是估计量，所以它不如 `whole_body_rnea` 那样严格。

## 8. `utils/robot.py` 与机器人建模

这个文件封装了机器人实例化。

### 8.1 `Robot`

公共父类负责：

- 用 URDF 建立 Pinocchio `RobotWrapper`
- 用 SRDF 读取 reference pose
- 读取 joint limits / velocity limits / torque limits

### 8.2 `B2`

纯四足模型。

### 8.3 `B2_Z1`

带机械臂的移动操作机器人模型。

这里的关键实现点：

- `arm_joints` 可以控制保留几个机械臂关节。
- 剩余关节通过 `buildReducedRobot()` 锁死。
- 若 arm 被启用，额外引入末端外力变量。

也就是说：

- 这个仓库天然支持“纯 locomotion”和“loco-manipulation”两种模式。

## 9. 步态不是优化出来的，优化的是接触下的动作

`utils/gait_sequence.py` 非常重要，因为它明确告诉你这个仓库没有做什么。

它负责生成：

- `contact_schedule`
- `swing_schedule`

支持的 gait：

- `trot`
- `walk`
- `stand`

含义是：

- 哪只脚在这一时刻接触地面，是预先设定的。
- 摆腿相位也是预先设定的。

但在这个固定步态模板下，MPC 仍然会优化：

- 足端具体如何移动
- 接触力怎么分配
- 机身如何配合
- arm 如何运动和施力

因此论文的“emergent whole-body behavior”不是指 gait phase 被优化，而是指在给定 gait schedule 的前提下，全身协同动作是通过优化自然产生的。

## 10. 论文公式与代码的直接映射

### Eq. (3): 二次代价

论文：

- 惩罚状态偏差
- 惩罚输入偏差
- 终端状态也惩罚

代码位置：

- `optimization/ocp.py` 中的 `setup_objective()`
- `optimization/ocp_whole_body_rnea.py` 中对输入维数做了特殊处理

### Eq. (4): 显式 Euler 离散

论文：

- `delta q_{k+1} = delta q_k + v_k dt_k`
- `v_{k+1} = v_k + a_k dt_k`

代码位置：

- 各个 OCP 子类中的 `setup_dynamics_constraints()`

### Eq. (5): inverse dynamics path constraint

论文：

- `f_RNEA(q_k, v_k, a_k, F_c,k) = [0; tau_j,k]`

代码位置：

- `optimization/ocp_whole_body_rnea.py`

实现细节需要特别注意：

- 前 `tau_nodes` 个节点，代码显式保留 `tau_j` 作为决策变量，并约束 `tau_rnea[6:] == tau_j`
- 后续节点，为了减少决策变量和加速求解，只保留 `tau_rnea[:6] == 0` 这 6 维 floating-base dynamics 约束

所以如果按实现来描述，更精确的说法是：

- 前几步：`f_RNEA(q_k, v_k, a_k, F_c,k) = [0; tau_j,k]`
- 后续步：仅保留 base 的 6 维一致性约束

### Eq. (6): 接触与摆动约束

论文内容：

- 接触脚法向力非负
- 摩擦锥约束
- 接触脚速度为零
- 摆动脚外力为零
- 摆动脚 z 向速度追踪样条

代码位置：

- `optimization/ocp.py` 的 `setup_constraints()`

### Eq. (7): arm force / velocity constraints

论文内容：

- `Fc_arm = Fc_arm_des`
- `vc_arm = vc_arm_des`

代码位置：

- `optimization/ocp.py`

代码里还额外把 arm 目标从“相对 base 的速度”转换到了世界系，并补偿 base 平动和角速度项。

### Eq. (8): 状态和输入边界

论文内容：

- 关节位置约束
- 关节速度约束
- 力矩约束只施加前几步

代码位置：

- 关节位置/速度限制：`optimization/ocp.py`
- 力矩限制：各子类中 `if i < tau_nodes`

## 11. 求解器设计与论文实现细节

### 11.1 Fatrop

论文主打的实时性能依赖 Fatrop。

代码里：

- `args.py` 配置了 Fatrop 选项。
- `optimization/ocp.py` 里对 `fatrop` 和 `ipopt` 走 CasADi `Opti.to_function()` 路线。

设计上有两个关键点：

1. 使用 exact Hessian。
2. 决策变量按 stage 组织，便于 Fatrop 识别块稀疏结构。

### 11.2 IPOPT

作为另一个 interior-point solver baseline。

### 11.3 OSQP

这里不是直接求 NLP，而是做一个简单 SQP：

- 线性化约束
- Hessian 近似成对角
- 用 OSQP 解 QP
- 再做 Armijo line search

这部分更像对照实验，而不是论文最终硬件部署方案。

## 12. Warm start 在这里为什么很重要

论文强调 warm-start 对实时 MPC 很重要。

代码中 warm start 的实现非常务实：

- 状态增量沿用上一轮解
- 接触力根据当前 contact schedule 重新初始化
- 摆动脚力直接清零
- arm force 目标重新覆盖到末端力部分

这是合理的，因为 adaptive time step 和 gait phase 切换会让直接平移上一轮接触力初值变得不稳定。

## 13. 可视化和调试工具

### 13.1 `utils/visualization.py`

它会把：

- 四足接触力
- arm 末端力

画成 MeshCat 中的红色箭头，便于观察优化结果的物理含义。

### 13.2 `utils/debug_fatrop.py`

这个脚本用来读取 `debug_fatrop_*.mtx` 调试文件，检查 Fatrop 是否正确识别了 stage-wise block sparsity。

这类文件不是源码的一部分，而是调试输出。

## 14. `codegen/` 目录意味着什么

论文硬件实验不是直接用 Python `Opti.solve()` 跑出来的，而是依赖 code generation。

这个目录的作用是：

1. 把 CasADi solver function 导出成 C。
2. 用 Fatrop 和 Blasfeo 编译成共享库。
3. 再在部署端加载这个共享库。

因此这个仓库里虽然没有完整硬件通信代码，但已经保留了“从研究原型到部署库”的关键步骤。

## 15. 如何高效阅读这个项目

推荐顺序如下。

### 第一遍：先看主流程

1. `main.py`
2. `args.py`
3. `optimization/ocp_factory.py`

目的：

- 知道程序怎么启动
- 有哪些模型和 solver 可选

### 第二遍：看公共 OCP 框架

1. `optimization/ocp.py`

目的：

- 弄清参数、约束、目标和求解器接口怎么统一组织

### 第三遍：只看论文主方法

1. `optimization/ocp_whole_body_rnea.py`
2. `dynamics/dynamics_whole_body_torque.py`

目的：

- 理解论文主方法怎样从公式变成代码

### 第四遍：再看 benchmark 模型

1. `optimization/ocp_whole_body_aba.py`
2. `optimization/ocp_centroidal_vel.py`
3. `optimization/ocp_centroidal_acc.py`
4. `dynamics/dynamics_centroidal_vel.py`
5. `dynamics/dynamics_centroidal_acc.py`

目的：

- 理解论文为什么说 inverse dynamics 更适合实时 whole-body loco-manipulation

## 16. 一句话总结这个仓库

一句话概括：

这是一个以 `Pinocchio + CasADi + Fatrop` 为核心、面向 legged loco-manipulation 的 whole-body MPC 研究框架，其中 `whole_body_rnea` 是论文主方法，其他动力学模型主要用于系统对比和 benchmark。

再具体一点说：

- `robots/` 定义机器人模型。
- `dynamics/` 提供动力学方程。
- `optimization/` 把这些方程装配成 NLP。
- `main.py` 驱动 MPC 求解。
- `codegen/` 提供部署导出路径。

## 17. 当前仓库与论文完整系统之间的差别

最后需要明确一点，这个仓库和论文中的完整硬件系统并不完全等价。

仓库里能直接看到的是：

- OCP 建模
- 求解
- warm start
- codegen
- 可视化

论文系统里还有但仓库里没有完整展开的部分包括：

- 真实机器人状态估计
- 以太网通信
- 500 Hz 插值与底层 PD 执行
- 硬件侧的安全机制和滤波细节

所以阅读这个仓库时，最正确的定位是：

- 这是论文“控制与优化核心”的开源实现。
- 不是完整产品级机器人控制系统。

## 18. 关键代码速查表

这一节用于快速定位“某个功能到底在哪实现”。

### 18.1 入口与总体配置

- `main.py:13`
  - 选择机器人实例，默认是 `B2_Z1(...)`。
- `main.py:14`
  - 选择动力学模型，默认是 `whole_body_rnea`。
- `main.py:17`
  - 设置 base 速度目标 `base_vel_des`。
- `main.py:18`
  - 设置 arm 末端速度目标 `arm_vel_des`。
- `main.py:19`
  - 设置 arm 末端力目标 `arm_force_des`。
- `main.py:22`
  - 预测节点数 `nodes`。
- `main.py:23`
  - `tau_nodes`，即只在前几个节点显式保留力矩变量和力矩约束。
- `main.py:24`
  - `dt_min`，最小时间步。
- `main.py:25`
  - `dt_max`，最大时间步。
- `main.py:34`
  - 选择求解器，默认 `fatrop`。
- `main.py:46`
  - `mpc_loop()`，整个 receding-horizon 求解循环。
- `main.py:120`
  - `main()`，程序主流程入口。

### 18.2 求解器与模型配置

- `args.py:2`
  - `DYN_ARGS`，不同动力学模型的构造参数。
- `args.py:19`
  - `SOLVER_ARGS`，不同求解器的参数。
- `args.py:20`
  - Fatrop 选项。
- `args.py:34`
  - IPOPT 选项。
- `args.py:46`
  - OSQP-SQP 的选项。

### 18.3 机器人模型与步态

- `utils/robot.py:9`
  - `Robot` 基类。
- `utils/robot.py:19`
  - 从 URDF 构建 Pinocchio `RobotWrapper`。
- `utils/robot.py:25`
  - 从 SRDF 读取 reference pose。
- `utils/robot.py:36`
  - 读取关节位置/速度/力矩上界。
- `utils/robot.py:46`
  - `set_gait_sequence()`，将 gait scheduler 绑定到机器人。
- `utils/robot.py:51`
  - `B2` 纯四足模型。
- `utils/robot.py:61`
  - `B2_Z1` 移动操作模型。
- `utils/robot.py:65`
  - 通过 `lock_joints` 锁住不使用的 arm joints。
- `utils/robot.py:71`
  - 启用 arm 末端外力变量。

- `utils/gait_sequence.py:5`
  - `GaitSequence` 类。
- `utils/gait_sequence.py:11`
  - `trot` 的接触脚数量和摆动周期。
- `utils/gait_sequence.py:15`
  - `walk` 的接触脚数量和摆动周期。
- `utils/gait_sequence.py:19`
  - `stand` 的接触脚数量和摆动周期。
- `utils/gait_sequence.py:26`
  - `get_gait_schedule()`，生成 horizon 上的 `contact_schedule` 和 `swing_schedule`。
- `utils/gait_sequence.py:96`
  - `get_spline_vel_z()`，摆腿 z 向速度样条。

### 18.4 OCP 总骨架

- `optimization/ocp.py:11`
  - `OCP` 抽象基类。
- `optimization/ocp.py:42`
  - `setup_problem()`，统一装配问题。
- `optimization/ocp.py:56`
  - `setup_parameters()`，定义所有参数。
- `optimization/ocp.py:77`
  - 自适应时间步 `dts` 的几何级数生成。
- `optimization/ocp.py:88`
  - 公共二次型目标函数。
- `optimization/ocp.py:111`
  - 公共约束入口 `setup_constraints()`。
- `optimization/ocp.py:153`
  - 接触脚摩擦锥约束。
- `optimization/ocp.py:159`
  - 摆动脚零力约束。
- `optimization/ocp.py:167`
  - 接触脚零 xy 速度约束。
- `optimization/ocp.py:172`
  - 摆动脚 z 向样条速度约束。
- `optimization/ocp.py:190`
  - arm 末端力约束。
- `optimization/ocp.py:200`
  - arm 末端速度约束。
- `optimization/ocp.py:207`
  - 关节位置与速度边界。
- `optimization/ocp.py:260`
  - 根据当前时间更新 gait schedule。
- `optimization/ocp.py:270`
  - `update_params()`，MPC 每轮更新入口。
- `optimization/ocp.py:292`
  - `init_solver()`，初始化 Fatrop / IPOPT / OSQP。
- `optimization/ocp.py:320`
  - 将 `Opti` 导出成 CasADi `solver_function`。
- `optimization/ocp.py:357`
  - `compile_solver()`，生成 C 代码。
- `optimization/ocp.py:372`
  - `solve()`，统一求解入口。

### 18.5 论文主方法 `whole_body_rnea`

- `optimization/ocp_whole_body_rnea.py:9`
  - `OCPWholeBodyRNEA`，论文主方法类。
- `optimization/ocp_whole_body_rnea.py:21`
  - `include_acc` 选项，是否把加速度显式作为输入。
- `optimization/ocp_whole_body_rnea.py:59`
  - 定义状态和输入维度。
- `optimization/ocp_whole_body_rnea.py:65`
  - 前 `tau_nodes` 个节点的输入是 `[a, forces, tau]`。
- `optimization/ocp_whole_body_rnea.py:67`
  - 后续节点输入只保留 `[a, forces]`。
- `optimization/ocp_whole_body_rnea.py:80`
  - 设置参考状态与参考输入。
- `optimization/ocp_whole_body_rnea.py:100`
  - 特化后的目标函数。
- `optimization/ocp_whole_body_rnea.py:124`
  - 论文主方法的动力学约束入口。
- `optimization/ocp_whole_body_rnea.py:141`
  - Euler 离散中的 `dq_next == dq + v * dt`。
- `optimization/ocp_whole_body_rnea.py:144`
  - Euler 离散中的 `dv_next == dv + a * dt`。
- `optimization/ocp_whole_body_rnea.py:147`
  - 调用 RNEA。
- `optimization/ocp_whole_body_rnea.py:148`
  - 浮动基底 6 维约束 `tau_rnea[:6] == 0`。
- `optimization/ocp_whole_body_rnea.py:152`
  - 前几个节点强制 `tau_rnea[6:] == tau_j`。
- `optimization/ocp_whole_body_rnea.py:155`
  - 前几个节点的关节力矩约束。
- `optimization/ocp_whole_body_rnea.py:190`
  - 论文主方法的 warm start。

### 18.6 对照模型

- `optimization/ocp_whole_body_aba.py:8`
  - forward dynamics whole-body OCP。
- `optimization/ocp_whole_body_aba.py:89`
  - 通过 ABA 计算加速度。
- `optimization/ocp_whole_body_aba.py:111`
  - 前几个节点的关节力矩限制。

- `optimization/ocp_centroidal_vel.py:8`
  - centroidal velocity OCP。
- `optimization/ocp_centroidal_vel.py:51`
  - 状态是 `6 + nq`，即 `h + q`，其中 `h` 是按质量归一化的 centroidal 量。
- `optimization/ocp_centroidal_vel.py:97`
  - centroidal momentum 离散更新。
- `optimization/ocp_centroidal_vel.py:105`
  - `A(q) v = h` 的 dynamics gap 约束。
- `optimization/ocp_centroidal_vel.py:110`
  - torque bound 通过 `tau_estimate()` 近似施加。

- `optimization/ocp_centroidal_acc.py:8`
  - centroidal acceleration OCP。
- `optimization/ocp_centroidal_acc.py:109`
  - `A(q) a + A_dot(q,v) v = dh` 的 gap 约束。

### 18.7 动力学函数

- `dynamics/dynamics.py:6`
  - 动力学基类。
- `dynamics/dynamics.py:34`
  - 通用 `rnea_dynamics()`。
- `dynamics/dynamics.py:63`
  - `tau_estimate()`，用 Jacobian 和重力估计关节力矩。
- `dynamics/dynamics.py:86`
  - 获取 frame 位置。
- `dynamics/dynamics.py:96`
  - 获取 frame 速度。
- `dynamics/dynamics.py:107`
  - 获取 base 位置。
- `dynamics/dynamics.py:117`
  - 获取 base 朝向。

- `dynamics/dynamics_whole_body_torque.py:7`
  - whole-body torque dynamics。
- `dynamics/dynamics_whole_body_torque.py:9`
  - 状态积分 `state_integrate()`。
- `dynamics/dynamics_whole_body_torque.py:25`
  - 状态差分 `state_difference()`。
- `dynamics/dynamics_whole_body_torque.py:40`
  - whole-body RNEA。
- `dynamics/dynamics_whole_body_torque.py:66`
  - whole-body ABA。

- `dynamics/dynamics_whole_body_acc.py:41`
  - 由 whole-body dynamics 反推 base acceleration。
- `dynamics/dynamics_whole_body_acc.py:78`
  - whole-body acceleration gap 约束。

- `dynamics/dynamics_centroidal_vel.py:41`
  - centroidal momentum dynamics。
- `dynamics/dynamics_centroidal_vel.py:64`
  - 从 `h` 和 `q` 反推出 base velocity。
- `dynamics/dynamics_centroidal_vel.py:82`
  - 从 centroidal quantities 反推出 base acceleration。
- `dynamics/dynamics_centroidal_vel.py:120`
  - `A(q) v - h * mass = 0` 的 gap。

### 18.8 部署与可视化

- `utils/visualization.py:6`
  - `visualize_forces()`，在 MeshCat 中画接触力箭头。
- `utils/visualization.py:23`
  - `add_force_to_viewer()`，将力向量转成圆柱箭头。

- `codegen/README.md:1`
  - 代码生成说明。
- `codegen/README.md:15`
  - 在 `main.py` 中打开 `compile_solver=True` 生成 `solver_function.c`。
- `codegen/README.md:16`
  - 在 `codegen/build` 中编译共享库。
- `codegen/README.md:23`
  - 用共享库替代 Python 版 solver function。

## 19. 主方法调用链流程图

下面这张图只画论文主方法 `whole_body_rnea + fatrop` 的主路径。

```text
main.py
  |
  |-- 读取用户配置
  |     robot / dynamics / targets / gait / solver
  |
  |-- robot = B2_Z1(...)
  |     |
  |     |-- utils/robot.py
  |           |
  |           |-- 读取 URDF/SRDF
  |           |-- 建立 Pinocchio model/data
  |           |-- 读取 joint limits
  |           |-- 设置 arm end-effector frame
  |
  |-- robot.set_gait_sequence(...)
  |     |
  |     |-- utils/gait_sequence.py
  |           |
  |           |-- 生成 gait 类型
  |           |-- 后续按 t_current 生成 contact_schedule / swing_schedule
  |
  |-- ocp = make_ocp("whole_body_rnea", ...)
  |     |
  |     |-- optimization/ocp_factory.py
  |           |
  |           |-- 返回 OCPWholeBodyRNEA
  |
  |-- OCPWholeBodyRNEA.__init__()
  |     |
  |     |-- dynamics = DynamicsWholeBodyTorque(...)
  |     |-- x_nom = [q0, 0]
  |     |-- 定义输入结构 [a, forces, tau] / [a, forces]
  |
  |-- ocp.setup_problem()
  |     |
  |     |-- setup_variables()
  |     |     |
  |     |     |-- 创建 DX_opt[k], U_opt[k]
  |     |
  |     |-- setup_parameters()
  |     |     |
  |     |     |-- x_init, dt_min, dt_max
  |     |     |-- contact_schedule, swing_schedule
  |     |     |-- Q_diag, R_diag
  |     |     |-- base_vel_des, arm_vel_des, arm_force_des
  |     |     |-- dts = geometric time steps
  |     |
  |     |-- setup_targets()
  |     |     |
  |     |     |-- dx_des
  |     |     |-- f_des = 支撑脚重力分配参考力
  |     |     |-- u_des = [0 acc, f_des, 0 tau]
  |     |
  |     |-- setup_constraints()
  |     |     |
  |     |     |-- 初始状态约束
  |     |     |-- 摩擦锥约束
  |     |     |-- 摆动脚零力
  |     |     |-- 接触脚速度约束
  |     |     |-- 摆动脚 z 速度样条约束
  |     |     |-- arm force / velocity 约束
  |     |     |-- joint pos / vel 约束
  |     |     |
  |     |     |-- setup_dynamics_constraints(i)
  |     |           |
  |     |           |-- q, v 由 state_integrate(x_init, DX_opt[i]) 得到
  |     |           |-- a 从 U_opt[i] 取出
  |     |           |-- forces 从 U_opt[i] 取出
  |     |           |-- tau_rnea = rnea(q, v, a, forces)
  |     |           |-- tau_rnea[:6] == 0
  |     |           |-- 前 tau_nodes 个节点:
  |     |                 tau_rnea[6:] == tau_j
  |     |                 tau_min <= tau_j <= tau_max
  |     |
  |     |-- setup_objective()
  |           |
  |           |-- 累积 sum_k ||dx - dx_des||_Q^2 + ||u - u_des||_R^2
  |
  |-- ocp.init_solver("fatrop", ...)
  |     |
  |     |-- CasADi Opti -> solver_function
  |     |-- 参数按固定顺序拼接
  |
  |-- mpc_loop()
        |
        |-- update_params(x_init, t_current)
        |     |
        |     |-- 更新初始状态
        |     |-- 更新 gait schedule
        |     |-- warm start 上一轮解
        |
        |-- solver_function(*solver_params)
        |     |
        |     |-- Fatrop 求解整个 horizon
        |
        |-- retract_stacked_sol(sol_x)
        |     |
        |     |-- 从堆叠解中恢复 q, v, a, forces, tau
        |
        |-- 取 DX_prev[1]
        |     |
        |     |-- 作为下一时刻状态更新 x_init
        |
        |-- 重复下一轮 MPC
```

## 20. 如果只想抓住论文主线，最少看哪些代码

如果只看最少文件来理解论文主方法，建议只看下面 6 个：

1. `main.py`
2. `args.py`
3. `optimization/ocp.py`
4. `optimization/ocp_whole_body_rnea.py`
5. `dynamics/dynamics_whole_body_torque.py`
6. `utils/gait_sequence.py`

它们分别回答六个问题：

1. 代码怎么跑起来。
2. 支持哪些模型和求解器。
3. OCP 是怎么统一组织的。
4. 论文主方法怎么写成约束。
5. RNEA/ABA 在代码里怎么调用。
6. 步态和摆腿参考是怎么输入到 MPC 的。


## 21. 如果要把 Go2+Piper 接到 `unitree_mujoco`，WBIC 该怎么改

这里给出一个面向工程落地的改造方案，目标是：

- 保留 `unitree_mujoco` 作为 MuJoCo 仿真端和 DDS 桥接端。
- 把 `wb-mpc-locoman` 改成一个“外部上层控制器”。
- 通过 `rt/lowstate` / `rt/sportmodestate` / `rt/lowcmd` 与仿真通信。

### 21.1 先明确：不要把整个 WBIC 强行塞进 `unitree_mujoco.py`

更合理的架构是三层：

1. `unitree_mujoco/simulate_python/unitree_mujoco.py`
   - 负责加载 MJCF、推进 MuJoCo、发布状态、接收控制。
2. `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`
   - 负责 DDS 话题桥接：
   - `rt/lowstate`
   - `rt/sportmodestate`
   - `rt/lowcmd`
3. `wb-mpc-locoman`
   - 新增一个实时控制入口脚本，作为上层控制器运行。

也就是说，WBIC 最终应该是“单独一个进程”，而不是并到 MuJoCo 仿真线程内部。

### 21.2 现有 WBIC 为什么不能直接接

当前 `wb-mpc-locoman` 的 `main.py` 本质上是一个离线/半离线 MPC demo：

- 直接在脚本顶部实例化机器人和 OCP。
- `mpc_loop()` 中不断更新参数并调用求解器。
- 求解结果主要用于可视化和论文复现。
- 没有实时状态输入接口。
- 没有 `LowCmd/LowState` 通信接口。

因此要接入 `unitree_mujoco`，关键不是改 MuJoCo，而是给 WBIC 加“机器人软件栈适配层”。

### 21.3 第一类改动：机器人模型层

推荐新增一个专门的 Go2+Piper 机器人类，而不是直接改 `B2_Z1`。

建议修改：

- 文件：`utils/robot.py`
- 做法：新增 `Go2Piper` 类

这个类需要完成：

1. 指向组合 URDF
   - 目标 URDF：
   - `/home/wzx/WholeBodyRL_WS/UnitreeSim2Real/unitree_mujoco/unitree_robots/Go2Arm_description/urdf/go2_piper_description_mjc_NoGripper.urdf`

2. 使用 floating-base 模型
   - 继续沿用 `Robot` 基类中的 `JointModelFreeFlyer()`。

3. 指定正确的 base frame 名称
   - 你的组合 URDF 中：
   - `base` 是四足机身根 link
   - `base_link` 是机械臂安装基座
   - 因此对于 locomotion / whole-body control，base frame 应该用 `base`，不能继续假设是 `base_link`。

4. 指定机械臂末端 frame
   - 你当前是 `NoGripper` 版本，没有 `gripperCenter`。
   - 第一版可以直接把机械臂末端 frame 设成 `link6`。
   - 更规范的做法是在 URDF 里额外增加一个固定 `ee_link`，让末端控制和可视化都统一针对该 frame。

5. 设置参考姿态 `q0`
   - 当前 Go2+Piper 没有现成 SRDF reference pose。
   - 第一版建议不要依赖 `pin.loadReferenceConfigurations()`。
   - 直接在 `Go2Piper` 类里手工构造一个 standing + arm home 的 `q0`。

6. 确认驱动关节维度
   - 组合模型现在是 12 条腿关节 + 6 个机械臂关节 = 18 个驱动关节。
   - 这需要和后续 DDS 控制输出严格一致。

### 21.4 第二类改动：动力学与 frame 假设

当前 `dynamics/dynamics.py` 中有一个关键假设：

- `self.base_frame = self.model.getFrameId("base_link")`

这对 B2/B2+Z1 合理，但对你的 Go2+Piper 不合理，因为你的 `base_link` 是机械臂基座，不是狗身躯干。

建议修改：

- 文件：`dynamics/dynamics.py`
- 做法：把 `base_frame` 改成由 robot 对象传入

推荐改法：

1. 在 `Robot` 基类中新增字段：
   - `self.base_frame_name`
2. 在 `Go2Piper` 中设置：
   - `self.base_frame_name = "base"`
3. 在 `Dynamics.__init__()` 中改成：
   - `self.base_frame = self.model.getFrameId(robot.base_frame_name)`
   - 或者直接在构造函数里传 `base_frame_name`

这一步必须优先做，否则所有 base 位置、姿态、速度相关代价和约束都会参考错 frame。

### 21.5 第三类改动：新增 DDS 适配层

这是最核心的工程改动。

建议新增文件：

- `unitree_go2piper_wbic_node.py`

职责如下：

1. 初始化 DDS
2. 订阅 `rt/lowstate`
3. 订阅 `rt/sportmodestate`
4. 把订阅到的数据转换成 Pinocchio 所需的 `q, v`
5. 调用 WBIC / MPC 求解
6. 把结果封装成 `LowCmd` 发到 `rt/lowcmd`

推荐不要把这些逻辑塞回 `main.py`，因为 `main.py` 仍然应该保留为论文 demo 入口。

### 21.6 状态映射该怎么做

这里是接入最容易出错的部分。

建议新建一个状态转换模块，例如：

- `utils/state_adapter.py`

把 DDS 输入转换成 WBIC 的状态向量。

推荐映射关系：

1. 从 `sportmodestate` 取 base 平移和线速度
   - `position[0:3] -> q[0:3]`
   - `velocity[0:3] -> v[0:3]`

2. 从 `lowstate.imu_state` 取 base 姿态和角速度
   - `imu quaternion -> q[3:7]`
   - `gyroscope -> v[3:6]`

3. 从 `lowstate.motor_state[i]` 取 18 个驱动关节状态
   - `q -> q[7:]`
   - `dq -> v[6:]`

这里至少要检查两件事：

- 四元数顺序是否和 Pinocchio 一致
- base 速度和角速度的参考坐标系是否与 WBIC 使用的一致

这两个问题建议在第一版接入时单独写测试，不要边接边猜。

### 21.7 好消息：关节顺序现在已经基本对齐

当前 MuJoCo WBIC 模型中，actuator / sensor 顺序已经整理成：

- `FL_hip, FL_thigh, FL_calf`
- `FR_hip, FR_thigh, FR_calf`
- `RL_hip, RL_thigh, RL_calf`
- `RR_hip, RR_thigh, RR_calf`
- `joint1 ... joint6`

而组合 URDF 被 Pinocchio 读取后的活动关节顺序也是这套顺序。

这意味着：

- 不需要再做一层复杂的关节编号重排
- `motor_cmd[i]`、`lowstate.motor_state[i]` 和 Pinocchio joint ordering 可以统一

这是整个接入过程里一个非常有利的条件。

### 21.8 第四类改动：把优化输出变成 MIT / torque 控制命令

`unitree_mujoco` 的桥接层现在已经支持 MIT 形式：

- `tau + kp * (q_des - q) + kd * (dq_des - dq)`

因此 WBIC 节点有两种发命令方式：

1. 纯 torque 模式
   - `kp = 0`
   - `kd = 0`
   - `tau = tau_ff`

2. MIT + feedforward 模式
   - `q = q_ref`
   - `dq = dq_ref`
   - `kp`, `kd` 给中等反馈增益
   - `tau = tau_ff`

推荐落地顺序是：

- 第一阶段先用纯 torque 模式打通链路
- 第二阶段再加 MIT 反馈，提高跟踪稳定性

### 21.9 第五类改动：约束与力矩范围统一

`WBIC` 会从 URDF 读取 effort limits 作为 torque constraints：

- `self.joint_torque_max = self.model.effortLimit[6:]`

这意味着接入前需要统一三套限制：

1. URDF 里的 `effort`
2. WBIC 优化器中的 torque bounds
3. MuJoCo `motor ctrlrange`

如果三者不一致，会出现：

- 优化器认为某个力矩可行，但 MuJoCo actuator 直接裁剪
- 或者 MuJoCo 允许很大力矩，但优化器又保守得多

建议在 Go2Piper 类完成后，专门做一次 torque-limit audit。

### 21.10 第六类改动：把 WBIC 从离线 demo 改成实时循环

当前 `main.py` 的 `mpc_loop()` 逻辑是：

- 用上一轮预测状态更新 `x_init`
- 继续滚动求解 horizon

真实接入 `unitree_mujoco` 后，应该改成：

1. 每轮从 DDS 读当前真实机器人状态
2. 用真实状态构造 `x_init`
3. 更新 gait schedule / tracking target
4. 调用求解器
5. 只取当前时刻的控制量发给 MuJoCo
6. 下一轮继续读取真实反馈，而不是只用内部 rollout

因此建议不要直接复用 `mpc_loop()` 原样，而是把它拆成：

- `controller_core.py`
  - 保留 OCP 初始化、warm start、solve 步骤
- `unitree_go2piper_wbic_node.py`
  - 负责实时循环、DDS 通信、命令发布

### 21.11 推荐的最小实施顺序

建议严格分 4 个阶段推进。

#### 阶段 A：模型适配

1. 新增 `Go2Piper` 类
2. 改 `base_frame` 为可配置
3. 指定 `arm_ee_frame`
4. 手工设置 `q0`
5. 本地用 Pinocchio 单独验证：
   - `nq/nv/nj` 是否正确
   - foot frame 是否能找到
   - arm ee frame 是否能找到

#### 阶段 B：状态通信打通

1. 新建 DDS 节点骨架
2. 订阅 `lowstate/highstate`
3. 打印恢复后的 `q/v`
4. 验证 base 姿态、腿关节、arm joints 是否与 MuJoCo 一致

这一阶段不要上优化器，只做观测对齐。

#### 阶段 C：控制链路打通

1. 先不跑完整 WBIC
2. 只发零力矩 / 重力补偿 / 简单站立 torque
3. 验证 `rt/lowcmd` 能否稳定驱动 MuJoCo
4. 检查 18 个关节索引是否完全正确

这一阶段目标是“命令链打通”，不是“走出漂亮运动”。

#### 阶段 D：接入 WBIC 求解器

1. 在实时循环里初始化 OCP
2. 每轮用真实状态更新 `x_init`
3. 求解 whole-body OCP
4. 先只取 horizon 第一拍输出
5. 发 torque 或 MIT+feedforward 命令
6. 逐步调 solver 频率、warm start、约束和权重

### 21.12 第一版工程上最值得先做的三个文件

如果只做第一版最小接入，我建议优先动这三个文件：

1. `utils/robot.py`
   - 新增 `Go2Piper`
2. `dynamics/dynamics.py`
   - 去掉 `base_link` 的硬编码假设
3. `unitree_go2piper_wbic_node.py`
   - 新增实时控制入口

只要这三处落下去，后续的 solver、参数、步态都可以逐步迭代。

### 21.13 一个现实判断：第一版先不要追求“论文级最终效果”

第一版的目标应该是：

- 正确读状态
- 正确发 18 关节命令
- 站立不炸仿真
- 机械臂能跟随简单目标

只要这一步打通，后面再逐渐做：

- torque limit 校准
- gait tuning
- arm force task
- 更高频求解
- solver codegen

这样改造风险最低，也最符合当前代码结构。
