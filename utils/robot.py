from os.path import dirname, abspath

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from .gait_sequence import GaitSequence


class Robot:
    def __init__(self, urdf_path, srdf_path, reference_pose, use_quaternion=True, lock_joints=None):
        # 解析 URDF 所在目录，便于 Pinocchio 继续找到 URDF 中通过相对路径引用的 mesh。
        urdf_dir = dirname(abspath(urdf_path))

        # 论文将机器人建模为 floating-base 系统，因此默认使用 free-flyer 关节，
        # 也就是给基座 6 个非驱动自由度。下面的 composite joint 只是备用表示。
        if use_quaternion:
            joint_model = pin.JointModelFreeFlyer()
        else:
            joint_model = pin.JointModelComposite()
            joint_model.addJoint(pin.JointModelTranslation())
            joint_model.addJoint(pin.JointModelSphericalZYX())

        # 先从 URDF 建完整模型，再按需要锁住一部分关节，得到某个实验实际使用的 reduced model。
        self.robot = RobotWrapper.BuildFromURDF(urdf_path, [urdf_dir], joint_model)
        if lock_joints:
            self.robot = self.robot.buildReducedRobot(lock_joints)

        # OCP 中后续所有运动学、动力学和约束上界计算，都会复用这个 Pinocchio model/data。
        self.model = self.robot.model
        self.data = self.robot.data

        # 若给了 SRDF 和 reference pose，就把该姿态作为名义姿态 q0。
        # 论文中的 MPC 会围绕这个名义姿态对构型做正则化。
        if srdf_path and reference_pose:
            pin.loadReferenceConfigurations(self.model, srdf_path)
            self.q0 = self.model.referenceConfigurations[reference_pose]
        else:
            self.q0 = self.robot.q0

        # 这些维度会在整个优化问题中反复用到：
        # nq: 广义位置维度
        # nv: 广义速度维度
        # nj: 仅驱动关节维度，不包含 floating-base 的位置/姿态变量
        # nf: 外力变量维度，默认先只统计四个足端，机械臂末端力后面再加
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nj = self.nq - 7  # without base position and quaternion
        self.nf = 12  # forces at feet

        # Joint limits from URDF (exclude base indices)
        self.joint_pos_min = self.model.lowerPositionLimit[7:]
        self.joint_pos_max = self.model.upperPositionLimit[7:]
        self.joint_vel_max = self.model.velocityLimit[6:]
        self.joint_torque_max = self.model.effortLimit[6:]

        # 这两个字段是为后续多机器人适配预留的。
        # 原项目默认把 base frame 写死成 base_link，但 Go2+Piper 的 base_link 是机械臂底座，
        # 真正的四足机身 frame 叫 base，所以这里统一改成由机器人子类自己声明。
        self.base_frame_name = "base_link"
        self.arm_ee_frame_name = None

        # 基类默认表示纯 locomotion 机器人。
        # 若是 loco-manipulation 机器人，会在子类里重写这些字段。
        self.arm_ee_frame = None  # end-effector frame in URDF
        self.arm_joints = 0  # number of joints to consider (the other ones are locked)

    def set_gait_sequence(self, gait_type, gait_period):
        # 论文假设 gait schedule 作为已知输入提供给 MPC。
        # 这里创建对应的步态调度器，并记录各足端 frame 的 id。
        self.gait_sequence = GaitSequence(gait_type, gait_period)
        self.foot_frames = [self.model.getFrameId(f) for f in self.gait_sequence.feet]


class B2(Robot):
    def __init__(self, reference_pose="standing"):
        # 纯四足模型，用于不带机械臂的 locomotion 实验。
        urdf_path = "robots/b2_description/urdf/b2.urdf"
        srdf_path = "robots/b2_description/srdf/b2.srdf"
        super().__init__(urdf_path, srdf_path, reference_pose)

        # 原始 B2 资产中的机身 frame 就是 base_link。
        self.base_frame_name = "base_link"

        # 参考接触力在前后足之间按经验重量分布分配，
        # 这样做能让 warm start 的接触力初值更接近真实情况。
        self.front_force_ratio = 0.4


class B2_Z1(Robot):
    def __init__(self, reference_pose="standing_with_arm_up", arm_joints=6):
        # 移动操作模型：Unitree B2 四足底盘 + Z1 机械臂。
        urdf_path = "robots/b2_z1_description/urdf/b2_z1.urdf"
        srdf_path = "robots/b2_z1_description/srdf/b2_z1.srdf"

        # 论文中的 MPC 实验只保留部分 arm joints 为活动关节。
        # 其余关节全部锁住，得到规模更小、实时优化更容易的 reduced model。
        lock_idx = 14 + arm_joints  # 14 is for the universe (0), base (1), and the 4 legs (2-13)
        lock_joints = range(lock_idx, 21)  # 20 is the last joint (the gripper)

        super().__init__(urdf_path, srdf_path, reference_pose, lock_joints=lock_joints)
        self.arm_joints = arm_joints  # init sets it to 0

        # B2+Z1 沿用原仓库对机身和末端 frame 的命名假设。
        self.base_frame_name = "base_link"
        self.arm_ee_frame_name = "gripperCenter"

        if self.arm_joints > 0:
            # 在 loco-manipulation 建模里，机械臂末端允许对环境施加外力，
            # 因此 OCP 会在这里额外引入一个 3 维末端力变量。
            self.arm_ee_frame = self.model.getFrameId(self.arm_ee_frame_name, type=pin.FIXED_JOINT)
            self.nf += 3

        # 与 B2 相同，warm start 时仍使用近似的前后足载荷分配。
        self.front_force_ratio = 0.4


class Go2Piper(Robot):
    def __init__(self, reference_height=0.445, arm_joints=6):
        # 这个类专门用于把 Go2+Piper 组合模型接到 unitree_mujoco。
        # 这里直接使用已经在 MuJoCo 侧验证过的组合 URDF，而不是再复用 B2/Z1 的资产。
        if arm_joints < 0 or arm_joints > 6:
            raise ValueError("arm_joints must be in [0, 6] for Go2Piper")

        urdf_path = (
            "/home/wzx/WholeBodyRL_WS/UnitreeSim2Real/unitree_mujoco/"
            "unitree_robots/Go2Arm_description/urdf/go2_piper_description_mjc_NoGripper.urdf"
        )

        # 当前组合模型没有配套 SRDF，也没有论文仓库里的 reference pose，
        # 所以后面会手工构造一份 standing + arm home 的 q0。
        srdf_path = None
        reference_pose = None

        # Pinocchio 读取该 URDF 后，12 条腿关节后面紧跟 6 个机械臂关节。
        # 如果以后想做 reduced arm model，这里仍然保留和 B2_Z1 一样的锁关节入口。
        lock_joints = None
        if arm_joints < 6:
            lock_idx = 14 + arm_joints  # 14 = universe + base + 12 leg joints
            lock_joints = range(lock_idx, 20)  # 19 is joint6, so exclusive upper bound is 20

        super().__init__(urdf_path, srdf_path, reference_pose, lock_joints=lock_joints)

        # 对 Go2+Piper 来说，真正的机身 frame 是 base。
        # 如果这里继续沿用 base_link，后续 whole-body base pose / velocity 都会参考到机械臂底座。
        self.base_frame_name = "base"

        self.arm_joints = arm_joints
        self.arm_ee_frame_name = "link6" if arm_joints > 0 else None

        if self.arm_joints > 0:
            # NoGripper 模型没有 gripperCenter，所以先把 link6 当成末端控制 frame。
            # 以后如果在 URDF 里额外增加 ee_link，可以只改这个名字，不用改 OCP 主体。
            self.arm_ee_frame = self.model.getFrameId(self.arm_ee_frame_name)
            self.nf += 3

        # Go2 也沿用与 B2 相同的前后足载荷初始分配，先保证 warm start 可用。
        self.front_force_ratio = 0.4

        # 手工构造一份与 MuJoCo home keyframe 对齐的参考姿态。
        # Pinocchio free-flyer 的四元数顺序是 [x, y, z, w]，因此这里使用 [0, 0, 0, 1]。
        self.q0 = self._build_home_configuration(reference_height)

    def _build_home_configuration(self, reference_height):
        q0 = np.array(self.robot.q0, copy=True)

        # 浮动基座放在世界坐标系正立姿态下，z 高度先与 MuJoCo 模型初始高度保持一致。
        q0[:7] = np.array([0.0, 0.0, reference_height, 0.0, 0.0, 0.0, 1.0])

        # 这一组关节角与当前 MuJoCo WBIC 模型的 home keyframe 保持一致。
        # 顺序同样和现在的 actuator / sensor 顺序一致：FL, FR, RL, RR, joint1..joint6。
        # FL_hip, FL_thigh, FL_calf......
        joint_home = np.array([
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ])

        q0[7:] = joint_home[: self.nj]
        return q0
