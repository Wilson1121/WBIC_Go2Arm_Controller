from os.path import dirname, abspath

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

        # 基类默认表示纯 locomotion 机器人。
        # 若是 loco-manipulation 机器人，会在子类里重写这两个字段。
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

        if self.arm_joints > 0:
            # 在 loco-manipulation 建模里，机械臂末端允许对环境施加外力，
            # 因此 OCP 会在这里额外引入一个 3 维末端力变量。
            self.arm_ee_frame = self.model.getFrameId("gripperCenter", type=pin.FIXED_JOINT)
            self.nf += 3

        # 与 B2 相同，warm start 时仍使用近似的前后足载荷分配。
        self.front_force_ratio = 0.4
