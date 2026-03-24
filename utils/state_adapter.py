import numpy as np


class Go2PiperStateAdapter:
    """
    将 unitree_mujoco 通过 DDS 发布的 LowState / SportModeState
    转成 WBIC / Pinocchio 使用的 q, v 向量。

    当前适配器对应的驱动关节顺序是：
    FL, FR, RL, RR, joint1..joint6

    这和当前 Go2+Piper WBIC 模型中的 actuator / sensor 顺序保持一致，
    也和 Pinocchio 读取组合 URDF 后的活动关节顺序一致。
    """

    JOINT_ORDER = [
        "FL_hip", "FL_thigh", "FL_calf",
        "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf",
        "RR_hip", "RR_thigh", "RR_calf",
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    ]

    def __init__(self, num_joints=18, imu_quat_order="wxyz"):
        # Go2+Piper 当前是 12 条腿关节 + 6 个机械臂关节。
        # 保留 num_joints 参数是为了以后若做 reduced arm model，
        # 适配器仍然可以只截取前若干个驱动关节。
        self.num_joints = num_joints

        # Pinocchio free-flyer 的四元数顺序是 [x, y, z, w]。
        # 而 MuJoCo 的 framequat / 现有 bridge 通常按 [w, x, y, z] 发布，
        # 因此这里默认做 wxyz -> xyzw 的重排。
        self.imu_quat_order = imu_quat_order

        if self.num_joints < 0 or self.num_joints > len(self.JOINT_ORDER):
            raise ValueError(f"num_joints must be in [0, {len(self.JOINT_ORDER)}]")
        if self.imu_quat_order not in {"wxyz", "xyzw"}:
            raise ValueError('imu_quat_order must be either "wxyz" or "xyzw"')

    def joint_names(self):
        # 暴露当前适配器所采用的关节顺序，便于 DDS 节点打印和交叉检查。
        return self.JOINT_ORDER[: self.num_joints]

    def joint_positions(self, low_state):
        # unitree_mujoco 的 bridge 会把每个驱动关节的 q 填到 low_state.motor_state[i].q。
        # 这里直接按当前约定顺序截取前 num_joints 个电机状态。
        q_j = np.array([low_state.motor_state[i].q for i in range(self.num_joints)], dtype=float)
        return q_j

    def joint_velocities(self, low_state):
        # 与 joint_positions 同理，dq 直接来自 motor_state[i].dq。
        v_j = np.array([low_state.motor_state[i].dq for i in range(self.num_joints)], dtype=float)
        return v_j

    def joint_torques(self, low_state):
        # 这个量当前不直接进入 q/v，但保留下来方便后续调试控制闭环和对比 tau_est。
        tau_j = np.array([low_state.motor_state[i].tau_est for i in range(self.num_joints)], dtype=float)
        return tau_j

    def base_position(self, high_state):
        # unitree_mujoco 侧把 MuJoCo 的 frame_pos 写进了 SportModeState.position。
        # 对 Go2+Piper 来说，这里期望的是机身 base 的世界坐标位置。
        return np.array(high_state.position[:3], dtype=float)

    def base_linear_velocity(self, high_state):
        # unitree_mujoco 侧把 MuJoCo 的 frame_vel 写进了 SportModeState.velocity。
        # 第一版适配器先原样转发这个三维线速度；如果后面验证到坐标系不一致，
        # 再在这里集中修改，而不是把坐标系处理散落在控制器代码里。
        return np.array(high_state.velocity[:3], dtype=float)

    def base_quaternion_xyzw(self, low_state):
        quat = np.array(low_state.imu_state.quaternion[:4], dtype=float)

        # Pinocchio 需要 [x, y, z, w]。
        # 如果 DDS 已经是 xyzw，就直接返回；
        # 如果 DDS 还是更常见的 wxyz，就统一在这里重排一次。
        if self.imu_quat_order == "xyzw":
            return quat
        return np.array([quat[1], quat[2], quat[3], quat[0]], dtype=float)

    def base_angular_velocity(self, low_state):
        # 角速度直接使用 imu_state.gyroscope。
        # 这里默认其顺序已经是 [wx, wy, wz]。
        return np.array(low_state.imu_state.gyroscope[:3], dtype=float)

    def base_linear_acceleration(self, low_state):
        # 当前 WBIC 状态构造暂时不直接用加速度，
        # 但单独暴露出来，方便后面做观测器、估计器或调试日志。
        return np.array(low_state.imu_state.accelerometer[:3], dtype=float)

    def to_q(self, low_state, high_state):
        # Pinocchio free-flyer 配置向量 q 的定义：
        # [base_pos(3), base_quat_xyzw(4), joint_pos(nj)]
        q_base = self.base_position(high_state)
        q_quat = self.base_quaternion_xyzw(low_state)
        q_j = self.joint_positions(low_state)
        return np.concatenate((q_base, q_quat, q_j))

    def to_v(self, low_state, high_state):
        # Pinocchio free-flyer 速度向量 v 的定义：
        # [base_linear_vel(3), base_angular_vel(3), joint_vel(nj)]
        v_lin = self.base_linear_velocity(high_state)
        v_ang = self.base_angular_velocity(low_state)
        v_j = self.joint_velocities(low_state)
        return np.concatenate((v_lin, v_ang, v_j))

    def to_qv(self, low_state, high_state):
        # 实时控制节点最常用的入口：一次性返回 WBIC 所需的 q, v。
        q = self.to_q(low_state, high_state)
        v = self.to_v(low_state, high_state)
        return q, v

    def debug_snapshot(self, low_state, high_state):
        # 这个辅助接口不参与控制，只用于打印和肉眼核对映射结果。
        q, v = self.to_qv(low_state, high_state)
        return {
            "joint_names": self.joint_names(),
            "base_position": self.base_position(high_state),
            "base_quaternion_xyzw": self.base_quaternion_xyzw(low_state),
            "base_linear_velocity": self.base_linear_velocity(high_state),
            "base_angular_velocity": self.base_angular_velocity(low_state),
            "joint_positions": self.joint_positions(low_state),
            "joint_velocities": self.joint_velocities(low_state),
            "joint_torques": self.joint_torques(low_state),
            "q": q,
            "v": v,
        }
