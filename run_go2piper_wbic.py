import time

import numpy as np

from unitree_go2piper_wbic_node import Go2PiperWbicNode


class Go2PiperWbicController:
    def __init__(
        self,
        domain_id=1,
        interface="lo",
        control_dt=0.005,       # 200 Hz control loop
        state_timeout=5.0,
    ):
        # 这个入口脚本只保留最小闭环：
        # 1. 从 DDS 订阅仿真状态
        # 2. 组装 q/v
        # 3. 计算一帧 18 关节 MIT 命令
        # 4. 发布到 rt/lowcmd
        self.node = Go2PiperWbicNode(domain_id=domain_id, interface=interface)
        self.robot = self.node.robot

        self.control_dt = control_dt
        self.state_timeout = state_timeout

        # 当前版本先把参考位形定义成 robot.q0 对应的 nominal 关节位形。
        # 这样控制入口从一开始就对齐模型名义姿态，后面接真正 WBIC 时也能直接复用这组参考。
        self._q_nominal = np.array(self.robot.q0[7 : 7 + self.robot.nj], dtype=float)
        self.transition_duration = 1.0
        self._q_start = None
        self._transition_start_time = None

        # 先给一组保守的 MIT 增益。
        # 关节顺序必须和 WBIC XML / state_adapter 保持一致：
        # FL_hip, FL_thigh, FL_calf,
        # FR_hip, FR_thigh, FR_calf,
        # RL_hip, RL_thigh, RL_calf,
        # RR_hip, RR_thigh, RR_calf,
        # joint1, joint2, joint3, joint4, joint5, joint6
        self.kp = np.array([
            60.0, 80.0, 100.0,  # FL_hip, FL_thigh, FL_calf
            60.0, 80.0, 100.0,  # FR_hip, FR_thigh, FR_calf
            60.0, 80.0, 100.0,  # RL_hip, RL_thigh, RL_calf
            60.0, 80.0, 100.0,  # RR_hip, RR_thigh, RR_calf
            10.0, 10.0, 10.0, 10.0, 10.0, 10.0,  # joint1..joint6
        ], dtype=float)
        self.kd = np.array([
            2.0, 2.0, 2.0,  # FL_hip, FL_thigh, FL_calf
            2.0, 2.0, 2.0,  # FR_hip, FR_thigh, FR_calf
            2.0, 2.0, 2.0,  # RL_hip, RL_thigh, RL_calf
            2.0, 2.0, 2.0,  # RR_hip, RR_thigh, RR_calf
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # joint1..joint6
        ], dtype=float)

    def wait_until_ready(self):
        if not self.node.wait_for_state(timeout=self.state_timeout):
            raise RuntimeError(
                f"did not receive lowstate/highstate within {self.state_timeout} seconds"
            )

    def nominal_joint_pose(self):
        # Pinocchio free-flyer 配置的前 7 维是 base，后面 18 维才是驱动关节。
        return self._q_nominal.copy()

    def capture_start_pose(self):
        q, _ = self.node.latest_qv()
        if q is None:
            raise RuntimeError("state is not available yet")

        # 控制刚接管时先记住当前关节角，后面 q_des 从这组值平滑过渡到 nominal。
        self._q_start = np.array(q[7 : 7 + self.robot.nj], dtype=float)
        self._transition_start_time = time.time()
        return self._q_start.copy()

    def _blended_nominal_joint_pose(self):
        if self._q_start is None or self._transition_start_time is None:
            return self.nominal_joint_pose()

        elapsed = time.time() - self._transition_start_time
        alpha = min(max(elapsed / self.transition_duration, 0.0), 1.0)
        return (1.0 - alpha) * self._q_start + alpha * self._q_nominal

    def compute_command(self, q, v):
        # 这是当前控制入口里唯一需要被真正 WBIC 替换的函数。
        # 现在先返回一个“平滑过渡到 nominal 位形”的 MIT 命令，避免刚启动控制时阶跃过大。
        q_des = self._blended_nominal_joint_pose()
        dq_des = np.zeros(self.robot.nj, dtype=float)
        tau = np.zeros(self.robot.nj, dtype=float)

        return {
            "q_des": q_des,
            "kp": self.kp,
            "dq_des": dq_des,
            "kd": self.kd,
            "tau": tau,
        }

    def step(self):
        q, v = self.node.latest_qv()
        if q is None or v is None:
            raise RuntimeError("state became unavailable during control loop")

        cmd = self.compute_command(q, v)
        self.node.publish_lowcmd(**cmd)
        return q, v, cmd

    def run(self):
        self.wait_until_ready()
        self.capture_start_pose()

        print(
            f"Go2PiperWbicController running: nj={self.robot.nj}, control_dt={self.control_dt}"
        )

        next_tick = time.time()
        while True:
            self.step()
            next_tick += self.control_dt
            sleep_time = next_tick - time.time()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                # 如果控制循环偶尔超时，直接跳到当前时刻继续，避免累计漂移。
                next_tick = time.time()


def main():
    controller = Go2PiperWbicController()
    try:
        controller.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
