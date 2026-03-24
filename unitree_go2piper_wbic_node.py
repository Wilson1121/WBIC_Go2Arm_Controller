import threading
import time

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

from utils.robot import Go2Piper
from utils.state_adapter import Go2PiperStateAdapter


TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"


class Go2PiperWbicNode:
    def __init__(self, domain_id=1, interface="lo", print_dt=0.2):
        # 这里沿用 unitree_mujoco 当前使用的 DDS 配置。
        # 后面如果仿真端改了 domain/interface，这里也需要同步修改。
        self.domain_id = domain_id
        self.interface = interface
        self.print_dt = print_dt

        # 机器人模型对象在这一版里主要用于做维度核对。
        # 后面真正接入 OCP / WBIC 时，也会复用这个对象。
        self.robot = Go2Piper()
        self.adapter = Go2PiperStateAdapter(num_joints=self.robot.nj)

        # DDS 回调是异步触发的，因此 low/high state 需要锁保护。
        # 第一版只做状态读入和打印，还不涉及控制命令发布。
        self._lock = threading.Lock()
        self._latest_low_state = None
        self._latest_high_state = None
        self._last_print_time = 0.0

        ChannelFactoryInitialize(self.domain_id, self.interface)

        self.low_state_sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self.high_state_sub = ChannelSubscriber(TOPIC_HIGHSTATE, SportModeState_)

        self.low_state_sub.Init(self._low_state_handler, 10)
        self.high_state_sub.Init(self._high_state_handler, 10)

    def _low_state_handler(self, msg: LowState_):
        # 低层状态里包含：
        # - 18 个电机的 q / dq / tau_est
        # - IMU quaternion / gyro / accel
        with self._lock:
            self._latest_low_state = msg

    def _high_state_handler(self, msg: SportModeState_):
        # 高层状态里当前主要使用：
        # - base position
        # - base linear velocity
        with self._lock:
            self._latest_high_state = msg

    def have_state(self):
        with self._lock:
            return self._latest_low_state is not None and self._latest_high_state is not None

    def latest_qv(self):
        # 这个接口把 DDS 消息转换成 Pinocchio / WBIC 需要的 q, v。
        # 如果四元数顺序或 base 速度坐标系后面发现需要调整，
        # 应该优先改 state_adapter，而不是在控制器里到处打补丁。
        with self._lock:
            if self._latest_low_state is None or self._latest_high_state is None:
                return None, None
            q, v = self.adapter.to_qv(self._latest_low_state, self._latest_high_state)
        return q, v

    def latest_snapshot(self):
        # 调试入口：返回更完整的结构化状态，便于打印和人工核对。
        with self._lock:
            if self._latest_low_state is None or self._latest_high_state is None:
                return None
            snapshot = self.adapter.debug_snapshot(self._latest_low_state, self._latest_high_state)
        return snapshot

    def _format_array(self, arr, precision=3):
        return np.array2string(np.asarray(arr), precision=precision, suppress_small=True)

    def print_snapshot(self):
        snapshot = self.latest_snapshot()
        if snapshot is None:
            print("waiting for lowstate/highstate ...")
            return

        q = snapshot["q"]
        v = snapshot["v"]

        print("=" * 80)
        print("Go2Piper DDS snapshot")
        print("joint_order:", snapshot["joint_names"])
        print("base_position:", self._format_array(snapshot["base_position"]))
        print("base_quaternion_xyzw:", self._format_array(snapshot["base_quaternion_xyzw"]))
        print("base_linear_velocity:", self._format_array(snapshot["base_linear_velocity"]))
        print("base_angular_velocity:", self._format_array(snapshot["base_angular_velocity"]))
        print("joint_positions:", self._format_array(snapshot["joint_positions"]))
        print("joint_velocities:", self._format_array(snapshot["joint_velocities"]))
        print("joint_torques:", self._format_array(snapshot["joint_torques"]))
        print("q shape:", q.shape, "q:", self._format_array(q))
        print("v shape:", v.shape, "v:", self._format_array(v))

        # 顺手做一层维度检查，避免后面接入 OCP 时才发现 q/v 维度不对。
        print("robot nq/nv:", self.robot.nq, self.robot.nv)
        if q.shape[0] != self.robot.nq:
            print(f"warning: q length mismatch, expected {self.robot.nq}, got {q.shape[0]}")
        if v.shape[0] != self.robot.nv:
            print(f"warning: v length mismatch, expected {self.robot.nv}, got {v.shape[0]}")

    def spin(self):
        # 第一版节点是一个纯状态观察器：
        # - 不发布控制命令
        # - 不跑 OCP
        # - 只周期性打印已经映射好的 q/v
        print(
            f"Go2PiperWbicNode listening on domain_id={self.domain_id}, interface={self.interface}"
        )
        print("topics:", TOPIC_LOWSTATE, TOPIC_HIGHSTATE)
        print("expected joint order:", self.adapter.joint_names())

        while True:
            now = time.time()
            if now - self._last_print_time >= self.print_dt:
                self.print_snapshot()
                self._last_print_time = now
            time.sleep(0.01)


def main():
    node = Go2PiperWbicNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        print("Go2PiperWbicNode stopped")


if __name__ == "__main__":
    main()
