import threading
import time

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC

from utils.robot import Go2Piper
from utils.state_adapter import Go2PiperStateAdapter


TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"


class Go2PiperWbicNode:
    def __init__(self, domain_id=1, interface="lo"):
        # 这里保留最小必要依赖：
        # 1. 订阅仿真状态并转换成 WBIC 需要的 q/v
        # 2. 发布 18 关节的 MIT 形式 LowCmd
        self.domain_id = domain_id
        self.interface = interface

        self.robot = Go2Piper()
        self.adapter = Go2PiperStateAdapter(num_joints=self.robot.nj)

        self._lock = threading.Lock()
        self._latest_low_state = None
        self._latest_high_state = None

        ChannelFactoryInitialize(self.domain_id, self.interface)

        self.low_state_sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self.high_state_sub = ChannelSubscriber(TOPIC_HIGHSTATE, SportModeState_)
        self.low_cmd_pub = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)

        self.low_state_sub.Init(self._low_state_handler, 10)
        self.high_state_sub.Init(self._high_state_handler, 10)
        self.low_cmd_pub.Init()

        self.crc = CRC()

    def _low_state_handler(self, msg: LowState_):
        with self._lock:
            self._latest_low_state = msg

    def _high_state_handler(self, msg: SportModeState_):
        with self._lock:
            self._latest_high_state = msg

    def have_state(self):
        with self._lock:
            return self._latest_low_state is not None and self._latest_high_state is not None

    def wait_for_state(self, timeout=5.0, sleep_dt=0.01):
        # 控制器启动后通常需要等第一帧 low/high state 到达，再开始求解和发命令。
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.have_state():
                return True
            time.sleep(sleep_dt)
        return False

    def latest_raw_state(self):
        # 如果后面需要直接访问 DDS 原始消息，而不是 q/v，这个接口可以直接复用。
        with self._lock:
            return self._latest_low_state, self._latest_high_state

    def latest_qv(self):
        # 这是上层控制器最常用的入口：
        # - q: Pinocchio free-flyer 配置 [base_pos, base_quat_xyzw, joints]
        # - v: Pinocchio 广义速度 [base_lin_vel, base_ang_vel, joint_vel]
        with self._lock:
            if self._latest_low_state is None or self._latest_high_state is None:
                return None, None
            q, v = self.adapter.to_qv(self._latest_low_state, self._latest_high_state)
        return q, v

    def latest_snapshot(self):
        # 调试或日志场景下，如果需要单独看 base/joint/torque 分项，直接取这个结构化快照。
        with self._lock:
            if self._latest_low_state is None or self._latest_high_state is None:
                return None
            snapshot = self.adapter.debug_snapshot(self._latest_low_state, self._latest_high_state)
        return snapshot

    def _build_lowcmd_template(self):
        # unitree_go 消息有 20 个 motor slot，但当前模型只使用前 18 个。
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        for i in range(20):
            cmd.motor_cmd[i].mode = 0x01
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0

        return cmd

    def build_lowcmd(self, q_des=None, kp=None, dq_des=None, kd=None, tau=None):
        # MuJoCo 桥接端按标准 MIT 公式执行：
        # tau + kp * (q_des - q) + kd * (dq_des - dq)
        # 因此这里直接提供这 5 组 18 维数组的打包接口。
        cmd = self._build_lowcmd_template()
        num_motor = self.robot.nj

        def _as_array(value, default):
            if value is None:
                return np.full(num_motor, default, dtype=float)
            arr = np.asarray(value, dtype=float)
            if arr.shape != (num_motor,):
                raise ValueError(f"expected shape ({num_motor},), got {arr.shape}")
            return arr

        q_des = _as_array(q_des, 0.0)
        kp = _as_array(kp, 0.0)
        dq_des = _as_array(dq_des, 0.0)
        kd = _as_array(kd, 0.0)
        tau = _as_array(tau, 0.0)

        for i in range(num_motor):
            cmd.motor_cmd[i].q = q_des[i]
            cmd.motor_cmd[i].kp = kp[i]
            cmd.motor_cmd[i].dq = dq_des[i]
            cmd.motor_cmd[i].kd = kd[i]
            cmd.motor_cmd[i].tau = tau[i]

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def publish_lowcmd(self, q_des=None, kp=None, dq_des=None, kd=None, tau=None):
        cmd = self.build_lowcmd(q_des=q_des, kp=kp, dq_des=dq_des, kd=kd, tau=tau)
        self.low_cmd_pub.Write(cmd)
        return cmd

    def spin(self, sleep_dt=1.0):
        # 如果只想让这个节点常驻并维持 DDS 订阅线程，可直接调用 spin。
        while True:
            time.sleep(sleep_dt)


def main():
    node = Go2PiperWbicNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
