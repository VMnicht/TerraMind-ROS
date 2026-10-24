"""复用面板控制端点的辨识程序：唯一发布者、相同看门狗和停机路径。"""
import math
import time
from collections import deque
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from .controls import Selection
from .coverage import wrap
from .lever_arm import SpinCalibration, base_yaw, estimate


class LeverArmRuntime:
    def __init__(self, node, mounting_wxyz, speed=.4):
        self.node = node
        self.mounting = tuple(mounting_wxyz)
        base_yaw(self.mounting,self.mounting)  # 在使能前验证安装姿态参数。
        self.speed = speed
        self.sample = None
        self.stamp = -1
        # 保留 0.8 秒窗口；容量覆盖导航的 100/200 Hz，不能因频率高而永远无法停稳。
        self.history = deque(maxlen=300)
        self.identity = None
        self.spin = None
        self.result = None
        self.error = ''
        self.running = False
        self.last_consumed = -1
        node.create_subscription(PoseStamped,'navigation/imu_pose',self.receive,qos_profile_sensor_data)

    def nav_identity(self):
        s = self.node.navigation
        if s is None or self.node.board is None:
            return None
        return (s.header.frame_id,s.session_id,s.origin_latitude_rad,
                s.origin_longitude_rad,s.origin_height_m,self.node.board.connection_id)

    def receive(self, message):
        received = self.node._source_time(message.header)
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        identity = self.nav_identity()
        if received is None or identity is None or message.header.frame_id != identity[0]:
            return
        if self.sample and self.sample[2] == identity and stamp <= self.stamp:
            return
        try:
            p,q = message.pose.position,message.pose.orientation
            yaw = base_yaw((q.w,q.x,q.y,q.z),self.mounting)
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
                raise ValueError('定位位置无效')
        except ValueError as error:
            if self.running:
                self.fail(str(error))
            self.sample = None
            return
        if self.sample is None or self.sample[2] != identity:
            self.history.clear()
        self.sample = ((p.x,p.y,yaw),received,identity)
        self.stamp = stamp
        self.history.append((received,p.x,p.y,yaw))
        while len(self.history)>1 and received-self.history[0][0] > .8:
            self.history.popleft()

    def ready(self):
        error = self.node.health_error() or self.node.input_conflict()
        if error:
            raise ValueError(error)
        s = self.node.navigation
        now = time.monotonic()
        if s is None or now-self.node.navigation_time > self.node.timeout:
            raise ValueError('导航状态过期')
        if not s.usable or not s.origin_ready or not s.nav_valid or s.nav_fault:
            raise ValueError('导航不可用于辨识')
        if not s.gnss_recent or s.rtk_state != 4:
            raise ValueError('辨识需要近期 GNSS 更新及 RTK 固定解')
        if s.simulated and not self.node.board.simulated:
            raise ValueError('真实控制板禁止使用模拟导航')
        if s.age_ms == 65535 or s.age_ms/1000+now-self.node.navigation_time > .5:
            raise ValueError('导航测量年龄超过 500 ms')
        if self.sample is None or now-self.sample[1] > self.node.timeout:
            raise ValueError('未收到新鲜的未补偿定位点')
        if self.sample[2] != self.nav_identity():
            raise ValueError('定位会话已改变')
        return self.sample[0]

    def start(self):
        pose = self.ready()
        if len(self.history)<10 or self.history[-1][0]-self.history[0][0] < .5:
            raise ValueError('等待至少 0.5 秒的停稳定位数据')
        if any(math.hypot(h[1]-pose[0],h[2]-pose[1])>.03 or abs(wrap(h[3]-pose[2]))>.025
               for h in self.history):
            raise ValueError('车辆尚未停稳或定位不稳定')
        # start() 使用原有 Stop -> Enable 握手，所有机具保持关闭。
        self.spin = SpinCalibration(pose[2],time.monotonic(),self.speed)
        self.identity = self.nav_identity()
        self.node.start(Selection())
        self.running = True

    def fail(self, error):
        self.error = error
        self.running = False
        self.node.stop('杆臂辨识终止：'+error)

    def tick(self):
        if not self.running:
            return
        try:
            pose = self.ready()
            if self.nav_identity() != self.identity:
                raise ValueError('定位原点、导航会话或控制板连接已改变')
            if self.node.phase == 'idle':
                raise ValueError(self.node.reason)
            if self.node.phase != 'running':
                return
            if self.stamp != self.last_consumed:
                self.last_consumed = self.stamp
                omega = self.spin.step(*pose,time.monotonic())
                self.node.active = Selection(angular_radps=omega)
                if self.spin.done:
                    # 先停机，再计算/保存；拟合失败也不得留下运动命令。
                    self.running = False
                    self.node.stop('两圈自转结束，正在检查辨识结果')
                    self.result = estimate(*self.spin.samples)
        except ValueError as error:
            self.fail(str(error))
