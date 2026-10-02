#!/usr/bin/env python3
"""显式使能并在限定时间内发送新鲜指令，退出时请求停止；也可控制实机。"""
import argparse
from collections import deque
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from terramind_interfaces.msg import ImplementCommand, McuState
from std_srvs.srv import SetBool, Trigger


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--linear', type=float, default=0.)
    p.add_argument('--angular', type=float, default=0.)
    p.add_argument('--left-rpm', type=float, default=0.)
    p.add_argument('--right-rpm', type=float, default=0.)
    p.add_argument('--mower-percent', type=float, default=0.)
    p.add_argument('--sprayer-percent', type=float, default=0., help='sprayer target 0..100 percent; requires capability bit5')
    p.add_argument('--duration', type=float, default=3.)
    a = p.parse_args()
    for value, low, high in [(a.linear,-.35,.35),(a.angular,-2,2),(a.left_rpm,-500,500),
                             (a.right_rpm,-500,500),(a.mower_percent,0,100),(a.sprayer_percent,0,100),(a.duration,.02,3600)]:
        if not math.isfinite(value) or not low <= value <= high:
            p.error('a value is outside its protocol range')
    rclpy.init()
    n = Node('terramind_control_cli')
    velocity = n.create_publisher(TwistStamped, 'cmd_vel', 1)
    implements = n.create_publisher(ImplementCommand, 'implements/command', 1)
    enable = n.create_client(SetBool, 'control/set_enabled')
    stop = n.create_client(Trigger, 'control/stop')
    states = deque(maxlen=1)
    subscription = n.create_subscription(McuState, 'mcu/state', lambda m: states.append(m), 1)

    def call(client, request):
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('control service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(n, future, timeout_sec=3)
        if not future.done() or future.exception():
            raise RuntimeError('control service timeout')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)

    try:
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=.02)
            if states and states[-1].link_ready and velocity.get_subscription_count() and implements.get_subscription_count():
                break
        else:
            raise RuntimeError('no ready backend or command subscribers')
        if a.sprayer_percent > 0 and not states[-1].capabilities & 32:
            raise RuntimeError('backend does not report sprayer capability (bit5)')
        print(f'Enabling {"SIMULATION" if states[-1].simulated else "REAL HARDWARE"} for {a.duration:g} s')
        req = SetBool.Request(); req.data = True; call(enable, req)
        deadline = time.monotonic()+a.duration
        while time.monotonic() < deadline:
            stamp = n.get_clock().now().to_msg()
            v = TwistStamped(); v.header.stamp = stamp; v.header.frame_id = 'base_link'
            v.twist.linear.x = a.linear; v.twist.angular.z = a.angular; velocity.publish(v)
            m = ImplementCommand(); m.header = v.header
            m.left_on = a.left_rpm != 0; m.left_rpm = a.left_rpm
            m.right_on = a.right_rpm != 0; m.right_rpm = a.right_rpm
            m.mower_on = a.mower_percent > 0; m.mower_percent = a.mower_percent
            m.sprayer_on = a.sprayer_percent > 0; m.sprayer_percent = a.sprayer_percent
            implements.publish(m)
            rclpy.spin_once(n, timeout_sec=0)
            time.sleep(.02)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            call(stop, Trigger.Request())
            print('Stop requested; check mcu/state for board acceptance.')
        finally:
            n.destroy_subscription(subscription); n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
