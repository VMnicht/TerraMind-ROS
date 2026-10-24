"""导航状态呈现；只显示事实，不将导航状态用作手动控制的使能条件。"""


def describe_navigation(status, fresh):
    if status is None:
        return '等待导航板状态；在调试总面板的实机模式中启用导航。'
    flag = lambda value: '是' if value else '否'
    rtk = {0: '无有效定位', 1: '单点', 2: '差分', 3: '浮点解', 4: '固定解', 7: '未知'}
    age = '未知' if status.age_ms == 65535 else f'{status.age_ms} ms（板端打包时）'
    lines = [
        f'状态消息：{"及时" if fresh else "已过期，以下仅为最后记录"}',
        f'串口在线：{flag(fresh and status.online)}  导航可用：{flag(fresh and status.usable and status.origin_ready)}',
        f'RTK：{rtk.get(status.rtk_state, "保留类型")}  结果年龄：{age}',
        f'时间锁定：{flag(status.time_locked)}  近期 GNSS 更新：{flag(status.gnss_recent)}',
        f'原点已建立：{flag(status.origin_ready)}  外参已标定：{flag(status.extrinsics_calibrated)}',
        f'位姿参考点：{status.pose_child_frame}  速度参考点：{status.velocity_point_frame}',
        f'坐标系：{status.header.frame_id}  时间戳：{status.stamp_reference}（不是测量时刻）',
        f'会话：{status.session_id}  序号：{status.packet_seq}  模拟数据：{flag(status.simulated)}',
        f'接收：{status.received_frames}  CRC 错误：{status.crc_errors}  丢帧：{status.missing_packets}',
        f'重启：{status.board_restarts}  重连：{status.reconnects}',
        f'原因：{status.reason}',
    ]
    if status.origin_ready:
        import math
        lines.append(f'原点：{math.degrees(status.origin_latitude_rad):.9f}°, '
                     f'{math.degrees(status.origin_longitude_rad):.9f}°, '
                     f'椭球高 {status.origin_height_m:.3f} m')
    return '\n'.join(lines)
