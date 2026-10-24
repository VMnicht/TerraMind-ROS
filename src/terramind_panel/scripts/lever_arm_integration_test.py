#!/usr/bin/env python3
"""独立仿真后轴 + 前置定位点 + PTY 导航 + 实际辨识 CLI；不访问硬件。"""
import binascii
from collections import deque
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import yaml


def main():
    os.environ['ROS_DOMAIN_ID']=str(100+os.getpid()%100)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from terramind_interfaces.msg import McuState,ControlStatus
    rclpy.init()
    node=rclpy.create_node('lever_arm_integration_test')
    pending=deque();board=[None];control=[None];poses=[];imu_poses=[]
    node.create_subscription(Odometry,'sim/ground_truth',lambda m:pending.append((time.monotonic(),m)),10)
    node.create_subscription(McuState,'mcu/state',lambda m:board.__setitem__(0,m),1)
    node.create_subscription(ControlStatus,'control/status',lambda m:control.__setitem__(0,m),1)
    node.create_subscription(PoseStamped,'navigation/pose',poses.append,qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'navigation/imu_pose',imu_poses.append,qos_profile_sensor_data)
    children=[];valid=True;sequence=0;measurement_time=1000.;last_send=0.
    signs=set();master,slave=os.openpty()
    frame=struct.Struct('<2sBBIdddf3f3f4fHH')
    offset=(.8,.12,.3);mount=.2
    with tempfile.TemporaryDirectory(prefix='terramind-lever-arm-') as directory:
        directory=Path(directory)
        logs=[]
        def launch(package,executable,*args):
            log=(directory/f'{len(logs)}-{executable}.log').open('w+')
            logs.append(log)
            process=subprocess.Popen([str(Path(get_package_prefix(package))/'lib'/package/executable),*args],
                                     stdout=log,stderr=subprocess.STDOUT)
            children.append(process)
            return process
        def stop(process):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        def pump():
            nonlocal sequence,measurement_time,last_send
            rclpy.spin_once(node,timeout_sec=.002)
            now=time.monotonic()
            if pending and now-pending[0][0]>=.2 and now-last_send>=.02:
                received,m=pending.popleft()
                while pending and now-pending[0][0]>=.2:
                    received,m=pending.popleft()
                q=m.pose.pose.orientation
                yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
                c,s=math.cos(yaw),math.sin(yaw)
                x=m.pose.pose.position.x+c*offset[0]-s*offset[1]
                y=m.pose.pose.position.y+s*offset[0]+c*offset[1]
                # 在米级小场地使用独立的 WGS84 曲率半径生成经纬度，误差 < 1 um。
                e2=6.6943799901413165e-3;lat0=.5;height=20.
                n=6378137/math.sqrt(1-e2*math.sin(lat0)**2)
                radius=6378137*(1-e2)/(1-e2*math.sin(lat0)**2)**1.5
                lat=lat0+y/(radius+height);lon=2.+x/((n+height)*math.cos(lat0))
                ned_yaw=math.pi/2-yaw-mount
                sequence+=1;measurement_time+=.02
                packet=bytearray(frame.pack(b'\xaa\x55',1,0x9D if valid else 0x94,sequence,
                    measurement_time,lat,lon,height+offset[2],0.,0.,0.,0.,0.,ned_yaw,
                    math.cos(ned_yaw/2),0.,0.,math.sin(ned_yaw/2),int(1000*(now-received)),0))
                struct.pack_into('<H',packet,78,binascii.crc_hqx(packet[2:78],0xFFFF))
                os.write(master,packet);last_send=now
                if m.twist.twist.angular.z>.1:signs.add(1)
                if m.twist.twist.angular.z<-.1:signs.add(-1)
            if board[0]:
                assert not any(getattr(board[0],key).on for key in ('left_spreader','right_spreader','mower','lift','sprayer'))
        def wait(predicate,timeout=10):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                pump()
                if predicate():return
            raise AssertionError('integration condition timed out')
        try:
            config=yaml.safe_load((Path(get_package_share_directory('terramind_bringup'))/'config/navigation.yaml').read_text())
            p=config['navigation_serial_node']['ros__parameters']
            p.update(device=os.ttyname(slave),allow_simulated=True,origin_mode='fixed',
                     origin_latitude_deg=math.degrees(.5),origin_longitude_deg=math.degrees(2.),origin_height_m=20.,
                     imu_rotation_in_base_wxyz=[math.cos(mount/2),0.,0.,math.sin(mount/2)],
                     imu_translation_in_base_m=[.35,-.1,.3],extrinsics_calibrated=True)
            source=directory/'navigation.yaml';source.write_text(yaml.safe_dump(config))
            output=directory/'identified.yaml'
            launch('terramind_sim','sim_interface_node','--ros-args','-p','capabilities:=63')
            launch('terramind_control','control_manager_node')
            driver=launch('terramind_navigation_driver','navigation_serial_node','--ros-args','--params-file',str(source))
            wait(lambda:board[0] is not None and poses and imu_poses)
            # 有旧补偿时，原始定位点仍与已补偿位置不同。
            assert math.hypot(poses[-1].pose.position.x-imu_poses[-1].pose.position.x,
                              poses[-1].pose.position.y-imu_poses[-1].pose.position.y)>.3
            cli=launch('terramind_panel','calibrate_lever_arm','--config',str(source),'--output',str(output))
            wait(lambda:cli.poll() is not None,80)
            assert cli.returncode==0
            result=yaml.safe_load(output.read_text())
            rp=result['navigation_serial_node']['ros__parameters']
            assert abs(rp['imu_translation_in_base_m'][0]-.8)<.003,rp
            assert abs(rp['imu_translation_in_base_m'][1]-.12)<.003,rp
            assert rp['imu_translation_in_base_m'][2]==.3 and rp['imu_rotation_in_base_wxyz']==p['imu_rotation_in_base_wxyz']
            assert not rp['extrinsics_calibrated'] and signs=={-1,1}
            wait(lambda:control[0] is not None and not control[0].enabled and board[0].mode==2)
            print('PASS actual CLI: both rotations, 200 ms delay, recovered offset (0.8, 0.12), all implements off, stopped/disarmed',flush=True)

            # 应用识别结果后，前置定位点绕圈，补偿点应回到后轴中心。
            stop(driver);rp['extrinsics_calibrated']=True;output.write_text(yaml.safe_dump(result))
            poses.clear();imu_poses.clear()
            driver=launch('terramind_navigation_driver','navigation_serial_node','--ros-args','--params-file',str(output))
            wait(lambda:len(poses)>10)
            pose=poses[-1].pose
            assert math.hypot(pose.position.x,pose.position.y)<.003
            assert abs(pose.position.z)<.003
            print('PASS applied configuration: corrected rear axle position within 3 mm; height and mounting rotation preserved',flush=True)

            # 导航中断必须停机，且不能产生一个看似成功的配置文件。
            failed_output=directory/'must-not-exist.yaml'
            cli=launch('terramind_panel','calibrate_lever_arm','--config',str(output),'--output',str(failed_output))
            wait(lambda:control[0].enabled)
            end=time.monotonic()+.5;wait(lambda:time.monotonic()>end)
            valid=False
            wait(lambda:cli.poll() is not None,5)
            assert cli.returncode!=0 and not failed_output.exists()
            wait(lambda:not control[0].enabled and board[0].mode==2)
            valid=True
            end=time.monotonic()+.5;wait(lambda:time.monotonic()>end)
            assert not control[0].enabled
            print('PASS navigation invalidation: stopped, no result written, no automatic restart',flush=True)

            aborted=directory/'aborted.yaml'
            cli=launch('terramind_panel','calibrate_lever_arm','--config',str(output),'--output',str(aborted))
            wait(lambda:control[0].enabled)
            cli.send_signal(signal.SIGINT)
            wait(lambda:cli.poll() is not None,5)
            assert cli.returncode==1 and not aborted.exists()
            wait(lambda:not control[0].enabled and board[0].mode==2)
            print('PASS Ctrl+C: explicit stop before ROS shutdown, no result written',flush=True)
        except BaseException:
            for log in logs:
                log.flush();log.seek(0);print(log.name+'\n'+log.read(),flush=True)
            raise
        finally:
            for child in reversed(children):stop(child)
            for log in logs:log.close()
            os.close(master);os.close(slave)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
