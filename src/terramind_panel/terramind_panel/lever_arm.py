"""后轴中心到定位测量点的平面杆臂辨识；不依赖 ROS。"""
from dataclasses import dataclass
import math
import numpy as np
from .coverage import wrap


def base_yaw(quaternion_wxyz, mounting_wxyz):
    """q_map_base = q_map_imu * inverse(q_base_imu)，限制为平地辨识。"""
    q = np.asarray(quaternion_wxyz, dtype=float)
    r = np.asarray(mounting_wxyz, dtype=float)
    if q.shape != (4,) or r.shape != (4,) or not np.isfinite([q, r]).all():
        raise ValueError('安装四元数或定位四元数无效')
    if abs(np.linalg.norm(q)-1) > .01 or abs(np.linalg.norm(r)-1) > .01:
        raise ValueError('安装四元数或定位四元数未归一化')
    q /= np.linalg.norm(q); r /= np.linalg.norm(r)
    w = q[0]*r[0]+np.dot(q[1:],r[1:])
    x,y,z = -q[0]*r[1:]+r[0]*q[1:]-np.cross(q[1:],r[1:])
    # z 轴倾角限制，避免未识别的高度混入水平杆臂。
    if 1-2*(x*x+y*y) < math.cos(math.radians(5)):
        raise ValueError('车体倾斜超过 5 度，请在平地辨识并核对安装姿态')
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


@dataclass(frozen=True)
class Estimate:
    x: float
    y: float
    rms: float
    direction_difference: float
    samples: tuple


def _fit(samples):
    data = np.asarray(samples, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3 or len(data) < 60 or not np.isfinite(data).all():
        raise ValueError('每个方向至少需要 60 个有效位置/航向样本')
    yaw = np.unwrap(data[:,2])
    if abs(yaw[-1]-yaw[0]) < 2*math.pi-.10:
        raise ValueError('转动角度不足一整圈')
    # 均衡角度采样，避免停留在某一角度的大量样本支配拟合。
    bins = np.floor(((yaw-yaw[0]) % (2*math.pi))/(2*math.pi)*72).astype(int)
    occupied = sorted(set(bins))
    if len(occupied) < 68:
        raise ValueError('整圈角度采样不完整')
    weights = np.array([1/math.sqrt(np.count_nonzero(bins==b)) for b in bins])
    n = len(data); a = np.zeros((2*n,4)); b = data[:,:2].ravel()
    c,s = np.cos(data[:,2]),np.sin(data[:,2])
    a[0::2,:2] = np.column_stack([c,-s])
    a[1::2,:2] = np.column_stack([s,c])
    a[0::2,2] = 1; a[1::2,3] = 1
    solution,_,rank,_ = np.linalg.lstsq(a*np.repeat(weights,2)[:,None],b*np.repeat(weights,2),rcond=None)
    if rank != 4:
        raise ValueError('轨迹无法辨识杆臂')
    residual = (a@solution-b).reshape(-1,2)
    return solution[:2], float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))), residual


def estimate(ccw, cw, max_rms=.03, max_difference=.04):
    """每圈独立估计转心，允许两圈之间转心微移；拒绝滑移/反向不一致。"""
    if not (0 < max_rms <= .1 and 0 < max_difference <= .2):
        raise ValueError('辨识质量阈值超出范围')
    left, lrms, lres = _fit(ccw)
    right, rrms, rres = _fit(cw)
    lyaw = np.unwrap(np.asarray(ccw)[:,2]); ryaw = np.unwrap(np.asarray(cw)[:,2])
    if lyaw[-1] <= lyaw[0] or ryaw[-1] >= ryaw[0]:
        raise ValueError('必须包含逆时针与顺时针各一整圈')
    difference = float(np.linalg.norm(left-right))
    if max(lrms,rrms) > max_rms or max(np.linalg.norm(lres,axis=1).max(),np.linalg.norm(rres,axis=1).max()) > 4*max_rms:
        raise ValueError('轨迹残差过大，定位噪声或车轮滑移，结果不保存')
    if difference > max_difference:
        raise ValueError('正反向估计不一致，结果不保存')
    xy = (left+right)/2
    if np.linalg.norm(xy) > 3:
        raise ValueError('估计偏移超过 3 m，请核对定位参考点')
    return Estimate(float(xy[0]),float(xy[1]),max(lrms,rrms),difference,(len(ccw),len(cw)))


class SpinCalibration:
    """反馈旋转两圈，圈间停稳；调用方负责新鲜度、使能和故障停机。"""
    def __init__(self, yaw, now, speed=.4):
        if not math.isfinite(speed) or not .15 <= speed <= .6:
            raise ValueError('辨识角速度必须在 0.15～0.6 rad/s')
        self.speed = speed
        self.start_time = now
        self.last_yaw = yaw
        self.angle = self.origin = yaw
        self.phase = 'ccw'
        self.samples = [[],[]]
        self.settled_since = None
        self.last_time = now
        self.done = False

    def step(self, x, y, yaw, now):
        if self.done:
            return 0.
        if not all(math.isfinite(v) for v in (x,y,yaw,now)):
            raise ValueError('辨识输入不是有限值')
        dt = now-self.last_time
        if dt <= 0:
            raise ValueError('辨识样本时间未递增')
        change = wrap(yaw-self.last_yaw)
        if abs(change) > .10 + 1.2*dt:
            raise ValueError('定位航向发生跳变')
        self.angle += change; self.last_yaw = yaw; self.last_time = now
        if now-self.start_time > 4*math.pi/self.speed+45:
            raise ValueError('自转辨识超时')
        index = 0 if self.phase == 'ccw' else 1
        self.samples[index].append((x,y,yaw))
        target = self.origin+2*math.pi if index == 0 else self.origin
        error = target-self.angle
        if abs(error) <= .015:
            if abs(change)/dt < .025:
                if self.settled_since is None:
                    self.settled_since = now
                if now-self.settled_since >= .8:
                    if index == 0:
                        self.phase = 'cw'
                        self.samples[1].append((x,y,yaw))
                        self.settled_since = None
                    else:
                        self.phase = 'done'; self.done = True
            else:
                self.settled_since = None
            return 0.
        self.settled_since = None
        return max(-self.speed,min(self.speed,1.2*error))
