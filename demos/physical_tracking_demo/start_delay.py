"""单线程启动倒计时：不发送控制，失联、取消或调度中断后不自动恢复。"""
import math


class DelayedStart:
    def __init__(self,start,validate,on_failure):
        self.start=start;self.validate=validate;self.on_failure=on_failure
        self.deadline=None;self.last_tick=None

    @property
    def pending(self):return self.deadline is not None

    def arm(self,seconds,now):
        if self.pending:raise ValueError('启动倒计时已经开始')
        if not math.isfinite(seconds) or not 0<=seconds<=120:raise ValueError('启动延时范围为 0～120 秒')
        self.validate()
        self.deadline=now+seconds;self.last_tick=now

    def cancel(self):self.deadline=None;self.last_tick=None

    def remaining(self,now):return max(0.,self.deadline-now) if self.pending else 0.

    def tick(self,now):
        if not self.pending:return
        try:
            if not math.isfinite(now) or now<self.last_tick or now-self.last_tick>.15:
                raise ValueError('倒计时期间界面响应超时')
            self.last_tick=now
            self.validate()
            if now>=self.deadline:
                self.cancel()
                self.start()
        except Exception as error:
            self.cancel();self.on_failure('启动倒计时已取消：'+str(error))
