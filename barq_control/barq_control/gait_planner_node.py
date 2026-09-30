"""
BARQ gait planner node (Stage 2D).

Subscribes /cmd_vel (geometry_msgs/Twist), generates trot foot trajectories, and streams 12
body-frame foot targets to /foot_targets at a fixed rate. The IK node turns those into joint
commands. At zero /cmd_vel the feet hold the neutral stance (no stepping).

Gait params (period, duty, step_height, stand_height, rate) are ROS parameters.
Geometry (hip offsets, coxa length) is read from barq_description/config/robot_params.yaml.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from barq_control.attitude import (AxisPID, HeadingHold, limit_to_reach, nominal_pitch,
                                   PhaseILC, roll_pitch_of, rotate_feet)
from barq_control.gait import foot_targets, LEGS
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import yaml


class GaitPlanner(Node):
    """Trot gait: /cmd_vel -> body-frame foot trajectories on /foot_targets."""

    def __init__(self):
        """Load geometry/params and start publishing foot targets at the configured rate."""
        super().__init__('gait_planner')
        legs = self._load_params()['legs']
        self.hip = {leg: legs['hip_offsets'][leg] for leg in LEGS}

        self.declare_parameter('period', 0.5)
        # duty >0.5 = stance overlap: calmer load transfer, less heave (D-019)
        self.declare_parameter('duty', 0.6)
        # Exact-model geometry; constraint: stand - step >= ~0.108 m (D-019 reach floor).
        # step 0.02 gives real swing clearance (foot sphere r=0.012 + contact/staircase margins).
        self.declare_parameter('step_height', 0.02)
        self.declare_parameter('stand_height', 0.13)
        # Stance trim (Aryaman): rear legs extended by this much -> nose-down pitch, load
        # shifts to the front feet, prevents backward body roll. ~5.3 deg at 0.02.
        self.declare_parameter('rear_raise', 0.02)
        self.declare_parameter('rate', 50.0)
        # Forward = body +X (Aryaman, watching the PHYSICS walk in Gazebo, 2026-06-11; this is
        # the arc direction approved in the RViz reversal session). +1 => cmd_vel +x drives the
        # body toward +X. Flip to -1 only if the frame convention is ever re-decided (Q-012).
        self.declare_parameter('forward_sign', 1.0)
        self.fwd = float(self.get_parameter('forward_sign').value)
        self.period = float(self.get_parameter('period').value)
        self.duty = float(self.get_parameter('duty').value)
        self.step_height = float(self.get_parameter('step_height').value)
        self.stand_height = float(self.get_parameter('stand_height').value)
        self.rear_raise = float(self.get_parameter('rear_raise').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)

        # Deadman: zero the command if /cmd_vel goes silent (Ctrl-C of a teleop publisher
        # must STOP the robot, not freeze the last velocity forever).
        self.declare_parameter('cmd_timeout', 1.0)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.last_cmd_time = None

        self.vx = self.vy = self.wz = 0.0
        self.t = 0.0

        # --- Experimental IMU feedback (exp/attitude-control). Both OFF = the open-loop gait.
        self.declare_parameter('attitude_ctrl', False)
        self.declare_parameter('att_kp', 0.6)
        self.declare_parameter('att_ki', 1.5)
        self.declare_parameter('att_kd', 0.04)
        self.declare_parameter('att_limit', 0.12)       # max corrective body rotation (rad)
        self.declare_parameter('att_i_limit', 0.08)     # integrator clamp (rad*s)
        # Low-pass on the attitude error (s). >0 regulates the MEAN attitude and leaves the
        # 2 Hz trot rock alone (feedback with servo lag amplified it in the first A/B).
        self.declare_parameter('att_lpf', 0.0)
        # Phase-indexed learning (ILC) for the periodic trot rock; 0 gain = off.
        self.declare_parameter('ilc_gain', 0.0)
        self.declare_parameter('ilc_lead', 3)
        self.declare_parameter('roll_ref', 0.0)
        self.declare_parameter('pitch_ref', float('nan'))   # nan -> designed trim pitch
        self.declare_parameter('heading_hold', False)
        self.declare_parameter('yaw_kp', 0.5)
        self.declare_parameter('yaw_ki', 1.0)
        self.declare_parameter('yaw_limit', 0.4)        # max yaw-rate correction (rad/s)
        gp = self.get_parameter
        self.att_on = bool(gp('attitude_ctrl').value)
        self.hdg_on = bool(gp('heading_hold').value)
        lim = float(gp('att_limit').value)
        il = float(gp('att_i_limit').value)
        kp, ki, kd = (float(gp(n).value) for n in ('att_kp', 'att_ki', 'att_kd'))
        self.pid_roll = AxisPID(kp, ki, kd, lim, il)
        self.pid_pitch = AxisPID(kp, ki, kd, lim, il)
        self.hdg = HeadingHold(float(gp('yaw_kp').value), float(gp('yaw_ki').value),
                               float(gp('yaw_limit').value), 0.5)
        self.att_lpf = float(gp('att_lpf').value)
        self.f_er = self.f_ep = 0.0
        g = float(gp('ilc_gain').value)
        lead = int(gp('ilc_lead').value)
        nb = max(8, int(round(self.period / self.dt)))
        self.ilc_on = g > 0.0
        self.ilc_r = PhaseILC(nb, g, lead)
        self.ilc_p = PhaseILC(nb, g, lead)
        self.roll_ref = float(gp('roll_ref').value)
        pr = float(gp('pitch_ref').value)
        self.pitch_ref = nominal_pitch(self.hip, self.rear_raise) if math.isnan(pr) else pr
        self.imu = None
        self.imu_time = None
        self.c_roll = self.c_pitch = 0.0
        if self.att_on or self.hdg_on:
            self.create_subscription(Imu, '/imu/data', self._on_imu, 20)
            # diag: roll, pitch, err_roll, err_pitch, c_roll, c_pitch, reach_scale, wz_eff
            self.diag = self.create_publisher(Float64MultiArray, '/gait/attitude', 10)
            self.get_logger().info(
                'IMU feedback: attitude=%s heading_hold=%s pitch_ref=%.4f rad'
                % (self.att_on, self.hdg_on, self.pitch_ref))

        self.pub = self.create_publisher(Float64MultiArray, '/foot_targets', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info('gait_planner up: trot on /cmd_vel -> /foot_targets')

    def _load_params(self):
        path = os.path.join(get_package_share_directory('barq_description'),
                            'config', 'robot_params.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    def _on_cmd(self, msg):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _on_imu(self, msg):
        self.imu = msg
        self.imu_time = self.get_clock().now()

    def _imu_fresh(self):
        if self.imu is None:
            return False
        return (self.get_clock().now() - self.imu_time).nanoseconds * 1e-9 < 0.2

    def _tick(self):
        self.t += self.dt
        if self.last_cmd_time is not None and (self.vx or self.vy or self.wz):
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.vx = self.vy = self.wz = 0.0
                self.get_logger().info('cmd_vel silent %.1fs - deadman stop' % age)
        # cmd_vel is robot-centric (+x = forward = body +X per forward_sign above); yaw about
        # Z is unchanged by the mapping.
        fresh = self._imu_fresh()
        wz = self.wz
        moving = abs(self.vx) + abs(self.vy) + abs(self.wz) > 1e-3
        if self.hdg_on and fresh and moving:
            wz = self.hdg.step(self.wz, self.imu.angular_velocity.z, self.dt)
        elif self.hdg_on:
            self.hdg.reset()
        ft = foot_targets(self.t, self.fwd * self.vx, self.fwd * self.vy, wz, self.hip,
                          period=self.period, duty=self.duty,
                          step_height=self.step_height, stand_height=self.stand_height,
                          rear_raise=self.rear_raise)
        if self.att_on:
            ft = self._attitude(ft, fresh, wz, moving)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in ft]
        self.pub.publish(msg)

    def _attitude(self, ft, fresh, wz, moving):
        if not fresh:                       # no IMU: decay the correction to zero, safely
            self.c_roll *= 0.9
            self.c_pitch *= 0.9
            self.pid_roll.reset()
            self.pid_pitch.reset()
            roll = pitch = er = ep = float('nan')
        else:
            q = self.imu.orientation
            roll, pitch = roll_pitch_of(q.x, q.y, q.z, q.w)
            er, ep = roll - self.roll_ref, pitch - self.pitch_ref
            w = self.imu.angular_velocity
            wx, wy = w.x, w.y
            raw_er, raw_ep = er, ep
            if self.att_lpf > 0.0:          # regulate the mean; D term off when filtering
                a = self.dt / (self.att_lpf + self.dt)
                self.f_er += a * (er - self.f_er)
                self.f_ep += a * (ep - self.f_ep)
                er, ep, wx, wy = self.f_er, self.f_ep, 0.0, 0.0
            freeze = self._last_scale < 1.0
            self.c_roll = self.pid_roll.step(er, wx, self.dt, freeze)
            self.c_pitch = self.pid_pitch.step(ep, wy, self.dt, freeze)
            if self.ilc_on:
                if moving:
                    ph = (self.t / self.period) % 1.0
                    # learn the PERIODIC part only: error minus its slow mean (the PID's job)
                    self.ilc_r.learn(ph, raw_er - self.f_er)
                    self.ilc_p.learn(ph, raw_ep - self.f_ep)
                    self.c_roll += self.ilc_r.output(ph)
                    self.c_pitch += self.ilc_p.output(ph)
                else:
                    self.ilc_r.decay()
                    self.ilc_p.decay()
        corrected = rotate_feet(ft, self.c_roll, self.c_pitch)
        ft, scale = limit_to_reach(ft, corrected, self.hip)
        self._last_scale = scale
        d = Float64MultiArray()
        d.data = [float(v) for v in (roll, pitch, er, ep, self.c_roll, self.c_pitch, scale, wz)]
        self.diag.publish(d)
        return ft

    _last_scale = 1.0


def main():
    """Spin the gait planner node."""
    rclpy.init()
    rclpy.spin(GaitPlanner())


if __name__ == '__main__':
    main()
