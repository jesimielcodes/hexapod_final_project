

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

# ---------------- MOTION PRIMITIVES ----------------
#we share these motion IDs with the astar_planner for easier movement definition
STOP        = 0
FORWARD     = 1
BACKWARD    = 3
TURN_LEFT   = 4
TURN_RIGHT  = 6
CURVE_LEFT  = 5
CURVE_RIGHT = 2

# (linear_x, angular_z) for each primitive
MOTION_PARAMS = {
    STOP:        ( 0.00,  0.0),
    FORWARD:     ( 0.25,  0.0),
    BACKWARD:    (-0.12,  0.0),
    TURN_LEFT:   ( 0.00,  0.5),
    TURN_RIGHT:  ( 0.00, -0.5),
    CURVE_LEFT:  ( 0.20,  0.4),
    CURVE_RIGHT: ( 0.20, -0.4),
}

# ---------------- CONFIG ----------------

SWING_DURATION  = 0.05
STANCE_DURATION = 0.15
MIN_VELOCITY    = 0.01

# Our tripod gait splits the six legs into two alternating triangles:
#   Group A: legs 0, 2, 4
#   Group B: legs 1, 3, 5

SWING_SEQUENCE = [0, 2, 4, 1, 3, 5]

_GROUP_SIZE = len(SWING_SEQUENCE) // 2
LEG_PHASE_OFFSET = {}
for _seq_idx, _leg in enumerate(SWING_SEQUENCE):
    LEG_PHASE_OFFSET[_leg] = 0.0 if _seq_idx < _GROUP_SIZE else 0.5

JOINT_NAMES = []
for i in range(6):
    JOINT_NAMES += [
        f'leg{i}_coxa_joint',
        f'leg{i}_femur_joint',
        f'leg{i}_tibia_joint',
    ]

# Our calibrated stance — NOBODY SHOULD CHANGE THIS, already confirmed
STABLE_STANCE = {
    'leg0': (-0.024,  0.00, -0.228),
    'leg1': ( 0.203,  0.00, -0.220),
    'leg2': (-0.381,  0.153, -0.254),
    'leg3': (-0.095,  1.036,  0.220),
    'leg4': (-0.203,  0.00, -0.153),
    'leg5': (-0.789,  0.00, -0.254),
}

# ---------------- KINEMATICS ----------------

class HexapodKinematics:
    def __init__(self):
        self.coxa  = 0.05
        self.femur = 0.1
        self.tibia = 0.15

    def fk(self, coxa, femur, tibia):
        r = (
            self.coxa +
            self.femur * math.cos(femur) +
            self.tibia * math.cos(femur + tibia)
        )
        x = r * math.cos(coxa)
        y = r * math.sin(coxa)
        z = (
            self.femur * math.sin(femur) +
            self.tibia * math.sin(femur + tibia)
        )
        return x, y, z

    def ik(self, x, y, z):
        theta_coxa = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - self.coxa
        d = math.sqrt(r**2 + z**2)

        def clamp(v): return max(-1.0, min(1.0, v))

        cos_tibia   = clamp((d**2 - self.femur**2 - self.tibia**2) /
                            (2 * self.femur * self.tibia))
        theta_tibia = math.acos(cos_tibia)
        alpha       = math.atan2(z, r)
        cos_beta    = clamp((d**2 + self.femur**2 - self.tibia**2) /
                            (2 * d * self.femur))
        beta        = math.acos(cos_beta)
        theta_femur = alpha + beta

        return theta_coxa, theta_femur, theta_tibia


# ---------------- NODE ----------------

class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')

        self.linear_x  = 0.0
        self.angular_z = 0.0
        self.phase_start_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/hexapod_controller/joint_trajectory',
            10
        )

        self.create_subscription(
            Int32,
            '/hexapod/motion_cmd',
            self._motion_cb,
            10
        )

        self.timer = self.create_timer(0.05, self.gait_step)

        self.kin = HexapodKinematics()

        self.foot_home = {}
        for i in range(6):
            c, f, t = STABLE_STANCE[f'leg{i}']
            self.foot_home[i] = self.kin.fk(c, f, t)

        self.joint_offsets = {}
        for i in range(6):
            c0, f0, t0 = STABLE_STANCE[f'leg{i}']
            x, y, z    = self.foot_home[i]
            ci, fi, ti = self.kin.ik(x, y, z)
            self.joint_offsets[i] = (c0 - ci, f0 - fi, t0 - ti)

        self.get_logger().info('Gait planner ready — /hexapod/motion_cmd')
        self.get_logger().info(
            'IDs: 0=STOP 1=FWD 3=BACK 4=TURN_L 6=TURN_R 5=CURVE_L 2=CURVE_R'
        )
        self.get_logger().info(
            f'Tripod groups — A(phase 0.0): {[l for l,p in LEG_PHASE_OFFSET.items() if p==0.0]}'
            f' B(phase 0.5): {[l for l,p in LEG_PHASE_OFFSET.items() if p==0.5]}'
        )

    # ---------------- MOTION CALLBACK ----------------

    def _motion_cb(self, msg):
        motion_id = int(msg.data)
        if motion_id not in MOTION_PARAMS:
            self.get_logger().warn(f'Unknown motion ID: {motion_id}')
            return

        lx, az = MOTION_PARAMS[motion_id]

        was_moving = (abs(self.linear_x)  > MIN_VELOCITY or
                      abs(self.angular_z) > MIN_VELOCITY)
        self.linear_x  = lx
        self.angular_z = az
        is_moving = (abs(self.linear_x)  > MIN_VELOCITY or
                     abs(self.angular_z) > MIN_VELOCITY)

        if is_moving and not was_moving:
            self.phase_start_time = self.get_clock().now()

    # ---------------- JOINT LIMITS ----------------

    def _clamp_coxa (self, a): return max(-0.8, min( 0.8, a))
    def _clamp_femur(self, a): return max(-1.5, min( 0.8, a))
    def _clamp_tibia(self, a): return max(-2.0, min( 1.5, a))

    # ---------------- GAIT STEP ----------------

    def gait_step(self):
        is_moving = (abs(self.linear_x)  > MIN_VELOCITY or
                     abs(self.angular_z) > MIN_VELOCITY)

        now     = self.get_clock().now()
        elapsed = (now - self.phase_start_time).nanoseconds / 1e9

        cycle_time     = SWING_DURATION + STANCE_DURATION
        swing_fraction = SWING_DURATION / cycle_time
        global_phase   = (elapsed % cycle_time) / cycle_time

        positions = []

        for i in range(6):
            base_x, base_y, base_z = self.foot_home[i]

            phase_offset = LEG_PHASE_OFFSET[i]

            # always use the IK path, even when stopped.
            # When not moving, stride=0 so the foot stays at home position,
            # and we get the exact same joint values as the moving branch at home.
            # This eliminates the jerk on start/stop transitions.
            if is_moving:
                stride   = self.linear_x * 0.15
                turn     = self.angular_z * 0.04
                # Group A legs push forward on positive stride and group B legs push back
                turn_dir = 1.0 if phase_offset == 0.0 else -1.0
                stride  += turn * turn_dir
            else:
                stride = 0.0

            local_phase = (global_phase - phase_offset) % 1.0

            if local_phase < swing_fraction:
                p = local_phase / swing_fraction
                x = base_x + (-stride/2 + stride * p)
                z = base_z + 0.08 * math.sin(math.pi * p)
            else:
                p = (local_phase - swing_fraction) / (1 - swing_fraction)
                x = base_x + (stride/2 - stride * p)
                z = base_z

            y = base_y

            c, f, t = self.kin.ik(x, y, z)

            oc, of_, ot = self.joint_offsets[i]
            c += oc
            f += of_
            t += ot

            c = self._clamp_coxa (c)
            f = self._clamp_femur(f)
            t = self._clamp_tibia(t)

            positions.extend([c, f, t])

        traj             = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        point                 = JointTrajectoryPoint()
        point.positions       = positions
        point.time_from_start = Duration(sec=0, nanosec=40_000_000)

        traj.points = [point]
        self.publisher.publish(traj)


# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = GaitPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()