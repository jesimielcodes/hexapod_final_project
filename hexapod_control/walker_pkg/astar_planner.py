#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import heapq
from geometry_msgs.msg import PoseStamped

#  MOTION IDS 
STOP=0
FORWARD=1 
BACKWARD=3
TURN_LEFT=4 
TURN_RIGHT=6
CURVE_LEFT=5 
CURVE_RIGHT=2

FORWARD_THRESH = 0.20
CURVE_THRESH   = 0.70

#  NAV 
GOAL_THRESH=0.35
REPLAN_HZ=3.0

FRONT_CLEAR=0.55
BACKTRACK_LIMIT=8
REVERSE_CYCLES=30
TURN_CYCLES=40

STALL_CYCLES = 12         
STALL_PROGRESS = 0.15   

IDLE=0
REVERSING=1
TURNING=2


class LocalAStarNavigator(Node):

    def __init__(self):
        super().__init__('local_astar_navigator')

        self.goal_x = 7.25
        self.goal_y = 8.05

        self.rx=0.0
        self.ry=0.0
        self.ryaw=0.0

        self.next_wp=None
        self.goal_reached=False
        self.best_dist=float('inf')
        self.backtrack_count=0

        self.stall_cycle_count=0

        self.recovery_state=IDLE
        self.recovery_cycles=0
        self._turn_dir=1.0

        # ---- SLAM MAP ----
        self.map_data=None
        self.map_width=0
        self.map_height=0
        self.map_resolution=0.0
        self.map_origin=(0.0,0.0)

        self.create_subscription(OccupancyGrid,'/map',self._map_cb,10)
        self.create_subscription(Odometry,'/odom',self._odom_cb,20)
        self.create_subscription(LaserScan,'/scan',self._scan_cb,10)
        self.create_subscription(PoseStamped,'/goal_pose',self._goal_cb,10)

        self.cmd_pub=self.create_publisher(Int32,'/hexapod/motion_cmd',10)

        self.create_timer(1.0/REPLAN_HZ,self._replan)
        self.create_timer(2.0,self._status)

        self.get_logger().info('A* Navigator with SLAM ready')


    def _goal_cb(self, msg):
        """Updates the target when you click in Rviz."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False
        self.best_dist = float('inf')      
        self.stall_cycle_count = 0
        self.get_logger().info(f"New Goal Received: ({self.goal_x:.2f}, {self.goal_y:.2f})")


    #  MAP 
    def _map_cb(self,msg):
        self.get_logger().info(f"Map Received! Size: {msg.info.width}x{msg.info.height}")
        self.map_width=msg.info.width
        self.map_height=msg.info.height
        self.map_resolution=msg.info.resolution
        self.map_origin=(msg.info.origin.position.x,
                         msg.info.origin.position.y)

        data=msg.data
        self.map_data=[data[i:i+self.map_width]
                       for i in range(0,len(data),self.map_width)]

    def _world_to_map(self,wx,wy):
        mx=int((wx-self.map_origin[0])/self.map_resolution)
        my=int((wy-self.map_origin[1])/self.map_resolution)
        return mx,my

    def _map_to_world(self,mx,my):
        wx=self.map_origin[0]+mx*self.map_resolution
        wy=self.map_origin[1]+my*self.map_resolution
        return wx,wy

    def _valid(self,c):
        x,y=c
        return 0<=x<self.map_width and 0<=y<self.map_height

    def _free(self,gx,gy):
        if not self._valid((gx,gy)):
            return False
        val=self.map_data[gy][gx]
        return val<60

    #  ODOM 
    def _odom_cb(self,msg):
        self.rx=msg.pose.pose.position.x
        self.ry=msg.pose.pose.position.y
        q=msg.pose.pose.orientation
        self.ryaw=math.atan2(
            2*(q.w*q.z+q.x*q.y),
            1-2*(q.y*q.y+q.z*q.z)
        )
    
    def back_min(self, msg):
        return min(self._sector(msg, 145, 180), self._sector(msg, -180, -145))

    #  SCAN 
    def _scan_cb(self, msg):
        if self.goal_reached:
            self._pub(STOP)
            return

        front = self._sector(msg, -35,  35)
        left  = self._sector(msg,  35,  90)
        right = self._sector(msg, -90, -35)
        back  = self.back_min(msg)

        # Recovery takes priority always
        if self.recovery_state == REVERSING:
            self.recovery_cycles -= 1
            if self.recovery_cycles <= 0 or back < 0.25:
                self.recovery_state = TURNING
                self.recovery_cycles = TURN_CYCLES
            else:
                self._pub(BACKWARD)
            return

        if self.recovery_state == TURNING:
            self.recovery_cycles -= 1
            if self.recovery_cycles <= 0:
                self.recovery_state = IDLE
                self.backtrack_count = 0
                self.best_dist = float('inf')
                self.stall_cycle_count = 0  
            else:
                self._pub(TURN_LEFT if self._turn_dir > 0 else TURN_RIGHT)
            return

        # Obstacle ahead
        if front < FRONT_CLEAR:
            if self.next_wp:
                dx = self.next_wp[0] - self.rx
                dy = self.next_wp[1] - self.ry
                wp_err = self._norm(math.atan2(dy, dx) - self.ryaw)
                self._pub(TURN_LEFT if wp_err >= 0 else TURN_RIGHT)
            else:
                self._pub(TURN_LEFT if left >= right else TURN_RIGHT)
            self.backtrack_count += 1
            if self.backtrack_count >= BACKTRACK_LIMIT:
                self._start_recovery()
            return

        self.backtrack_count = 0

        # No map yet, wall follow to explore
        if self.map_data is None:
            self._wall_follow(front, left, right)
            return

        # Map exists, follow A* waypoint
        if self.next_wp:
            dx = self.next_wp[0] - self.rx
            dy = self.next_wp[1] - self.ry
            if math.hypot(dx, dy) > 0.15:
                err = self._norm(math.atan2(dy, dx) - self.ryaw)
                self._pub(self._select_motion(err))
                return
            else:
                self.next_wp = None

        self._wall_follow(front, left, right)

    def _wall_follow(self, front, left, right):
        """Hug the right wall to systematically explore the maze."""
        CLOSE_WALL = 0.20
        FAR_WALL   = 0.55

        if front < FRONT_CLEAR:
            self._pub(TURN_LEFT)
        elif right < CLOSE_WALL:
            self._pub(CURVE_LEFT)
        elif right > FAR_WALL:
            self._pub(CURVE_RIGHT)
        else:
            self._pub(FORWARD)

    #  MOTION 
    def _select_motion(self,err):
        a=abs(err)
        if a<FORWARD_THRESH:
            return FORWARD
        elif a<CURVE_THRESH:
            return CURVE_LEFT if err>0 else CURVE_RIGHT
        else:
            return TURN_LEFT if err>0 else TURN_RIGHT

    def _pub(self,m):
        msg=Int32()
        msg.data=m
        self.cmd_pub.publish(msg)

    #  RECOVERY 
    def _start_recovery(self):
        self._turn_dir*=-1
        self.recovery_state=REVERSING
        self.recovery_cycles=REVERSE_CYCLES
        self.next_wp=None
        self.backtrack_count=0          

    #  REPLAN 
    def _replan(self):
        if self.goal_reached:
            return

        if self.map_data is None:
            return

        dist = math.hypot(self.goal_x - self.rx, self.goal_y - self.ry)
        if dist < GOAL_THRESH:
            self.goal_reached = True
            self._pub(STOP)
            return

        if self.recovery_state != IDLE:
            return

        if dist < self.best_dist - STALL_PROGRESS:
            self.best_dist = dist
            self.stall_cycle_count = 0
        else:
            self.stall_cycle_count += 1
            if self.stall_cycle_count >= STALL_CYCLES:
                self.get_logger().warn(
                    f"Stall detected, no progress for {self.stall_cycle_count} cycles, "
                    f"best_dist={self.best_dist:.2f} current={dist:.2f}"
                )
                self._start_recovery()
                self.stall_cycle_count = 0
                return

        start = self._world_to_map(self.rx, self.ry)
        if not self._free(*start):
            start = self._nearest_free(start, r=30)
            if start is None:
                self.get_logger().warn("Robot trapped in obstacle, cannot plan")
                return

        goal = self._world_to_map(self.goal_x, self.goal_y)

        if not self._valid(start):
            return

        if not self._valid(goal):
            goal = self._clamp_goal_to_map(start, goal)
            if goal is None:
                self.get_logger().warn('Cannot clamp goal to map edge')
                return
            self.get_logger().info(f'Goal off-map, clamped to map cell {goal}')

        path = self._astar(start, goal)

        if not path or len(path) < 2:
            self.get_logger().warn("A* failed to find path to goal")
            return

        steps = int(0.5 / self.map_resolution)
        idx = min(max(1, steps), len(path) - 1)
        self.next_wp = self._map_to_world(*path[idx])


    def _clamp_goal_to_map(self, start, goal):
        """Walk from start toward goal, return the last valid map cell before going out of bounds."""
        sx, sy = start
        gx, gy = goal
        dx = gx - sx
        dy = gy - sy
        dist = math.hypot(dx, dy)
        if dist == 0:
            return None
        last_valid = None
        steps = int(dist) + 1
        for i in range(1, steps + 1):
            cx = int(sx + dx * i / dist)
            cy = int(sy + dy * i / dist)
            if self._valid((cx, cy)):
                last_valid = (cx, cy)
            else:
                break
        return last_valid
    

    #  A* 
    def _astar(self,start,goal):
        if not self._free(*goal):
            goal=self._nearest_free(goal, r=20)
            if goal is None:
                return None

        open_set=[(0,start)]
        came={}
        g={start:0}
        closed=set()

        def h(c):
            return math.hypot(c[0]-goal[0], c[1]-goal[1])

        while open_set:
            _,cur=heapq.heappop(open_set)
            if cur==goal:
                path=[]
                while cur in came:
                    path.append(cur)
                    cur=came[cur]
                path.append(start)
                return path[::-1]

            if cur in closed:
                continue
            closed.add(cur)

            for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),
                          (-1,-1),(-1,1),(1,-1),(1,1)]:

                nb=(cur[0]+dx, cur[1]+dy)
                if nb in closed or not self._free(*nb):
                    continue

                move=1.414 if dx and dy else 1.0
                heading=math.atan2(dy, dx)
                val = self.map_data[nb[1]][nb[0]]
                unknown_penalty = 2.0 if val == -1 else 0.0
                penalty=abs(self._norm(heading-self.ryaw))
                tg=g[cur]+move+0.2*penalty + unknown_penalty

                if tg<g.get(nb, float('inf')):
                    came[nb]=cur
                    g[nb]=tg
                    heapq.heappush(open_set, (tg+h(nb), nb))

        return None

    def _nearest_free(self,cell,r=8):
        gx,gy=cell
        for rad in range(1,r+1):
            for dx in range(-rad,rad+1):
                for dy in range(-rad,rad+1):
                    if abs(dx)==rad or abs(dy)==rad:
                        if self._free(gx+dx, gy+dy):
                            return (gx+dx, gy+dy)
        return None

    #  UTILS 
    def _sector(self,msg,a1,a2):
        lo,hi=math.radians(a1),math.radians(a2)
        vals=[r for i,r in enumerate(msg.ranges)
              if math.isfinite(r) and
              lo<=msg.angle_min+i*msg.angle_increment<=hi]
        return min(vals) if vals else 10.0

    def _norm(self,a):
        while a>math.pi:
            a-=2*math.pi
        while a<-math.pi:
            a+=2*math.pi
        return a

    def _status(self):
        self.get_logger().info(
            f'pos=({self.rx:.2f},{self.ry:.2f}) '
            f'dist_to_goal={math.hypot(self.goal_x-self.rx, self.goal_y-self.ry):.2f} '
            f'wp={self.next_wp} stall={self.stall_cycle_count}/{STALL_CYCLES}'
        )


def main():
    rclpy.init()
    node=LocalAStarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()