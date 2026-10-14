#!/usr/bin/env python3
"""隔离 ROS 仿真验证泊车跟踪；只发送场景输入，不注入车辆运动反馈。"""
import json
import math
import os
import time
import sys
import rospy
from driver_msgs.msg import ChassisReport, DriveCmd, MotionStartCmd
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from localization_msgs.msg import Localization
from nav_msgs.msg import OccupancyGrid
from planning_msgs.msg import TrajectoryPointArray, OpenSpaceExecutionStatus
from rosgraph_msgs.msg import Log
from std_msgs.msg import Empty, Float32, Float64

BASE_X, BASE_Y = 400000.0, 4400000.0
pose = (0.0, 0.0, 0.0)
speed = phi = lat_error = yaw_error = acc_command = 0.0
gear = 0
active = None
EXPECT_FALLBACK = False


def on_pose(msg):
    global pose
    p, q = msg.location.pose.pose.position, msg.location.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    # 仿真定位沿用原工程的 INS 航向约定，规划器和控制器也补偿 +pi/2。
    pose = (p.x-BASE_X, p.y-BASE_Y, math.remainder(yaw+math.pi/2, 2*math.pi))


def on_chassis(msg):
    global speed, phi, gear
    speed = msg.current_velocity * (-1 if msg.gear_location == 7 else 1)
    if active is not None:
        active['max_forward_speed_mps'] = max(active['max_forward_speed_mps'], speed)
        active['max_reverse_speed_mps'] = max(active['max_reverse_speed_mps'], -speed)
    phi = math.radians(msg.steering_wheel_angle)/18.5
    if active is not None and msg.gear_location != gear:
        active['gear_changes'].append([msg.gear_location, speed])
    gear = msg.gear_location


def on_drive(msg):
    global acc_command
    acc_command = msg.acc_target


def on_lat(msg):
    global lat_error
    lat_error = msg.data


def on_yaw(msg):
    global yaw_error
    yaw_error = msg.data


def on_goal(_):
    if active is not None:
        active['reached'] = True


def on_trajectory(msg):
    if active is None or len(msg.points) < 2 or not any(abs(p.v)>0.001 for p in msg.points):
        return
    end = msg.points[-1]
    key = (round(end.x-BASE_X, 4), round(end.y-BASE_Y, 4), msg.is_forward_shift)
    if key in active['trajectory_keys']:
        return
    active['trajectory_keys'].append(key)
    active['pending_end'] = [end.x-BASE_X, end.y-BASE_Y, end.theta]
    active['segments'].append(dict(end_pose=active['pending_end'], forward=msg.is_forward_shift,
                                   points=len(msg.points), duration_s=end.relative_time))

    dt = [b.relative_time-a.relative_time for a, b in zip(msg.points, msg.points[1:])]
    active['equal_time_nodes'].append(min(dt)>0 and max(dt)-min(dt)<1e-5)
    print('TRAJECTORY', active['name'], key, 'points=', len(msg.points), flush=True)
    zeros = [dict(index=i, s=p.s, x=p.x-BASE_X, y=p.y-BASE_Y,
                  v=p.v, phi=math.atan((1 if msg.is_forward_shift else -1)*2.85*p.kappa),
                  t=p.relative_time)
             for i,p in enumerate(msg.points[1:-1], 1) if abs(p.v)<=0.001]
    if zeros:
        print('MIDSTOP', json.dumps(zeros), flush=True)


def on_status(msg):
    if active is not None:
        active['end_hold_feedback'] += 1


def on_log(msg):
    if active is None:
        return
    if msg.name == '/trajectory_follower' and 'stationary steering' in msg.msg:
        print('STEERING_PHASE', json.dumps(dict(message=msg.msg, pose=list(pose),
                                              speed=speed, phi=phi)), flush=True)
    if msg.name == '/trajectory_follower' and 'activate segment after steering preparation' in msg.msg:
        active['executing_end'] = active.get('pending_end')
        print('EXECUTION_START', json.dumps(dict(pose=list(pose), speed=speed,
                                                expected_end=active.get('executing_end'))), flush=True)
    if msg.name == '/trajectory_follower' and 'state EXECUTING ->' in msg.msg:
        end = active.get('executing_end')
        if end is not None:
            item = dict(transition=msg.msg, actual_pose=list(pose), speed=speed, expected_end=end,
                        distance_m=math.hypot(pose[0]-end[0], pose[1]-end[1]),
                        heading_error_rad=abs(math.remainder(pose[2]-end[2], 2*math.pi)))
            active['segment_finishes'].append(item)
            print('EXECUTION_END', json.dumps(item), flush=True)
    if msg.name == '/trajectory_follower' and 'front wheels straightened; parking complete' in msg.msg:
        active['complete'] = True
    if msg.name != '/planner':
        return
    if 'state ' in msg.msg and '-> PLANNING' in msg.msg:
        active['planning_requests'] += 1
        active['replans'] = max(0, active['planning_requests']-1)
    if '-> EMERGENCY_BRAKING' in msg.msg:
        active['emergency_braking'] += 1
    for token, field in [('LIOM accepted', 'accepted'), ('LIOM rejected', 'rejected'),
                         ('using fully checked search fallback', 'fallback')]:
        if token in msg.msg:
            active[field] += 1
    if '[open_space]' in msg.msg and ('state ' in msg.msg or 'LIOM ' in msg.msg):
        print('PLANNER', active['name'], msg.msg, flush=True)


def overlaps_obstacle(x, y, yaw, margin, obstacle):
    # 用与规划参数一致的 5x3 m 车身做独立 SAT 复核，参考点为后轴中心。
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy, hx, hy = obstacle
    dx, dy = x+1.15*c-ox, y+1.15*s-oy
    for ax, ay in [(1, 0), (0, 1), (c, s), (-s, c)]:
        body_radius = (2.5+margin)*abs(ax*c+ay*s)+(1.5+margin)*abs(-ax*s+ay*c)
        if abs(ax*dx+ay*dy) > body_radius+hx*abs(ax)+hy*abs(ay):
            return False
    return True



def obstacle_clearance(x, y, yaw, obstacle):
    """实际车身矩形与占据格外包矩形的最小欧氏距离；相交返回零。"""
    if overlaps_obstacle(x, y, yaw, 0.0, obstacle):
        return 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    body = [(x+c*u-s*v, y+s*u+c*v) for u, v in
            [(-1.35,-1.5),(3.65,-1.5),(3.65,1.5),(-1.35,1.5)]]
    ox, oy, hx, hy = obstacle
    box = [(ox-hx,oy-hy),(ox+hx,oy-hy),(ox+hx,oy+hy),(ox-hx,oy+hy)]
    def point_segment(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        ratio = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-a[0]-ratio*dx, p[1]-a[1]-ratio*dy)
    return min(point_segment(p, polygon[i], polygon[(i+1)%4])
               for vertices, polygon in [(body,box),(box,body)]
               for p in vertices for i in range(4))

def run_scene(name, goal, timeout, obstacle=None):
    global active, lat_error, yaw_error
    active = None
    lat_error = yaw_error = 0.0
    initial = PoseWithCovarianceStamped()
    initial.header.frame_id = 'map'
    initial.header.stamp = rospy.Time.now()
    initial.pose.pose.position.x, initial.pose.pose.position.y = BASE_X, BASE_Y
    initial.pose.pose.orientation.w = 1.0
    initial_pub.publish(initial)
    time.sleep(1.0)
    if obstacle is not None:
        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = rospy.Time.now()
        point.point.x, point.point.y = BASE_X+obstacle[0], BASE_Y+obstacle[1]
        obstacle_pub.publish(point)
        time.sleep(0.5)
        grid = rospy.wait_for_message('/free_space_map', OccupancyGrid, timeout=5)
        cells = [i for i, value in enumerate(grid.data) if value == 100]
        resolution, width = grid.info.resolution, grid.info.width
        xmin = grid.info.origin.position.x+min(i % width for i in cells)*resolution
        xmax = grid.info.origin.position.x+(max(i % width for i in cells)+1)*resolution
        ymin = grid.info.origin.position.y+min(i // width for i in cells)*resolution
        ymax = grid.info.origin.position.y+(max(i // width for i in cells)+1)*resolution
        obstacle = ((xmin+xmax)/2, (ymin+ymax)/2, (xmax-xmin)/2, (ymax-ymin)/2)
    active = dict(name=name, reached=False, complete=False, accepted=0, rejected=0, fallback=0, replans=0,
                  planning_requests=0,
                  trajectory_keys=[], equal_time_nodes=[], gear_changes=[], end_hold_feedback=0,
                  max_forward_speed_mps=0., max_reverse_speed_mps=0., emergency_braking=0,
                  segments=[], segment_finishes=[], obstacle=obstacle)
    target = PoseStamped()
    target.header.frame_id = 'map'
    target.header.stamp = rospy.Time.now()
    target.pose.position.x, target.pose.position.y = BASE_X+goal[0], BASE_Y+goal[1]
    target.pose.orientation.z, target.pose.orientation.w = math.sin(goal[2]/2), math.cos(goal[2]/2)
    goal_pub.publish(target)
    started = time.monotonic()
    while not active['trajectory_keys'] and time.monotonic()-started < 10:
        time.sleep(0.05)
    motion = MotionStartCmd()
    motion.header.stamp = rospy.Time.now()
    motion.motion_start = 1
    motion_pub.publish(motion)
    samples, collisions, margin_violations, margin_02_violations = [], 0, 0, 0
    minimum_clearance = float("inf")
    closest_sample = None
    while time.monotonic()-started < timeout and not rospy.is_shutdown():
        age = time.monotonic()-started
        x, y, yaw = pose
        samples.append([age, x, y, yaw, speed, phi, acc_command, lat_error, yaw_error, gear])
        if obstacle is not None:
            clearance = obstacle_clearance(x, y, yaw, obstacle)
            if clearance < minimum_clearance:
                minimum_clearance, closest_sample = clearance, samples[-1]
            collisions += overlaps_obstacle(x, y, yaw, 0.0, obstacle)
            margin_violations += overlaps_obstacle(x, y, yaw, 0.4, obstacle)
            margin_02_violations += overlaps_obstacle(x, y, yaw, 0.2, obstacle)
        if active['reached'] and active['complete'] and abs(speed)<0.001:
            break
        if active['emergency_braking'] >= 3 or active['replans'] >= 4 or collisions:
            break
        time.sleep(0.05)
    time.sleep(0.3)
    distance = math.hypot(pose[0]-goal[0], pose[1]-goal[1])
    heading = abs(math.remainder(pose[2]-goal[2], 2*math.pi))
    rates = [abs(b[5]-a[5])/(b[0]-a[0]) for a,b in zip(samples,samples[1:])]
    # 检查采样为 20 Hz，底盘反馈为 100 Hz；容许两个反馈周期的时刻量化误差。
    rate_ok = all(abs(b[5]-a[5]) <= 0.23*(b[0]-a[0]+0.02)+0.001
                  for a,b in zip(samples,samples[1:]))
    active.update(elapsed_s=round(time.monotonic()-started,3), final_pose=list(pose),
                  goal_distance_m=distance, goal_heading_error_rad=heading, final_speed_mps=speed,
                  max_lat_error_m=max(abs(p[7]) for p in samples),
                  max_yaw_error_rad=max(abs(p[8]) for p in samples),
                  max_observed_steer_rate_radps=max(rates, default=0),
                  steer_rate_sample_check=rate_ok,
                  collisions=collisions, margin_violations=margin_violations,
                  margin_02_violations=margin_02_violations,
                  max_abs_acc_command_mps2=max(abs(p[6]) for p in samples),
                  minimum_body_clearance_m=minimum_clearance, closest_sample=closest_sample)
    optimization_mode_ok = (active['accepted']==0 and active['fallback']>0) if EXPECT_FALLBACK else (
                            active['accepted']>0 and active['fallback']==0)
    # 允许用户接受的轻微速度超限，保留实际峰值记录，不放宽停车和碰撞标准。
    active['passed'] = (active['reached'] and active['complete'] and distance<=0.25 and heading<=0.0873 and
                        abs(speed)<=0.001 and abs(phi)<=0.03 and optimization_mode_ok and
                        collisions==0 and margin_02_violations==0 and rate_ok and
                        all(abs(v)<=0.05 for _,v in active['gear_changes']) and active['replans']==0 and
                        active['max_forward_speed_mps']<=6/3.6+0.01 and
                        active['max_reverse_speed_mps']<=4/3.6+0.01)
    print('RESULT', json.dumps(active, ensure_ascii=False), flush=True)
    print('SAMPLES', name, json.dumps(samples), flush=True)
    motion.motion_start = 0
    motion_pub.publish(motion)
    return active['passed']


# 防止忘记切换 ROS 环境后，把起点和运动开始指令发到默认或实车 master。
if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11483':
    raise SystemExit('请使用隔离仿真 ROS_MASTER_URI=http://127.0.0.1:11483')
rospy.init_node('liom_closedloop_check')
rospy.Subscriber('/odomData', Localization, on_pose)
rospy.Subscriber('/chassis', ChassisReport, on_chassis)
rospy.Subscriber('/auto_chassis_drive_cmd', DriveCmd, on_drive)
rospy.Subscriber('/lat_error', Float32, on_lat)
rospy.Subscriber('/control/open_space_mpc/yaw_error', Float64, on_yaw)
rospy.Subscriber('/trajectory', TrajectoryPointArray, on_trajectory)
rospy.Subscriber('/open_space_goal_reached', Empty, on_goal)
rospy.Subscriber('/open_space_execution_status', OpenSpaceExecutionStatus, on_status)
rospy.Subscriber('/rosout', Log, on_log)
initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
motion_pub = rospy.Publisher('/chassis_motion_start_cmd', MotionStartCmd, queue_size=1)
obstacle_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
started = time.monotonic()
while initial_pub.get_num_connections()<2 or motion_pub.get_num_connections()<2:
    if time.monotonic()-started>20:
        raise RuntimeError('原 planner/simulate/trajectory_follower 未准备好')
    time.sleep(0.1)
# 等待 launch 完成参数加载后再判断模式，避免 ROS master 刚启动时读到默认预算。
EXPECT_FALLBACK = rospy.get_param('/planner/liom_time_budget_ms', 1000.0) < 1.0
print('EXPECTED', 'search_fallback' if EXPECT_FALLBACK else 'LIOM', flush=True)
print('PARAMETERS', json.dumps({key: rospy.get_param('/trajectory_follower/'+key) for key in
      ['open_space_lookahead_distance', 'mpc_weight_heading_error',
       'mpc_weight_terminal_heading_error', 'open_space_max_acceleration',
       'open_space_max_deceleration', 'open_space_max_jerk', 'open_space_forward_speed_kp',
       'open_space_reverse_speed_kp', 'input_delay', 'vehicle_model_steer_tau']}), flush=True)
mode = sys.argv[1] if len(sys.argv)>1 else 'all'
results = []
if mode == 'short':
    results.extend([run_scene('forward_short', (1.5,0,0), 35),
                    run_scene('reverse_short', (-1.5,0,0), 35)])
if mode == 'curves':
    results.extend([run_scene('parking_cusp', (4,4,math.pi/2), 90),
                    run_scene('parking_cusp_mirror', (4,-4,-math.pi/2), 90)])
if mode == 'all':
    results.extend([run_scene('forward_straight', (8,0,0), 35),
                    run_scene('reverse_straight', (-12,0,0), 45),
                    run_scene('parking_cusp', (4,4,math.pi/2), 90)])
if mode in ['all', 'detour']:
    name = 'timeout_search_fallback_detour' if EXPECT_FALLBACK else 'optimized_detour_baseline'
    results.append(run_scene(name, (12,0,0), 120, (6,0,1.05,2.05)))
if not results:
    raise SystemExit('场景选项为 all、short、curves 或 detour')
print('SUMMARY', results, flush=True)
raise SystemExit(0 if all(results) else 1)
