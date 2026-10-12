#!/usr/bin/env python3
"""向原泊车仿真发送三组输入，记录实际控制闭环；不注入车辆运动反馈。"""
import json
import math
import os
import time
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


def on_pose(msg):
    global pose
    p, q = msg.location.pose.pose.position, msg.location.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    # 仿真定位沿用原工程的 INS 航向约定，规划器和控制器也补偿 +pi/2。
    pose = (p.x-BASE_X, p.y-BASE_Y, math.remainder(yaw+math.pi/2, 2*math.pi))


def on_chassis(msg):
    global speed, phi, gear
    speed = msg.current_velocity * (-1 if msg.gear_location == 7 else 1)
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
    dt = [b.relative_time-a.relative_time for a, b in zip(msg.points, msg.points[1:])]
    active['equal_time_nodes'].append(min(dt)>0 and max(dt)-min(dt)<1e-5)
    print('TRAJECTORY', active['name'], key, 'points=', len(msg.points), flush=True)


def on_status(msg):
    if active is not None:
        active['end_hold_feedback'] += 1


def on_log(msg):
    if active is None:
        return
    if msg.name == '/trajectory_follower' and 'front wheels straightened; parking complete' in msg.msg:
        active['complete'] = True
    if msg.name != '/planner':
        return
    for token, field in [('LIOM accepted', 'accepted'), ('LIOM rejected', 'rejected'),
                         ('using fully checked search fallback', 'fallback'),
                         ('segment exhausted and stopped', 'replans')]:
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


def run_scene(name, goal, timeout, obstacle=None):
    global active
    active = None
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
        point.point.x, point.point.y = BASE_X+6.0, BASE_Y
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
                  trajectory_keys=[], equal_time_nodes=[], gear_changes=[], end_hold_feedback=0)
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
    samples, collisions, margin_violations = [], 0, 0
    while time.monotonic()-started < timeout and not rospy.is_shutdown():
        age = time.monotonic()-started
        x, y, yaw = pose
        samples.append([age, x, y, yaw, speed, phi, acc_command, lat_error, yaw_error, gear])
        if obstacle is not None:
            collisions += overlaps_obstacle(x, y, yaw, 0.0, obstacle)
            margin_violations += overlaps_obstacle(x, y, yaw, 0.4, obstacle)
        if active['reached'] and active['complete'] and abs(speed)<0.001:
            break
        time.sleep(0.1)
    time.sleep(0.3)
    distance = math.hypot(pose[0]-goal[0], pose[1]-goal[1])
    heading = abs(math.remainder(pose[2]-goal[2], 2*math.pi))
    rates = [abs(b[5]-a[5])/(b[0]-a[0]) for a,b in zip(samples,samples[1:])]
    # 观测为 10 Hz，底盘反馈为 100 Hz；容许两个反馈周期的采样时刻量化误差。
    rate_ok = all(abs(b[5]-a[5]) <= 0.23*(b[0]-a[0]+0.02)+0.001
                  for a,b in zip(samples,samples[1:]))
    active.update(elapsed_s=round(time.monotonic()-started,3), final_pose=list(pose),
                  goal_distance_m=distance, goal_heading_error_rad=heading, final_speed_mps=speed,
                  max_lat_error_m=max(abs(p[7]) for p in samples),
                  max_yaw_error_rad=max(abs(p[8]) for p in samples),
                  max_observed_steer_rate_radps=max(rates, default=0),
                  steer_rate_sample_check=rate_ok,
                  collisions=collisions, margin_violations=margin_violations)
    active['passed'] = (active['reached'] and active['complete'] and distance<=0.25 and heading<=0.0873 and
                        abs(speed)<=0.001 and abs(phi)<=0.03 and active['accepted']>0 and
                        collisions==0 and margin_violations==0 and rate_ok and
                        all(abs(v)<=0.05 for _,v in active['gear_changes']))
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
results = [run_scene('forward_straight', (8,0,0), 35),
           run_scene('parking_cusp', (4,4,math.pi/2), 90),
           run_scene('obstacle_detour', (12,0,0), 120, (6,0,1.05,2.05))]
print('SUMMARY', results, flush=True)
raise SystemExit(0 if all(results) else 1)
