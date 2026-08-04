#!/usr/bin/env python3
"""Guarded low-speed longitudinal calibration for the CARLA Cybertruck.

The script owns /vehicle_control_cmd while it runs, resets the vehicle to a
lane-centre pose between trials, and measures the pedal model used by Lite:

    drive acceleration = throttle_gain * throttle - rolling_resistance
    braking deceleration = brake_gain * brake + rolling_resistance
"""

import argparse
import csv
import math
import os
import statistics
import threading
import time

import rosgraph
import rospy
from std_msgs.msg import Float64MultiArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def linear_fit(xs, ys):
    """Return slope, intercept and R squared for y = slope*x + intercept."""
    if len(xs) < 2 or len(xs) != len(ys):
        raise ValueError("linear fit needs at least two paired samples")
    x_mean = statistics.mean(xs)
    y_mean = statistics.mean(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator <= 1.0e-12:
        raise ValueError("linear fit input has no spread")
    slope = sum(
        (x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)
    ) / denominator
    intercept = y_mean - slope * x_mean
    total = sum((y - y_mean) ** 2 for y in ys)
    residual = sum(
        (y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys)
    )
    r_squared = 1.0 - residual / total if total > 1.0e-12 else 1.0
    return slope, intercept, r_squared


def slope_over_time(rows):
    times = [row["elapsed"] for row in rows]
    speeds = [row["speed"] for row in rows]
    return linear_fit(times, speeds)


class LongitudinalCalibration:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.state = None
        self.collision = None
        self.raw_rows = []
        self.summaries = []
        self.creep_rows = []
        self.creep_stop_rows = []
        self.reverse_map_rows = []
        self.reverse_stop_rows = []
        self.trial_start_xy = None
        self.actor = None
        self.reset_transform = None

        self.control_pub = rospy.Publisher(
            args.control_topic, Float64MultiArray, queue_size=1)
        rospy.Subscriber(
            args.state_topic, Float64MultiArray,
            self.state_callback, queue_size=20)
        rospy.Subscriber(
            args.collision_topic, Float64MultiArray,
            self.collision_callback, queue_size=10)

        self.prepare_carla_reset()

    def prepare_carla_reset(self):
        import carla

        client = carla.Client(self.args.carla_host, self.args.carla_port)
        client.set_timeout(4.0)
        world = client.get_world()
        actors = list(world.get_actors().filter("vehicle.tesla.cybertruck"))
        if len(actors) != 1:
            raise RuntimeError(
                "expected exactly one Cybertruck, found {}".format(len(actors)))
        self.actor = actors[0]
        transform = self.actor.get_transform()
        if self.args.snap_reset_to_road:
            waypoint = world.get_map().get_waypoint(
                transform.location, project_to_road=True,
                lane_type=carla.LaneType.Driving)
            if waypoint is None:
                raise RuntimeError("could not find a driving-lane reset waypoint")
            transform = waypoint.transform
            transform.location.z += self.args.reset_z_offset
        self.reset_transform = carla.Transform(
            carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z),
            carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw,
                roll=transform.rotation.roll))
        rospy.loginfo(
            "calibration reset pose: x=%.3f y=%.3f z=%.3f yaw=%.3f deg",
            transform.location.x, transform.location.y,
            transform.location.z, transform.rotation.yaw)

    def state_callback(self, message):
        data = message.data
        for offset in range(0, len(data), 18):
            if offset + 18 > len(data):
                break
            if int(round(data[offset])) != self.args.vehicle_id:
                continue
            yaw = float(data[offset + 7])
            vx = float(data[offset + 9])
            vy = float(data[offset + 10])
            sample = {
                "stamp": time.monotonic(),
                # zheda-2.py publishes CARLA y before CARLA x.
                "x": float(data[offset + 2]),
                "y": float(data[offset + 1]),
                "yaw": yaw,
                "vx": vx,
                "vy": vy,
                "speed": math.hypot(vx, vy),
                "longitudinal_speed": vx * math.cos(yaw) + vy * math.sin(yaw),
                "actual_throttle": float(data[offset + 12]),
                "actual_steer": float(data[offset + 13]),
                "actual_brake": float(data[offset + 14]),
                "actual_reverse": int(round(data[offset + 15])),
                "gear": int(round(data[offset + 17])),
            }
            with self.lock:
                self.state = sample
            return

    def collision_callback(self, message):
        if len(message.data) < 3:
            return
        if int(round(message.data[0])) != self.args.vehicle_id:
            return
        with self.lock:
            self.collision = {
                "active": float(message.data[1]),
                "count": float(message.data[2]),
            }

    def snapshot(self):
        with self.lock:
            state = dict(self.state) if self.state else None
            collision = dict(self.collision) if self.collision else None
        return state, collision

    def publish_control(
            self, throttle=0.0, brake=0.0, reverse=False,
            hand_brake=False):
        message = Float64MultiArray()
        message.data = [
            float(self.args.vehicle_id),
            clamp(throttle, 0.0, 1.0),
            0.0,
            clamp(brake, 0.0, 1.0),
            1.0 if reverse else 0.0,
            1.0 if hand_brake else 0.0,
        ]
        self.control_pub.publish(message)

    def assert_exclusive_control(self):
        master = rosgraph.Master(rospy.get_name())
        publishers, _, _ = master.getSystemState()
        own_name = rospy.get_name()
        for topic, nodes in publishers:
            if topic != self.args.control_topic:
                continue
            others = [node for node in nodes if node != own_name]
            if others:
                raise RuntimeError(
                    "{} has other publishers: {}".format(
                        self.args.control_topic, ", ".join(others)))

    def wait_for_state(self):
        deadline = time.monotonic() + self.args.input_timeout
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, _ = self.snapshot()
            self.publish_control(brake=1.0, hand_brake=True)
            if state and time.monotonic() - state["stamp"] < 0.5:
                return
            rate.sleep()
        raise RuntimeError("timed out waiting for vehicle state on /chatter")

    def safety_check(self, state, collision):
        if state is None or time.monotonic() - state["stamp"] > 0.5:
            return "vehicle-state timeout"
        if collision and collision["active"] >= 0.5:
            return "collision reported"
        if state["speed"] > self.args.max_speed:
            return "maximum speed exceeded ({:.3f} m/s)".format(state["speed"])
        if self.trial_start_xy is not None:
            displacement = math.hypot(
                state["x"] - self.trial_start_xy[0],
                state["y"] - self.trial_start_xy[1])
            if displacement > self.args.max_displacement:
                return "maximum trial displacement exceeded"
        return None

    def hold_stop(self, seconds=1.0, reverse=False):
        deadline = time.monotonic() + seconds
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, _ = self.snapshot()
            hand_brake = state is not None and state["speed"] < 0.025
            self.publish_control(
                brake=1.0, reverse=reverse, hand_brake=hand_brake)
            rate.sleep()

    def reset_actor(self, reverse=False):
        import carla

        self.hold_stop(0.5, reverse=reverse)
        self.actor.apply_control(carla.VehicleControl(
            throttle=0.0, steer=0.0, brake=1.0,
            hand_brake=True, reverse=reverse))
        self.actor.set_target_velocity(carla.Vector3D())
        self.actor.set_target_angular_velocity(carla.Vector3D())
        self.actor.set_transform(self.reset_transform)
        self.trial_start_xy = (
            self.reset_transform.location.x, self.reset_transform.location.y)
        self.hold_stop(self.args.reset_settle_seconds, reverse=reverse)

    def record_row(
            self, trial, phase, started, command_throttle,
            command_brake, command_reverse, state):
        actor_acceleration = self.actor.get_acceleration()
        actor_forward = self.actor.get_transform().get_forward_vector()
        forward_acceleration = (
            actor_acceleration.x * actor_forward.x +
            actor_acceleration.y * actor_forward.y +
            actor_acceleration.z * actor_forward.z)
        speed_acceleration = (
            -forward_acceleration if command_reverse else forward_acceleration)
        row = {
            "wall_time": time.time(),
            "trial": trial,
            "phase": phase,
            "elapsed": time.monotonic() - started,
            "direction": "reverse" if command_reverse else "forward",
            "command_throttle": command_throttle,
            "command_brake": command_brake,
            "actual_throttle": state["actual_throttle"],
            "actual_brake": state["actual_brake"],
            "actual_reverse": state["actual_reverse"],
            "gear": state["gear"],
            "x": state["x"],
            "y": state["y"],
            "yaw": state["yaw"],
            "vx": state["vx"],
            "vy": state["vy"],
            "longitudinal_speed": state["longitudinal_speed"],
            "speed": state["speed"],
            "carla_speed_acceleration": speed_acceleration,
        }
        self.raw_rows.append(row)
        return row

    def prepare_gear(self, reverse):
        started = time.monotonic()
        deadline = started + self.args.gear_prepare_seconds
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.publish_control(brake=1.0, reverse=reverse)
            rate.sleep()

    def accelerate_to_speed(self, trial, reverse, target_speed):
        self.prepare_gear(reverse)
        started = time.monotonic()
        deadline = started + self.args.accelerate_timeout
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            error = target_speed - state["speed"]
            if error <= 0.0:
                return
            base_throttle = (
                self.args.reverse_approach_throttle
                if reverse else self.args.approach_throttle)
            max_throttle = (
                self.args.reverse_max_approach_throttle
                if reverse else self.args.max_approach_throttle)
            throttle = clamp(
                base_throttle + self.args.approach_kp * error,
                base_throttle, max_throttle)
            self.publish_control(throttle=throttle, reverse=reverse)
            self.record_row(
                trial, "approach", started, throttle, 0.0, reverse, state)
            rate.sleep()
        raise RuntimeError(
            "{}: could not reach {:.2f} m/s within {:.1f} s".format(
                trial, target_speed, self.args.accelerate_timeout))

    def run_coast_trial(self, reverse):
        direction = "reverse" if reverse else "forward"
        trial = "coast_{}".format(direction)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=reverse)
        self.accelerate_to_speed(
            trial, reverse, self.args.coast_start_speed)
        started = time.monotonic()
        deadline = started + self.args.coast_seconds
        rows = []
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(reverse=reverse)
            row = self.record_row(
                trial, "sample", started, 0.0, 0.0, reverse, state)
            if row["elapsed"] >= self.args.transient_seconds:
                rows.append(row)
            if state["speed"] <= self.args.coast_end_speed:
                break
            rate.sleep()
        if len(rows) < self.args.min_samples:
            raise RuntimeError("{}: insufficient coast samples".format(trial))
        slope, intercept, r_squared = slope_over_time(rows)
        summary = {
            "trial": trial,
            "kind": "coast",
            "direction": direction,
            "pedal": 0.0,
            "acceleration": slope,
            "speed_start": rows[0]["speed"],
            "speed_end": rows[-1]["speed"],
            "duration": rows[-1]["elapsed"] - rows[0]["elapsed"],
            "sample_count": len(rows),
            "time_fit_r_squared": r_squared,
            "status": "ok" if slope < -0.005 else "weak_deceleration",
        }
        self.summaries.append(summary)
        rospy.loginfo(
            "%s: coast acceleration=%+.4f m/s^2 R2=%.3f",
            trial, slope, r_squared)

    def run_throttle_trial(self, reverse, pedal):
        direction = "reverse" if reverse else "forward"
        trial = "throttle_{}_{:.3f}".format(direction, pedal)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=reverse)
        self.prepare_gear(reverse)
        started = time.monotonic()
        deadline = started + self.args.throttle_seconds
        rows = []
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(throttle=pedal, reverse=reverse)
            row = self.record_row(
                trial, "sample", started, pedal, 0.0, reverse, state)
            if (row["elapsed"] >= self.args.transient_seconds and
                    state["speed"] <= self.args.fit_max_speed):
                rows.append(row)
            if state["speed"] >= self.args.trial_stop_speed:
                break
            rate.sleep()
        moving_rows = [row for row in rows if row["speed"] >= 0.08]
        fit_rows = moving_rows if len(moving_rows) >= self.args.min_samples else rows
        if len(fit_rows) < self.args.min_samples:
            status = "insufficient_samples"
            acceleration = float("nan")
            r_squared = float("nan")
        else:
            acceleration, _, r_squared = slope_over_time(fit_rows)
            speed_span = fit_rows[-1]["speed"] - fit_rows[0]["speed"]
            status = "ok" if speed_span >= 0.08 and acceleration > 0.01 else "no_motion"
        summary = {
            "trial": trial,
            "kind": "throttle",
            "direction": direction,
            "pedal": pedal,
            "acceleration": acceleration,
            "speed_start": fit_rows[0]["speed"] if fit_rows else float("nan"),
            "speed_end": fit_rows[-1]["speed"] if fit_rows else float("nan"),
            "duration": (
                fit_rows[-1]["elapsed"] - fit_rows[0]["elapsed"]
                if len(fit_rows) >= 2 else 0.0),
            "sample_count": len(fit_rows),
            "time_fit_r_squared": r_squared,
            "status": status,
        }
        self.summaries.append(summary)
        rospy.loginfo(
            "%s: acceleration=%+.4f m/s^2 R2=%.3f status=%s",
            trial, acceleration, r_squared, status)

    def run_brake_trial(self, reverse, pedal):
        direction = "reverse" if reverse else "forward"
        trial = "brake_{}_{:.3f}".format(direction, pedal)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=reverse)
        self.accelerate_to_speed(
            trial, reverse, self.args.brake_start_speed)
        started = time.monotonic()
        deadline = started + self.args.brake_seconds
        rows = []
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(brake=pedal, reverse=reverse)
            row = self.record_row(
                trial, "sample", started, 0.0, pedal, reverse, state)
            if (row["elapsed"] >= self.args.brake_transient_seconds and
                    state["speed"] >= self.args.brake_fit_min_speed):
                rows.append(row)
            if (row["elapsed"] > self.args.brake_transient_seconds and
                    state["speed"] <= self.args.brake_stop_speed):
                break
            rate.sleep()
        if len(rows) < self.args.min_brake_samples:
            status = "insufficient_samples"
            acceleration = float("nan")
            r_squared = float("nan")
        else:
            acceleration, _, r_squared = slope_over_time(rows)
            status = "ok" if acceleration < -0.02 else "weak_braking"
        summary = {
            "trial": trial,
            "kind": "brake",
            "direction": direction,
            "pedal": pedal,
            "acceleration": acceleration,
            "speed_start": rows[0]["speed"] if rows else float("nan"),
            "speed_end": rows[-1]["speed"] if rows else float("nan"),
            "duration": (
                rows[-1]["elapsed"] - rows[0]["elapsed"]
                if len(rows) >= 2 else 0.0),
            "sample_count": len(rows),
            "time_fit_r_squared": r_squared,
            "status": status,
        }
        self.summaries.append(summary)
        rospy.loginfo(
            "%s: acceleration=%+.4f m/s^2 deceleration=%.4f R2=%.3f status=%s",
            trial, acceleration, -acceleration, r_squared, status)

    def run_hold_trial(self, reverse, pedal):
        direction = "reverse" if reverse else "forward"
        trial = "hold_{}_{:.3f}".format(direction, pedal)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=reverse)
        self.prepare_gear(reverse)
        started = time.monotonic()
        deadline = started + self.args.hold_seconds
        rows = []
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(brake=pedal, reverse=reverse)
            rows.append(self.record_row(
                trial, "sample", started, 0.0, pedal, reverse, state))
            rate.sleep()
        speeds = [row["speed"] for row in rows]
        displacement = math.hypot(
            rows[-1]["x"] - rows[0]["x"],
            rows[-1]["y"] - rows[0]["y"])
        max_speed = max(speeds)
        status = (
            "holds" if max_speed <= self.args.hold_max_speed and
            displacement <= self.args.hold_max_displacement else "moves")
        self.summaries.append({
            "trial": trial,
            "kind": "hold",
            "direction": direction,
            "pedal": pedal,
            "acceleration": float("nan"),
            "speed_start": speeds[0],
            "speed_end": speeds[-1],
            "duration": rows[-1]["elapsed"] - rows[0]["elapsed"],
            "sample_count": len(rows),
            "time_fit_r_squared": float("nan"),
            "status": status,
            "max_speed": max_speed,
            "displacement": displacement,
        })
        rospy.loginfo(
            "%s: max_speed=%.4f m/s displacement=%.4f m status=%s",
            trial, max_speed, displacement, status)

    def run_creep_trial(self, reverse, pedal, repeat_index):
        """Observe a fixed small pedal from standstill for an extended period."""
        direction = "reverse" if reverse else "forward"
        trial = "creep_{}_t{:.3f}_r{}".format(
            direction, pedal, repeat_index)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=reverse)
        self.prepare_gear(reverse)

        started = time.monotonic()
        deadline = started + self.args.creep_seconds
        thresholds = (0.05, 0.10, 0.20, 0.30, 0.50)
        threshold_times = {threshold: float("nan") for threshold in thresholds}
        rows = []
        speed_limit_reached = False
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(throttle=pedal, reverse=reverse)
            row = self.record_row(
                trial, "creep", started, pedal, 0.0, reverse, state)
            rows.append(row)
            for threshold in thresholds:
                if (math.isnan(threshold_times[threshold]) and
                        state["speed"] >= threshold):
                    threshold_times[threshold] = row["elapsed"]
            if state["speed"] >= self.args.creep_speed_limit:
                speed_limit_reached = True
                break
            rate.sleep()

        if not rows:
            raise RuntimeError("{}: no samples recorded".format(trial))
        last_elapsed = rows[-1]["elapsed"]
        stable_rows = [
            row for row in rows
            if row["elapsed"] >=
            max(0.0, last_elapsed - self.args.creep_stable_window)]
        stable_speed = statistics.mean(row["speed"] for row in stable_rows)
        stable_speed_stddev = (
            statistics.stdev(row["speed"] for row in stable_rows)
            if len(stable_rows) >= 2 else 0.0)
        stable_acceleration = statistics.mean(
            row["carla_speed_acceleration"] for row in stable_rows)
        stable_slope = (
            slope_over_time(stable_rows)[0]
            if len(stable_rows) >= 2 else float("nan"))
        peak_speed = max(row["speed"] for row in rows)
        peak_acceleration = max(
            row["carla_speed_acceleration"] for row in rows)
        displacement = math.hypot(
            rows[-1]["x"] - rows[0]["x"],
            rows[-1]["y"] - rows[0]["y"])
        if speed_limit_reached:
            status = "speed_limit_reached"
        elif peak_speed < self.args.creep_motion_speed:
            status = "no_motion"
        elif (stable_speed_stddev <= self.args.creep_stable_speed_stddev and
              abs(stable_slope) <= self.args.creep_stable_slope):
            status = "stable"
        else:
            status = "moving_unsettled"
        result = {
            "trial": trial,
            "repeat": repeat_index,
            "direction": direction,
            "pedal": pedal,
            "duration": last_elapsed,
            "time_to_0_05": threshold_times[0.05],
            "time_to_0_10": threshold_times[0.10],
            "time_to_0_20": threshold_times[0.20],
            "time_to_0_30": threshold_times[0.30],
            "time_to_0_50": threshold_times[0.50],
            "peak_speed": peak_speed,
            "stable_speed": stable_speed,
            "stable_speed_stddev": stable_speed_stddev,
            "stable_acceleration": stable_acceleration,
            "stable_speed_slope": stable_slope,
            "peak_acceleration": peak_acceleration,
            "displacement": displacement,
            "sample_count": len(rows),
            "status": status,
        }
        self.creep_rows.append(result)
        rospy.loginfo(
            "%s: peak_speed=%.4f stable_speed=%.4f peak_accel=%+.4f "
            "distance=%.3f status=%s",
            trial, peak_speed, stable_speed, peak_acceleration,
            displacement, status)

    def run_creep_stop_trial(self, reverse, pedal, repeat_index):
        """Precondition at fixed pedal, then measure coast-to-stop distance."""
        direction = "reverse" if reverse else "forward"
        trial = "creep_stop_{}_t{:.3f}_r{}".format(
            direction, pedal, repeat_index)
        if self.args.creep_stop_brake > 0.0:
            trial += "_b{:.3f}".format(self.args.creep_stop_brake)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=reverse)
        self.prepare_gear(reverse)
        precondition_started = time.monotonic()
        precondition_deadline = (
            precondition_started + self.args.creep_stop_precondition_seconds)
        rate = rospy.Rate(self.args.publish_rate)
        while (not rospy.is_shutdown() and
               time.monotonic() < precondition_deadline):
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(throttle=pedal, reverse=reverse)
            self.record_row(
                trial, "creep_stop_precondition", precondition_started,
                pedal, 0.0, reverse, state)
            rate.sleep()

        initial_state, _ = self.snapshot()
        initial_speed = initial_state["speed"]
        initial_x = initial_state["x"]
        initial_y = initial_state["y"]
        coast_started = time.monotonic()
        coast_deadline = coast_started + self.args.creep_stop_timeout
        coast_rows = []
        stopped_since = None
        while not rospy.is_shutdown() and time.monotonic() < coast_deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(
                brake=self.args.creep_stop_brake, reverse=reverse)
            coast_rows.append(self.record_row(
                trial, "creep_stop_coast", coast_started,
                0.0, self.args.creep_stop_brake, reverse, state))
            if state["speed"] <= self.args.creep_stop_speed_tolerance:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                elif (time.monotonic() - stopped_since >=
                      self.args.creep_stop_stable_seconds):
                    break
            else:
                stopped_since = None
            rate.sleep()
        final_state, _ = self.snapshot()
        peak_post_cut_speed = max(
            [initial_speed] + [row["speed"] for row in coast_rows])
        stop_distance = math.hypot(
            final_state["x"] - initial_x, final_state["y"] - initial_y)
        stop_time = time.monotonic() - coast_started
        stopped = (
            final_state["speed"] <= self.args.creep_stop_speed_tolerance and
            stopped_since is not None)
        result = {
            "trial": trial,
            "repeat": repeat_index,
            "direction": direction,
            "precondition_pedal": pedal,
            "precondition_seconds": self.args.creep_stop_precondition_seconds,
            "stop_brake": self.args.creep_stop_brake,
            "initial_speed": initial_speed,
            "peak_post_cut_speed": peak_post_cut_speed,
            "final_speed": final_state["speed"],
            "stop_distance": stop_distance,
            "stop_time": stop_time,
            "sample_count": len(coast_rows),
            "status": "ok" if stopped else "timeout",
        }
        self.creep_stop_rows.append(result)
        rospy.loginfo(
            "%s: initial_speed=%.4f peak_after_cut=%.4f stop_distance=%.4f "
            "stop_time=%.3f status=%s",
            trial, initial_speed, peak_post_cut_speed, stop_distance,
            stop_time, result["status"])

    def run_reverse_map_trial(self, target_speed, pedal, repeat_index):
        """Measure local reverse acceleration at one speed/pedal operating point."""
        trial = "reverse_map_v{:.2f}_t{:.3f}_r{}".format(
            target_speed, pedal, repeat_index)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=True)
        self.accelerate_to_speed(trial, True, target_speed)

        started = time.monotonic()
        deadline = started + self.args.map_sample_seconds
        rows = []
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(throttle=pedal, reverse=True)
            row = self.record_row(
                trial, "reverse_map", started, pedal, 0.0, True, state)
            if (row["elapsed"] >= self.args.map_transient_seconds and
                    state["speed"] >= self.args.map_fit_min_speed):
                rows.append(row)
            if (row["elapsed"] >= self.args.map_transient_seconds and
                    (state["speed"] <= self.args.map_stop_speed or
                     state["speed"] >= self.args.map_max_speed)):
                break
            rate.sleep()

        if len(rows) >= self.args.map_min_samples:
            acceleration, _, r_squared = slope_over_time(rows)
            status = "ok"
        else:
            acceleration = float("nan")
            r_squared = float("nan")
            status = "insufficient_samples"
        mean_speed = statistics.mean(
            row["speed"] for row in rows) if rows else float("nan")
        speed_start = rows[0]["speed"] if rows else float("nan")
        speed_end = rows[-1]["speed"] if rows else float("nan")
        duration = (
            rows[-1]["elapsed"] - rows[0]["elapsed"]
            if len(rows) >= 2 else 0.0)
        result = {
            "trial": trial,
            "repeat": repeat_index,
            "target_speed": target_speed,
            "mean_speed": mean_speed,
            "pedal": pedal,
            "acceleration": acceleration,
            "speed_start": speed_start,
            "speed_end": speed_end,
            "duration": duration,
            "sample_count": len(rows),
            "time_fit_r_squared": r_squared,
            "status": status,
        }
        self.reverse_map_rows.append(result)
        rospy.loginfo(
            "%s: mean_speed=%.4f acceleration=%+.4f m/s^2 R2=%.3f status=%s",
            trial, mean_speed, acceleration, r_squared, status)

    def append_reverse_sweep_bin(
            self, trial, target_speed, pedal, repeat_index, rows):
        window_rows = [
            row for row in rows
            if abs(row["speed"] - target_speed) <= self.args.map_sweep_window]
        if len(window_rows) >= self.args.map_sweep_min_samples:
            acceleration = statistics.mean(
                row["carla_speed_acceleration"] for row in window_rows)
            acceleration_stddev = (
                statistics.stdev(
                    row["carla_speed_acceleration"] for row in window_rows)
                if len(window_rows) >= 2 else 0.0)
            if len(window_rows) >= 2:
                _, _, r_squared = slope_over_time(window_rows)
            else:
                r_squared = float("nan")
            status = "ok"
        else:
            acceleration = float("nan")
            acceleration_stddev = float("nan")
            r_squared = float("nan")
            status = "not_crossed"
        mean_speed = (
            statistics.mean(row["speed"] for row in window_rows)
            if window_rows else float("nan"))
        result = {
            "trial": trial,
            "repeat": repeat_index,
            "target_speed": target_speed,
            "mean_speed": mean_speed,
            "pedal": pedal,
            "acceleration": acceleration,
            "acceleration_stddev": acceleration_stddev,
            "speed_start": (
                window_rows[0]["speed"] if window_rows else float("nan")),
            "speed_end": (
                window_rows[-1]["speed"] if window_rows else float("nan")),
            "duration": (
                window_rows[-1]["elapsed"] - window_rows[0]["elapsed"]
                if len(window_rows) >= 2 else 0.0),
            "sample_count": len(window_rows),
            "time_fit_r_squared": r_squared,
            "status": status,
        }
        self.reverse_map_rows.append(result)
        rospy.loginfo(
            "%s speed bin %.2f: mean=%.4f acceleration=%+.4f m/s^2 "
            "R2=%.3f samples=%d status=%s",
            trial, target_speed, mean_speed, acceleration, r_squared,
            len(window_rows), status)

    def run_reverse_sweep_trial(self, pedal, repeat_index):
        """Measure speed-dependent acceleration while crossing all speed bins."""
        descending = pedal <= self.args.map_sweep_decelerating_max_pedal
        direction = "down" if descending else "up"
        trial = "reverse_sweep_{}_t{:.3f}_r{}".format(
            direction, pedal, repeat_index)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=True)
        if descending:
            self.accelerate_to_speed(
                trial, True, self.args.map_sweep_start_speed)
        else:
            self.prepare_gear(True)

        started = time.monotonic()
        deadline = started + self.args.map_sweep_timeout
        rows = []
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(throttle=pedal, reverse=True)
            rows.append(self.record_row(
                trial, "reverse_sweep", started, pedal, 0.0, True, state))
            if descending:
                if (time.monotonic() - started >=
                        self.args.map_sweep_transient_seconds and
                        state["speed"] <
                        min(self.args.map_speed_steps) -
                        self.args.map_sweep_window):
                    break
            elif (state["speed"] >
                  max(self.args.map_speed_steps) +
                  self.args.map_sweep_window):
                break
            rate.sleep()

        analysis_rows = rows
        if descending and rows:
            # Discard the short surge after cutting the approach throttle and
            # only fit the monotonically descending side of the sweep.
            peak_index = max(range(len(rows)), key=lambda index: rows[index]["speed"])
            analysis_rows = rows[peak_index:]
        analysis_rows = [
            row for row in analysis_rows
            if row["elapsed"] >= self.args.map_sweep_transient_seconds]
        for target_speed in self.args.map_speed_steps:
            self.append_reverse_sweep_bin(
                trial, target_speed, pedal, repeat_index, analysis_rows)

    def run_reverse_stop_trial(self, target_speed, repeat_index):
        """Measure coast-to-stop distance after cutting reverse throttle."""
        trial = "reverse_stop_v{:.2f}_r{}".format(target_speed, repeat_index)
        rospy.loginfo("running %s", trial)
        self.reset_actor(reverse=True)
        self.accelerate_to_speed(trial, True, target_speed)
        initial_state, _ = self.snapshot()
        initial_speed = initial_state["speed"]
        initial_x = initial_state["x"]
        initial_y = initial_state["y"]

        started = time.monotonic()
        deadline = started + self.args.stop_distance_timeout
        rows = []
        stopped_since = None
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, collision = self.snapshot()
            reason = self.safety_check(state, collision)
            if reason:
                raise RuntimeError("{}: {}".format(trial, reason))
            self.publish_control(reverse=True)
            rows.append(self.record_row(
                trial, "reverse_stop", started, 0.0, 0.0, True, state))
            if state["speed"] <= self.args.stop_distance_speed_tolerance:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                elif (time.monotonic() - stopped_since >=
                        self.args.stop_distance_stable_seconds):
                    break
            else:
                stopped_since = None
            rate.sleep()

        final_state, _ = self.snapshot()
        stopped = (
            final_state["speed"] <= self.args.stop_distance_speed_tolerance and
            stopped_since is not None)
        stop_distance = math.hypot(
            final_state["x"] - initial_x, final_state["y"] - initial_y)
        stop_time = time.monotonic() - started
        average_deceleration = (
            (initial_speed - final_state["speed"]) / stop_time
            if stop_time > 1.0e-6 else float("nan"))
        result = {
            "trial": trial,
            "repeat": repeat_index,
            "target_speed": target_speed,
            "initial_speed": initial_speed,
            "final_speed": final_state["speed"],
            "stop_distance": stop_distance,
            "stop_time": stop_time,
            "average_deceleration": average_deceleration,
            "sample_count": len(rows),
            "status": "ok" if stopped else "timeout",
        }
        self.reverse_stop_rows.append(result)
        rospy.loginfo(
            "%s: initial_speed=%.4f stop_distance=%.4f m stop_time=%.3f s "
            "status=%s", trial, initial_speed, stop_distance, stop_time,
            result["status"])

    def fit_parameters(self):
        coasts = [
            row for row in self.summaries
            if row["kind"] == "coast" and row["status"] == "ok"]
        rolling = (
            statistics.mean(-row["acceleration"] for row in coasts)
            if coasts else float("nan"))

        result = {
            "rolling_resistance": rolling,
            "throttle_gain": float("nan"),
            "brake_gain": float("nan"),
            "throttle_fit_r_squared": float("nan"),
            "brake_fit_r_squared": float("nan"),
        }
        throttle_rows = [
            row for row in self.summaries
            if row["kind"] == "throttle" and row["status"] == "ok"]
        brake_rows = [
            row for row in self.summaries
            if row["kind"] == "brake" and row["status"] == "ok"]

        if len(throttle_rows) >= 2:
            gain, intercept, r_squared = linear_fit(
                [row["pedal"] for row in throttle_rows],
                [row["acceleration"] for row in throttle_rows])
            result.update({
                "throttle_gain": gain,
                "throttle_intercept": intercept,
                "throttle_fit_rolling_resistance": -intercept,
                "throttle_fit_r_squared": r_squared,
            })
        if len(brake_rows) >= 2:
            gain, intercept, r_squared = linear_fit(
                [row["pedal"] for row in brake_rows],
                [-row["acceleration"] for row in brake_rows])
            result.update({
                "brake_gain": gain,
                "brake_intercept": intercept,
                "brake_fit_r_squared": r_squared,
            })

        for direction in ("forward", "reverse"):
            for kind in ("throttle", "brake"):
                rows = [
                    row for row in self.summaries
                    if row["kind"] == kind and
                    row["direction"] == direction and
                    row["status"] == "ok"]
                if len(rows) < 2:
                    continue
                values = [
                    row["acceleration"] if kind == "throttle"
                    else -row["acceleration"] for row in rows]
                gain, intercept, r_squared = linear_fit(
                    [row["pedal"] for row in rows], values)
                result["{}_{}_gain".format(direction, kind)] = gain
                result["{}_{}_intercept".format(direction, kind)] = intercept
                result["{}_{}_r_squared".format(direction, kind)] = r_squared

        holding = [
            row["pedal"] for row in self.summaries
            if row["kind"] == "hold" and row["status"] == "holds"]
        result["minimum_tested_hold_brake"] = (
            min(holding) if holding else float("nan"))
        return result

    def write_results(self, fit_result=None, error=None):
        output_dir = os.path.dirname(os.path.abspath(self.args.output_prefix))
        os.makedirs(output_dir, exist_ok=True)
        raw_path = self.args.output_prefix + "_raw.csv"
        summary_path = self.args.output_prefix + "_summary.csv"
        report_path = self.args.output_prefix + "_report.md"
        reverse_map_path = self.args.output_prefix + "_reverse_map.csv"
        reverse_stop_path = self.args.output_prefix + "_reverse_stop_distance.csv"
        creep_path = self.args.output_prefix + "_creep.csv"
        creep_stop_path = self.args.output_prefix + "_creep_stop.csv"
        raw_fields = [
            "wall_time", "trial", "phase", "elapsed", "direction",
            "command_throttle", "command_brake", "actual_throttle",
            "actual_brake", "actual_reverse", "gear", "x", "y", "yaw",
            "vx", "vy", "longitudinal_speed", "speed",
            "carla_speed_acceleration"]
        with open(raw_path, "w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=raw_fields)
            writer.writeheader()
            writer.writerows(self.raw_rows)
        if self.summaries:
            summary_fields = [
                "trial", "kind", "direction", "pedal", "acceleration",
                "speed_start", "speed_end", "duration", "sample_count",
                "time_fit_r_squared", "status", "max_speed", "displacement"]
            with open(summary_path, "w", newline="") as output:
                writer = csv.DictWriter(
                    output, fieldnames=summary_fields, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(self.summaries)
        if self.creep_rows:
            creep_fields = [
                "trial", "repeat", "direction", "pedal", "duration",
                "time_to_0_05", "time_to_0_10", "time_to_0_20",
                "time_to_0_30", "time_to_0_50", "peak_speed",
                "stable_speed", "stable_speed_stddev",
                "stable_acceleration", "stable_speed_slope",
                "peak_acceleration", "displacement", "sample_count",
                "status"]
            with open(creep_path, "w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=creep_fields)
                writer.writeheader()
                writer.writerows(self.creep_rows)
        if self.creep_stop_rows:
            creep_stop_fields = [
                "trial", "repeat", "direction", "precondition_pedal",
                "precondition_seconds", "stop_brake", "initial_speed",
                "peak_post_cut_speed", "final_speed", "stop_distance",
                "stop_time", "sample_count", "status"]
            with open(creep_stop_path, "w", newline="") as output:
                writer = csv.DictWriter(
                    output, fieldnames=creep_stop_fields)
                writer.writeheader()
                writer.writerows(self.creep_stop_rows)
        if self.reverse_map_rows:
            map_fields = [
                "trial", "repeat", "target_speed", "mean_speed", "pedal",
                "acceleration", "acceleration_stddev", "speed_start",
                "speed_end", "duration", "sample_count",
                "time_fit_r_squared", "status"]
            with open(reverse_map_path, "w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=map_fields)
                writer.writeheader()
                writer.writerows(self.reverse_map_rows)
        if self.reverse_stop_rows:
            stop_fields = [
                "trial", "repeat", "target_speed", "initial_speed",
                "final_speed", "stop_distance", "stop_time",
                "average_deceleration", "sample_count", "status"]
            with open(reverse_stop_path, "w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=stop_fields)
                writer.writeheader()
                writer.writerows(self.reverse_stop_rows)
        with open(report_path, "w") as output:
            output.write("# CARLA Cybertruck longitudinal calibration\n\n")
            output.write("Model: `a_drive = G_t * throttle - r`; ")
            output.write("`deceleration = G_b * brake + r`.\n\n")
            if error:
                output.write("Calibration stopped: `{}`\n\n".format(error))
            if fit_result:
                output.write("## Fitted parameters\n\n")
                for key in sorted(fit_result):
                    value = fit_result[key]
                    output.write("- `{}`: `{:.8g}`\n".format(key, value))
                output.write("\n")
            output.write("## Trial summaries\n\n")
            output.write("| Trial | acceleration (m/s^2) | R2 | status |\n")
            output.write("|---|---:|---:|---|\n")
            for row in self.summaries:
                output.write(
                    "| {} | {:.5g} | {:.4g} | {} |\n".format(
                        row["trial"], row["acceleration"],
                        row["time_fit_r_squared"], row["status"]))
            if self.creep_rows:
                output.write("\n## Long-duration creep trials\n\n")
                output.write(
                    "| direction | pedal | t@0.1 | t@0.3 | peak speed | "
                    "stable speed | peak acceleration | status |\n")
                output.write("|---|---:|---:|---:|---:|---:|---:|---|\n")
                for row in self.creep_rows:
                    output.write(
                        "| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | "
                        "{:.3f} | {:+.3f} | {} |\n".format(
                            row["direction"], row["pedal"],
                            row["time_to_0_10"], row["time_to_0_30"],
                            row["peak_speed"], row["stable_speed"],
                            row["peak_acceleration"], row["status"]))
            if self.creep_stop_rows:
                output.write("\n## Pedal-conditioned coast stopping distance\n\n")
                output.write(
                    "| direction | pedal | stop brake | initial speed | "
                    "peak after cut | stop distance | stop time | status |\n")
                output.write("|---|---:|---:|---:|---:|---:|---:|---|\n")
                for row in self.creep_stop_rows:
                    output.write(
                        "| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} | {:.4f} | "
                        "{:.3f} | {} |\n".format(
                            row["direction"], row["precondition_pedal"],
                            row["stop_brake"],
                            row["initial_speed"], row["peak_post_cut_speed"],
                            row["stop_distance"], row["stop_time"],
                            row["status"]))
            if self.reverse_map_rows:
                output.write("\n## Reverse low-speed map\n\n")
                output.write(
                    "| target speed | mean speed | throttle | acceleration | R2 | status |\n")
                output.write("|---:|---:|---:|---:|---:|---|\n")
                for row in self.reverse_map_rows:
                    output.write(
                        "| {:.3f} | {:.3f} | {:.3f} | {:+.4f} | {:.3f} | {} |\n".format(
                            row["target_speed"], row["mean_speed"], row["pedal"],
                            row["acceleration"], row["time_fit_r_squared"],
                            row["status"]))
            if self.reverse_stop_rows:
                output.write("\n## Reverse coast stopping distance\n\n")
                output.write(
                    "| target speed | initial speed | stop distance | stop time | status |\n")
                output.write("|---:|---:|---:|---:|---|\n")
                for row in self.reverse_stop_rows:
                    output.write(
                        "| {:.3f} | {:.3f} | {:.4f} | {:.3f} | {} |\n".format(
                            row["target_speed"], row["initial_speed"],
                            row["stop_distance"], row["stop_time"],
                            row["status"]))
        rospy.loginfo("raw data: %s", raw_path)
        rospy.loginfo("summary: %s", summary_path)
        rospy.loginfo("report: %s", report_path)
        if self.reverse_map_rows:
            rospy.loginfo("reverse map: %s", reverse_map_path)
        if self.reverse_stop_rows:
            rospy.loginfo("reverse stop distance: %s", reverse_stop_path)
        if self.creep_rows:
            rospy.loginfo("creep trials: %s", creep_path)
        if self.creep_stop_rows:
            rospy.loginfo("pedal-conditioned stop trials: %s", creep_stop_path)

    def run(self):
        self.wait_for_state()
        self.assert_exclusive_control()
        self.reset_actor()
        error = None
        fit_result = None
        try:
            directions = [value == "reverse" for value in self.args.directions]
            for reverse in directions:
                if "creep" not in self.args.stages:
                    continue
                steps = (
                    self.args.reverse_creep_throttle_steps
                    if reverse else self.args.forward_creep_throttle_steps)
                for pedal in steps:
                    for repeat_index in range(1, self.args.repeats + 1):
                        self.run_creep_trial(reverse, pedal, repeat_index)
            for reverse in directions:
                if "creep_stop" not in self.args.stages:
                    continue
                steps = (
                    self.args.reverse_creep_stop_throttle_steps
                    if reverse else self.args.forward_creep_stop_throttle_steps)
                for pedal in steps:
                    for repeat_index in range(1, self.args.repeats + 1):
                        self.run_creep_stop_trial(reverse, pedal, repeat_index)
            for reverse in directions:
                if "coast" in self.args.stages:
                    for _ in range(self.args.repeats):
                        self.run_coast_trial(reverse)
            for reverse in directions:
                if "throttle" not in self.args.stages:
                    continue
                steps = (
                    self.args.reverse_throttle_steps
                    if reverse else self.args.throttle_steps)
                for pedal in steps:
                    for _ in range(self.args.repeats):
                        self.run_throttle_trial(reverse, pedal)
            for reverse in directions:
                if "brake" not in self.args.stages:
                    continue
                for pedal in self.args.brake_steps:
                    for _ in range(self.args.repeats):
                        self.run_brake_trial(reverse, pedal)
            for reverse in directions:
                if "hold" not in self.args.stages:
                    continue
                for pedal in self.args.hold_steps:
                    for _ in range(self.args.repeats):
                        self.run_hold_trial(reverse, pedal)
            if "reverse_map" in self.args.stages:
                for target_speed in self.args.map_speed_steps:
                    for pedal in self.args.map_throttle_steps:
                        for repeat_index in range(1, self.args.repeats + 1):
                            self.run_reverse_map_trial(
                                target_speed, pedal, repeat_index)
            if "reverse_sweep" in self.args.stages:
                for pedal in self.args.map_throttle_steps:
                    for repeat_index in range(1, self.args.repeats + 1):
                        self.run_reverse_sweep_trial(pedal, repeat_index)
            if "reverse_stop" in self.args.stages:
                for target_speed in self.args.stop_distance_speed_steps:
                    for repeat_index in range(1, self.args.repeats + 1):
                        self.run_reverse_stop_trial(target_speed, repeat_index)
            fit_result = self.fit_parameters()
            rospy.loginfo("fitted parameters: %s", fit_result)
            return fit_result
        except Exception as caught:
            error = caught
            raise
        finally:
            try:
                self.reset_actor()
                self.hold_stop(2.0)
            finally:
                self.write_results(fit_result=fit_result, error=error)


def comma_separated_floats(value):
    result = [float(item) for item in value.split(",") if item.strip()]
    if not result:
        raise argparse.ArgumentTypeError("at least one value is required")
    return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-id", type=int, default=0)
    parser.add_argument(
        "--stages", type=lambda value: [item.strip() for item in value.split(",")],
        default=["coast", "throttle", "brake", "hold"],
        help=(
            "comma-separated subset of creep,creep_stop,coast,throttle,brake,hold,"
            "reverse_map,reverse_sweep,reverse_stop"))
    parser.add_argument(
        "--directions", type=lambda value: [item.strip() for item in value.split(",")],
        default=["forward", "reverse"],
        help="comma-separated subset of forward,reverse")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument(
        "--throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.04,0.06,0.08,0.10,0.12,0.15"))
    parser.add_argument(
        "--reverse-throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.12,0.16,0.20,0.24,0.28,0.32"))
    parser.add_argument(
        "--brake-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.03,0.05,0.08,0.12,0.18"))
    parser.add_argument(
        "--hold-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.02,0.05,0.10,0.15"))
    parser.add_argument(
        "--forward-creep-throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.02,0.03,0.04,0.05,0.06,0.08,0.10,0.12"))
    parser.add_argument(
        "--reverse-creep-throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats(
            "0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.10,0.12,0.14,0.16"))
    parser.add_argument("--creep-seconds", type=float, default=15.0)
    parser.add_argument("--creep-speed-limit", type=float, default=0.85)
    parser.add_argument("--creep-motion-speed", type=float, default=0.02)
    parser.add_argument("--creep-stable-window", type=float, default=3.0)
    parser.add_argument(
        "--creep-stable-speed-stddev", type=float, default=0.02)
    parser.add_argument("--creep-stable-slope", type=float, default=0.02)
    parser.add_argument(
        "--forward-creep-stop-throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.02,0.03,0.04"))
    parser.add_argument(
        "--reverse-creep-stop-throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.07,0.10,0.12,0.14,0.145,0.15,0.16"))
    parser.add_argument(
        "--creep-stop-precondition-seconds", type=float, default=15.0)
    parser.add_argument("--creep-stop-timeout", type=float, default=6.0)
    parser.add_argument("--creep-stop-brake", type=float, default=0.0)
    parser.add_argument(
        "--creep-stop-speed-tolerance", type=float, default=0.01)
    parser.add_argument(
        "--creep-stop-stable-seconds", type=float, default=0.30)
    parser.add_argument(
        "--map-speed-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.20,0.30,0.40,0.50,0.60"))
    parser.add_argument(
        "--map-throttle-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.00,0.08,0.12,0.14,0.16,0.18,0.20"))
    parser.add_argument("--map-sample-seconds", type=float, default=0.70)
    parser.add_argument("--map-transient-seconds", type=float, default=0.08)
    parser.add_argument("--map-fit-min-speed", type=float, default=0.04)
    parser.add_argument("--map-stop-speed", type=float, default=0.025)
    parser.add_argument("--map-max-speed", type=float, default=0.85)
    parser.add_argument("--map-min-samples", type=int, default=5)
    parser.add_argument("--map-sweep-window", type=float, default=0.06)
    parser.add_argument("--map-sweep-min-samples", type=int, default=4)
    parser.add_argument("--map-sweep-start-speed", type=float, default=0.72)
    parser.add_argument("--map-sweep-timeout", type=float, default=10.0)
    parser.add_argument("--map-sweep-transient-seconds", type=float, default=0.12)
    parser.add_argument(
        "--map-sweep-decelerating-max-pedal", type=float, default=0.16)
    parser.add_argument(
        "--stop-distance-speed-steps", type=comma_separated_floats,
        default=comma_separated_floats("0.20,0.30,0.40,0.50,0.60"))
    parser.add_argument("--stop-distance-timeout", type=float, default=5.0)
    parser.add_argument(
        "--stop-distance-speed-tolerance", type=float, default=0.025)
    parser.add_argument(
        "--stop-distance-stable-seconds", type=float, default=0.20)
    parser.add_argument("--publish-rate", type=float, default=30.0)
    parser.add_argument("--input-timeout", type=float, default=6.0)
    parser.add_argument("--max-speed", type=float, default=1.35)
    parser.add_argument("--max-displacement", type=float, default=18.0)
    parser.add_argument("--trial-stop-speed", type=float, default=1.10)
    parser.add_argument("--fit-max-speed", type=float, default=1.00)
    parser.add_argument("--transient-seconds", type=float, default=0.35)
    parser.add_argument("--throttle-seconds", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=12)
    parser.add_argument("--coast-start-speed", type=float, default=0.90)
    parser.add_argument("--coast-end-speed", type=float, default=0.25)
    parser.add_argument("--coast-seconds", type=float, default=8.0)
    parser.add_argument("--brake-start-speed", type=float, default=0.90)
    parser.add_argument("--brake-fit-min-speed", type=float, default=0.10)
    parser.add_argument("--brake-stop-speed", type=float, default=0.035)
    parser.add_argument("--brake-transient-seconds", type=float, default=0.08)
    parser.add_argument("--brake-seconds", type=float, default=6.0)
    parser.add_argument("--min-brake-samples", type=int, default=5)
    parser.add_argument("--approach-throttle", type=float, default=0.08)
    parser.add_argument("--reverse-approach-throttle", type=float, default=0.24)
    parser.add_argument("--approach-kp", type=float, default=0.06)
    parser.add_argument("--max-approach-throttle", type=float, default=0.14)
    parser.add_argument(
        "--reverse-max-approach-throttle", type=float, default=0.32)
    parser.add_argument("--accelerate-timeout", type=float, default=10.0)
    parser.add_argument("--gear-prepare-seconds", type=float, default=0.7)
    parser.add_argument("--reset-settle-seconds", type=float, default=1.0)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--hold-max-speed", type=float, default=0.02)
    parser.add_argument("--hold-max-displacement", type=float, default=0.02)
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--reset-z-offset", type=float, default=0.35)
    parser.add_argument(
        "--no-snap-reset-to-road", dest="snap_reset_to_road",
        action="store_false")
    parser.set_defaults(snap_reset_to_road=True)
    parser.add_argument("--control-topic", default="/vehicle_control_cmd")
    parser.add_argument("--state-topic", default="/chatter")
    parser.add_argument(
        "--collision-topic", default="/carla/vehicle_collision")
    parser.add_argument(
        "--output-prefix",
        default=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "calibration_results",
            "carla_cybertruck_longitudinal_{}".format(
                time.strftime("%Y%m%d_%H%M%S"))))
    args = parser.parse_args(rospy.myargv()[1:])
    invalid_stages = set(args.stages) - {
        "creep", "creep_stop", "coast", "throttle", "brake", "hold", "reverse_map",
        "reverse_sweep", "reverse_stop"}
    invalid_directions = set(args.directions) - {"forward", "reverse"}
    if invalid_stages:
        parser.error("invalid stages: {}".format(sorted(invalid_stages)))
    if invalid_directions:
        parser.error("invalid directions: {}".format(sorted(invalid_directions)))
    if args.repeats < 1:
        parser.error("--repeats must be at least one")
    return args


def main():
    args = parse_args()
    rospy.init_node("carla_longitudinal_calibration", anonymous=True)
    calibration = LongitudinalCalibration(args)
    try:
        calibration.run()
    except Exception as error:
        rospy.logerr("longitudinal calibration stopped: %s", error)
        raise


if __name__ == "__main__":
    main()
