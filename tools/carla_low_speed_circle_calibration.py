#!/usr/bin/env python3
"""Run a guarded low-speed steering circle calibration through Lite ROS topics."""

import argparse
import csv
import math
import os
import statistics
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class CircleCalibration:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.state = None
        self.imu = None
        self.collision = None
        self.start_xy = None
        self.raw_rows = []
        self.summaries = []
        self.stop_reason = "completed"
        self.carla_actor = None
        self.carla_reset_transform = None

        self.control_pub = rospy.Publisher(
            args.control_topic, Float64MultiArray, queue_size=1)
        rospy.Subscriber(args.state_topic, Float64MultiArray,
                         self.state_callback, queue_size=10)
        rospy.Subscriber(args.imu_topic, Float64MultiArray,
                         self.imu_callback, queue_size=20)
        rospy.Subscriber(args.collision_topic, Float64MultiArray,
                         self.collision_callback, queue_size=10)
        if args.reset_between_steps:
            self.prepare_carla_reset()

    def prepare_carla_reset(self):
        import carla

        client = carla.Client(self.args.carla_host, self.args.carla_port)
        client.set_timeout(3.0)
        actors = list(client.get_world().get_actors().filter(
            "vehicle.tesla.cybertruck"))
        if len(actors) != 1:
            raise RuntimeError(
                "expected exactly one Cybertruck for pose reset, found {}".format(
                    len(actors)))
        self.carla_actor = actors[0]
        transform = self.carla_actor.get_transform()
        self.carla_reset_transform = carla.Transform(
            carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z),
            carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw,
                roll=transform.rotation.roll))
        rospy.loginfo(
            "reset pose: x=%.3f y=%.3f z=%.3f yaw=%.3f deg",
            transform.location.x, transform.location.y,
            transform.location.z, transform.rotation.yaw)

    def reset_carla_actor(self):
        if self.carla_actor is None or self.carla_reset_transform is None:
            return
        import carla

        self.carla_actor.apply_control(carla.VehicleControl(
            throttle=0.0, steer=0.0, brake=1.0, hand_brake=True))
        self.carla_actor.set_target_velocity(carla.Vector3D())
        self.carla_actor.set_target_angular_velocity(carla.Vector3D())
        self.carla_actor.set_transform(self.carla_reset_transform)
        self.brake_to_stop(1.0)

    def state_callback(self, message):
        data = message.data
        for offset in range(0, len(data), 18):
            if offset + 18 > len(data):
                break
            if int(round(data[offset])) != self.args.vehicle_id:
                continue
            sample = {
                "stamp": time.monotonic(),
                # Lite state stores CARLA y before x.
                "x": float(data[offset + 2]),
                "y": float(data[offset + 1]),
                "yaw": float(data[offset + 7]),
                "speed": math.hypot(data[offset + 9], data[offset + 10]),
                "actual_steer": float(data[offset + 13]),
                "actual_throttle": float(data[offset + 12]),
                "actual_brake": float(data[offset + 14]),
            }
            with self.lock:
                self.state = sample
                if self.start_xy is None:
                    self.start_xy = (sample["x"], sample["y"])
            return

    def imu_callback(self, message):
        if len(message.data) < 7:
            return
        raw_yaw_rate = float(message.data[6])
        with self.lock:
            self.imu = {
                "stamp": time.monotonic(),
                "yaw_rate_raw": raw_yaw_rate,
                "yaw_rate": raw_yaw_rate * self.args.imu_yaw_rate_scale,
            }

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
            imu = dict(self.imu) if self.imu else None
            collision = dict(self.collision) if self.collision else None
            start_xy = self.start_xy
        return state, imu, collision, start_xy

    def publish_control(self, throttle, steer, brake, hand_brake=False):
        message = Float64MultiArray()
        message.data = [
            float(self.args.vehicle_id),
            clamp(throttle, 0.0, 1.0),
            clamp(steer, -1.0, 1.0),
            clamp(brake, 0.0, 1.0),
            0.0,
            1.0 if hand_brake else 0.0,
        ]
        self.control_pub.publish(message)

    def wait_for_inputs(self):
        deadline = time.monotonic() + self.args.input_timeout
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, imu, _, _ = self.snapshot()
            if state and imu:
                return
            self.publish_control(0.0, 0.0, 1.0, True)
            rate.sleep()
        raise RuntimeError("timed out waiting for /chatter and IMU data")

    def safety_check(self, state, imu, collision, start_xy):
        now = time.monotonic()
        if state is None or now - state["stamp"] > 0.5:
            return "vehicle state timeout"
        if imu is None or now - imu["stamp"] > 0.5:
            return "IMU timeout"
        if collision and collision["active"] >= 0.5:
            return "collision reported"
        if start_xy:
            displacement = math.hypot(
                state["x"] - start_xy[0], state["y"] - start_xy[1])
            if displacement > self.args.max_displacement:
                return "maximum displacement exceeded"
        if state["speed"] > self.args.max_speed:
            return "maximum speed exceeded"
        return None

    def speed_control(self, speed):
        error = self.args.target_speed - speed
        if error >= -0.03:
            throttle = clamp(
                self.args.throttle_feedforward +
                self.args.speed_kp * max(0.0, error),
                0.0, self.args.max_throttle)
            return throttle, 0.0
        return 0.0, clamp(
            self.args.brake_kp * (-error), 0.0, self.args.max_brake)

    def brake_to_stop(self, duration):
        deadline = time.monotonic() + duration
        rate = rospy.Rate(self.args.publish_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, _, _, _ = self.snapshot()
            stopped = state is not None and state["speed"] < 0.03
            self.publish_control(0.0, 0.0, 1.0, stopped)
            rate.sleep()

    def run_step(self, command_steer, step_index):
        total_duration = self.args.settle_seconds + self.args.sample_seconds
        started = time.monotonic()
        deadline = started + total_duration
        samples = []
        rate = rospy.Rate(self.args.publish_rate)
        rospy.loginfo(
            "step %d/%d: normalized steer=%+.4f",
            step_index + 1, len(self.args.steps), command_steer)

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state, imu, collision, start_xy = self.snapshot()
            reason = self.safety_check(state, imu, collision, start_xy)
            if reason:
                self.stop_reason = reason
                raise RuntimeError(reason)

            throttle, brake = self.speed_control(state["speed"])
            self.publish_control(throttle, command_steer, brake, False)
            elapsed = time.monotonic() - started
            phase = "sample" if elapsed >= self.args.settle_seconds else "settle"
            row = {
                "wall_time": time.time(),
                "step": step_index,
                "phase": phase,
                "command_steer": command_steer,
                "actual_steer": state["actual_steer"],
                "x": state["x"],
                "y": state["y"],
                "yaw": state["yaw"],
                "speed": state["speed"],
                "yaw_rate_raw": imu["yaw_rate_raw"],
                "yaw_rate": imu["yaw_rate"],
                "throttle": throttle,
                "brake": brake,
            }
            self.raw_rows.append(row)
            if (phase == "sample" and
                    state["speed"] >= self.args.min_sample_speed):
                samples.append(row)
            rate.sleep()

        if len(samples) < 10:
            raise RuntimeError(
                "insufficient steady samples for steer {:.4f}".format(
                    command_steer))
        self.summaries.append(self.summarize(command_steer, samples))
        summary = self.summaries[-1]
        rospy.loginfo(
            "result steer=%+.4f speed=%.3f m/s yaw_rate=%+.4f rad/s "
            "equivalent_angle=%+.3f deg radius=%.3f m samples=%d",
            summary["actual_steer_mean"], summary["speed_mean"],
            summary["yaw_rate_mean"], summary["equivalent_angle_deg"],
            summary["radius_m"], summary["sample_count"])

    def summarize(self, command_steer, samples):
        actual_steer = [row["actual_steer"] for row in samples]
        speeds = [row["speed"] for row in samples]
        yaw_rates = [row["yaw_rate"] for row in samples]
        mean_steer = statistics.mean(actual_steer)
        mean_speed = statistics.mean(speeds)
        mean_yaw_rate = statistics.mean(yaw_rates)
        abs_yaw_rate = statistics.mean(abs(value) for value in yaw_rates)
        radius = mean_speed / abs_yaw_rate if abs_yaw_rate > 1.0e-6 else math.inf
        equivalent_angle = math.atan2(self.args.wheel_base, radius)
        equivalent_angle = math.copysign(equivalent_angle, mean_steer)

        inner_angle = abs(mean_steer) * self.args.carla_inner_max_angle
        if inner_angle > 1.0e-9:
            inner_radius = self.args.wheel_base / math.tan(inner_angle)
            center_radius = inner_radius + 0.5 * self.args.front_track
            analytic_angle = math.atan2(self.args.wheel_base, center_radius)
            analytic_angle = math.copysign(analytic_angle, mean_steer)
        else:
            analytic_angle = 0.0

        return {
            "command_steer": command_steer,
            "actual_steer_mean": mean_steer,
            "actual_steer_std": statistics.pstdev(actual_steer),
            "speed_mean": mean_speed,
            "speed_std": statistics.pstdev(speeds),
            "yaw_rate_mean": mean_yaw_rate,
            "yaw_rate_abs_mean": abs_yaw_rate,
            "yaw_rate_std": statistics.pstdev(yaw_rates),
            "radius_m": radius,
            "equivalent_angle_rad": equivalent_angle,
            "equivalent_angle_deg": math.degrees(equivalent_angle),
            "analytic_angle_deg": math.degrees(analytic_angle),
            "angle_error_deg": math.degrees(equivalent_angle - analytic_angle),
            "sample_count": len(samples),
        }

    def write_results(self):
        output_dir = os.path.dirname(os.path.abspath(self.args.output_prefix))
        os.makedirs(output_dir, exist_ok=True)
        raw_path = self.args.output_prefix + "_raw.csv"
        summary_path = self.args.output_prefix + "_summary.csv"
        raw_fields = [
            "wall_time", "step", "phase", "command_steer", "actual_steer",
            "x", "y", "yaw", "speed", "yaw_rate_raw", "yaw_rate",
            "throttle", "brake"]
        with open(raw_path, "w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=raw_fields)
            writer.writeheader()
            writer.writerows(self.raw_rows)
        if self.summaries:
            with open(summary_path, "w", newline="") as output:
                writer = csv.DictWriter(
                    output, fieldnames=list(self.summaries[0].keys()))
                writer.writeheader()
                writer.writerows(self.summaries)
        rospy.loginfo("raw data: %s", raw_path)
        if self.summaries:
            rospy.loginfo("summary: %s", summary_path)

    def run(self):
        self.wait_for_inputs()
        self.brake_to_stop(1.0)
        try:
            for index, steer in enumerate(self.args.steps):
                self.run_step(steer, index)
                self.brake_to_stop(self.args.rest_seconds)
                self.reset_carla_actor()
        finally:
            self.brake_to_stop(2.0)
            self.write_results()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-id", type=int, default=0)
    parser.add_argument("--steps", default="0.15,-0.15,0.25,-0.25,0.35,-0.35,0.45,-0.45,0.55,-0.55")
    parser.add_argument("--target-speed", type=float, default=1.2)
    parser.add_argument("--max-speed", type=float, default=2.0)
    parser.add_argument("--min-sample-speed", type=float, default=0.6)
    parser.add_argument("--max-displacement", type=float, default=45.0)
    parser.add_argument("--settle-seconds", type=float, default=4.0)
    parser.add_argument("--sample-seconds", type=float, default=5.0)
    parser.add_argument("--rest-seconds", type=float, default=2.0)
    parser.add_argument("--publish-rate", type=float, default=30.0)
    parser.add_argument("--input-timeout", type=float, default=5.0)
    parser.add_argument("--reset-between-steps", action="store_true")
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", type=int, default=2000)
    # zheda-2.py 当前将 CARLA IMU gyroscope 原样发布，现场值的单位为 deg/s。
    parser.add_argument(
        "--imu-yaw-rate-scale", type=float, default=math.pi / 180.0,
        help="multiply IMU field 6 by this value to obtain rad/s")
    parser.add_argument("--throttle-feedforward", type=float, default=0.035)
    parser.add_argument("--speed-kp", type=float, default=0.14)
    parser.add_argument("--brake-kp", type=float, default=0.30)
    parser.add_argument("--max-throttle", type=float, default=0.22)
    parser.add_argument("--max-brake", type=float, default=0.25)
    parser.add_argument("--wheel-base", type=float, default=4.0752487613)
    parser.add_argument("--front-track", type=float, default=1.9454631016)
    parser.add_argument(
        "--carla-inner-max-angle", type=float,
        default=math.radians(70.0))
    parser.add_argument("--control-topic", default="/vehicle_control_cmd")
    parser.add_argument("--state-topic", default="/chatter")
    parser.add_argument("--imu-topic", default="/carla/imu_attacked")
    parser.add_argument("--collision-topic", default="/carla/vehicle_collision")
    parser.add_argument(
        "--output-prefix",
        default=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "calibration_results", "carla_cybertruck_steering"))
    args = parser.parse_args(rospy.myargv()[1:])
    args.steps = [float(value) for value in args.steps.split(",") if value]
    if not args.steps:
        parser.error("at least one steering step is required")
    return args


def main():
    args = parse_args()
    rospy.init_node("carla_low_speed_circle_calibration", anonymous=True)
    calibration = CircleCalibration(args)
    try:
        calibration.run()
    except Exception as error:
        rospy.logerr("calibration stopped: %s", error)
        raise


if __name__ == "__main__":
    main()
