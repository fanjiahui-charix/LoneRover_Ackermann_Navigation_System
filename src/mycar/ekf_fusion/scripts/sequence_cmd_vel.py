#!/usr/bin/env python3
import argparse
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist


Segment = Tuple[float, float, float]


def parse_segment(text: str) -> Segment:
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "segment must be linear_x,angular_z,duration, for example 0.70,0.50,3.0"
        )
    try:
        linear_x = float(parts[0])
        angular_z = float(parts[1])
        duration = float(parts[2])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    if duration < 0.0:
        raise argparse.ArgumentTypeError("segment duration must be non-negative")
    return linear_x, angular_z, duration


def make_twist(linear_x: float, angular_z: float) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


def publish_for(node, pub, msg: Twist, duration: float, rate: float) -> int:
    period = 1.0 / rate
    end = time.monotonic() + duration
    count = 0
    while time.monotonic() < end:
        pub.publish(msg)
        count += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        sleep_for = min(period, max(0.0, end - time.monotonic()))
        if sleep_for > 0.0:
            time.sleep(sleep_for)
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a sequence of /cmd_vel segments.")
    parser.add_argument(
        "--segment",
        action="append",
        type=parse_segment,
        required=True,
        help="linear_x,angular_z,duration. May be repeated.",
    )
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--pre-stop", type=float, default=0.5)
    parser.add_argument("--post-stop", type=float, default=2.0)
    parser.add_argument("--wait-subscriber", type=float, default=3.0)
    args = parser.parse_args()

    if args.rate <= 0.0 or args.pre_stop < 0.0 or args.post_stop < 0.0:
        raise SystemExit("rate must be positive; pre-stop and post-stop must be non-negative")

    rclpy.init()
    node = rclpy.create_node("sequence_cmd_vel")
    pub = node.create_publisher(Twist, args.topic, 10)

    wait_until = time.monotonic() + args.wait_subscriber
    while pub.get_subscription_count() == 0 and time.monotonic() < wait_until:
        rclpy.spin_once(node, timeout_sec=0.05)

    if pub.get_subscription_count() == 0:
        node.get_logger().warning(f"No subscribers on {args.topic}; publishing anyway")

    stop = make_twist(0.0, 0.0)
    if args.pre_stop > 0.0:
        publish_for(node, pub, stop, args.pre_stop, args.rate)

    total_motion_samples = 0
    for index, (linear_x, angular_z, duration) in enumerate(args.segment, start=1):
        msg = make_twist(linear_x, angular_z)
        count = publish_for(node, pub, msg, duration, args.rate)
        total_motion_samples += count
        node.get_logger().info(
            f"segment {index}: linear.x={linear_x:.3f}, angular.z={angular_z:.3f}, "
            f"duration={duration:.3f}s, samples={count}"
        )

    if args.post_stop > 0.0:
        publish_for(node, pub, stop, args.post_stop, args.rate)
    pub.publish(stop)

    node.get_logger().info(
        f"Published {len(args.segment)} segments to {args.topic}, "
        f"total motion samples={total_motion_samples}, rate={args.rate:.1f}Hz"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
