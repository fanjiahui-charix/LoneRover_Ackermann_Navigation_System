#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


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
    parser = argparse.ArgumentParser(description="Publish /cmd_vel for an exact duration, then stop.")
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--pre-stop", type=float, default=0.5)
    parser.add_argument("--post-stop", type=float, default=2.0)
    parser.add_argument("--wait-subscriber", type=float, default=3.0)
    args = parser.parse_args()

    if args.duration < 0.0 or args.rate <= 0.0 or args.pre_stop < 0.0 or args.post_stop < 0.0:
        raise SystemExit("duration, rate, pre-stop, and post-stop must be positive values")

    rclpy.init()
    node = rclpy.create_node("timed_cmd_vel")
    pub = node.create_publisher(Twist, args.topic, 10)

    wait_until = time.monotonic() + args.wait_subscriber
    while pub.get_subscription_count() == 0 and time.monotonic() < wait_until:
        rclpy.spin_once(node, timeout_sec=0.05)

    if pub.get_subscription_count() == 0:
        node.get_logger().warning(f"No subscribers on {args.topic}; publishing anyway")

    stop = make_twist(0.0, 0.0)
    if args.pre_stop > 0.0:
        publish_for(node, pub, stop, args.pre_stop, args.rate)

    move = make_twist(args.linear_x, args.angular_z)
    count = publish_for(node, pub, move, args.duration, args.rate)

    if args.post_stop > 0.0:
        publish_for(node, pub, stop, args.post_stop, args.rate)
    pub.publish(stop)

    node.get_logger().info(
        f"Published {count} motion samples to {args.topic}: "
        f"linear.x={args.linear_x:.3f}, angular.z={args.angular_z:.3f}, "
        f"duration={args.duration:.3f}s, rate={args.rate:.1f}Hz"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
