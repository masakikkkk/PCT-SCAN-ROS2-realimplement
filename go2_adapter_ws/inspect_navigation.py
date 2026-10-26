#!/usr/bin/env python3
"""Collect navigation inputs and TF using the existing Go2 ROS 2 environment.

Standalone: does not require building the adapter or importing Unitree messages.
Creates subscriptions only; never publishes commands or starts SLAM.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time


ODOM_TOPICS = (
    "/utlidar/robot_odom", "/uslam/frontend/odom", "/uslam/localization/odom",
)
CLOUD_TOPICS = (
    "/utlidar/cloud_deskewed", "/uslam/cloud_map",
    "/uslam/frontend/cloud_world_ds", "/uslam/localization/cloud_world",
)
CONTROL_TOPICS = (
    "/api/sport/request", "/api/sport/response",
    "/api/sport_lease/request", "/api/sport_lease/response",
    "/uslam/client_command",
)


def seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def vector(value, names="xyz"):
    # Preserve JSON validity even if a publisher sends NaN/Inf.
    return {key: float(getattr(value, key)) if math.isfinite(getattr(value, key))
            else None for key in names}


def sample(message, now, cloud):
    result = {"frame_id": message.header.frame_id,
              "stamp_sec": seconds(message.header.stamp),
              "age_sec_at_receive": now - seconds(message.header.stamp)}
    if cloud:
        result.update(points=message.width * message.height,
                      fields=[field.name for field in message.fields],
                      width=message.width, height=message.height,
                      point_step=message.point_step, row_step=message.row_step,
                      data_bytes=len(message.data))
    else:
        result.update(child_frame_id=message.child_frame_id,
                      position=vector(message.pose.pose.position),
                      orientation=vector(message.pose.pose.orientation, "xyzw"),
                      linear_velocity=vector(message.twist.twist.linear),
                      angular_velocity=vector(message.twist.twist.angular))
    return result


def endpoint_info(endpoint):
    qos = endpoint.qos_profile
    return {"node": endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name,
            "reliability": str(qos.reliability), "durability": str(qos.durability),
            "depth": qos.depth}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Sampling duration in seconds (default: 20; max: 60).")
    parser.add_argument("--output", type=Path, default=Path("go2_navigation_report.json"))
    parser.add_argument("--odom-topic", default=ODOM_TOPICS[0])
    parser.add_argument("--cloud-topic", default=CLOUD_TOPICS[0])
    parser.add_argument("--map-topic", default=CLOUD_TOPICS[1])
    parser.add_argument("--sensor-frame", help="Known physical lidar TF frame, if available.")
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 60:
        parser.error("--duration must be between 2 and 60 seconds")
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                              qos_profile_sensor_data)
        from rclpy.time import Time
        from tf2_ros import Buffer
    except ImportError as exc:
        parser.exit(2, "Cannot import existing ROS 2 environment: %s\n"
                    "Run in the robot's existing Foxy terminal with /usr/bin/python3; "
                    "no ROS installation or adapter build is needed.\n" % exc)

    rclpy.init(args=[])
    node = rclpy.create_node("go2_navigation_inventory_%d" % os.getpid())
    buffer = Buffer()
    streams, latest, edges = {}, {}, {}
    subscriptions = []
    report = {"collected_at_utc": datetime.now(timezone.utc).isoformat(),
              "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
              "control_commands_sent": False,
              "requested_topics": {"odom": args.odom_topic, "cloud": args.cloud_topic,
                                   "map": args.map_topic},
              "sampling_duration_sec": args.duration,
              "unresolved": [
                  "Odom child-frame name alone does not establish the physical reference point.",
                  "body_height requires body-reference z minus ground z in the same frame; "
                  "it is not the odom z value. No height is inferred automatically.",
                  "Point counts/frequency do not prove full-map versus incremental publication.",
                  "TF availability does not prove loop closure or correct frame semantics.",
                  "Sport/lease topic existence does not prove control access or lease requirement.",
                  "Twist reference-frame semantics must agree with the publisher/SDK.",
              ]}

    def receive(topic, message, cloud):
        arrived = time.monotonic()
        item = streams[topic]
        value = sample(message, node.get_clock().now().nanoseconds * 1e-9, cloud)
        if item["received_count"] == 0:
            item["first"] = value
            item["first_received_monotonic"] = arrived
        item["last"] = value
        item["last_received_monotonic"] = arrived
        item["received_count"] += 1
        item["frame_ids"].add(message.header.frame_id)
        if not cloud:
            item["child_frame_ids"].add(message.child_frame_id)
        else:
            points = value["points"]
            item["min_points"] = min(item.get("min_points", points), points)
            item["max_points"] = max(item.get("max_points", points), points)
        latest[topic] = message

    def receive_tf(message, static):
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id, static)
            edges[key] = {"parent": key[0], "child": key[1], "static": static,
                          "stamp_sec": seconds(transform.header.stamp),
                          "translation": vector(transform.transform.translation),
                          "rotation": vector(transform.transform.rotation, "xyzw")}
            try:
                if static:
                    buffer.set_transform_static(transform, "inventory_subscription")
                else:
                    buffer.set_transform(transform, "inventory_subscription")
            except Exception as exc:
                edges[key]["buffer_error"] = str(exc)

    try:
        # Discover publishers before choosing durability for retained map messages.
        discovery_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        odom_topics = list(dict.fromkeys((args.odom_topic,) + ODOM_TOPICS))
        cloud_topics = list(dict.fromkeys((args.cloud_topic, args.map_topic) + CLOUD_TOPICS))
        for cloud, topics, message_type in (
                (False, odom_topics, Odometry), (True, cloud_topics, PointCloud2)):
            for topic in topics:
                streams[topic] = {"received_count": 0, "frame_ids": set(),
                                  "child_frame_ids": set()}
                qos = qos_profile_sensor_data
                try:
                    publishers = node.get_publishers_info_by_topic(topic)
                except Exception as exc:
                    publishers = []
                    streams[topic]["qos_discovery_error"] = str(exc)
                if cloud and publishers and all(
                        info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                        for info in publishers):
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                streams[topic]["subscription_durability"] = str(qos.durability)
                subscriptions.append(node.create_subscription(
                    message_type, topic,
                    lambda msg, name=topic, is_cloud=cloud: receive(name, msg, is_cloud),
                    qos))
        # BEST_EFFORT receives both reliable and best-effort TF publishers.
        # TRANSIENT_LOCAL is needed to receive static TF sent before this script.
        for topic, static in (("/tf", False), ("/tf_static", True)):
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=(DurabilityPolicy.TRANSIENT_LOCAL if static
                                         else DurabilityPolicy.VOLATILE))
            subscriptions.append(node.create_subscription(
                TFMessage, topic, lambda msg, flag=static: receive_tf(msg, flag), qos))

        print("Read-only sampling for %.0f seconds; no motion or SLAM commands are sent."
              % args.duration, flush=True)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        graph = dict(node.get_topic_names_and_types())
        inspected = sorted(set(streams).union(CONTROL_TOPICS, ("/tf", "/tf_static")))
        report["topic_inventory"] = {}
        for topic in inspected:
            item = {"types": graph.get(topic, []), "publishers": [], "subscribers": []}
            try:
                item["publishers"] = [endpoint_info(info)
                                      for info in node.get_publishers_info_by_topic(topic)]
                item["subscribers"] = [endpoint_info(info)
                                       for info in node.get_subscriptions_info_by_topic(topic)
                                       if info.node_name != node.get_name()]
            except Exception as exc:
                item["endpoint_query_error"] = str(exc)
            report["topic_inventory"][topic] = item

        ended = time.monotonic()
        for topic, item in streams.items():
            item["frame_ids"] = sorted(item["frame_ids"])
            item["child_frame_ids"] = sorted(item["child_frame_ids"])
            if item["received_count"]:
                first = item.pop("first_received_monotonic")
                last = item.pop("last_received_monotonic")
                elapsed = last - first
                item["observed_receive_hz"] = ((item["received_count"] - 1) / elapsed
                                               if elapsed > 0 else None)
                item["seconds_since_last_received"] = ended - last
        report["streams"] = streams
        report["frequency_note"] = (
            "Rates are callbacks observed on this computer, not guaranteed publisher rates. "
            "Zero samples mean not observed in this window; no SLAM capability is inferred.")
        report["tf_edges"] = [edges[key] for key in sorted(edges)]
        queries = set()
        odom = latest.get(args.odom_topic)
        if odom is not None:
            world, body = odom.header.frame_id, odom.child_frame_id
            for topic in (args.cloud_topic, args.map_topic):
                message = latest.get(topic)
                if message is not None and message.header.frame_id:
                    queries.add((world, message.header.frame_id, "latest", 0, 0))
                    stamp = message.header.stamp
                    queries.add((world, message.header.frame_id, "message_stamp",
                                 stamp.sec, stamp.nanosec))
                    queries.add((body, message.header.frame_id, "latest", 0, 0))
            if args.sensor_frame:
                queries.add((body, args.sensor_frame, "latest", 0, 0))
        report["tf_queries"] = []
        for target, source, when, sec, nanosec in sorted(queries):
            item = {"target_frame": target, "source_frame": source, "at": when,
                    "identity_query": target == source}
            try:
                transform = buffer.lookup_transform(
                    target, source, Time(seconds=sec, nanoseconds=nanosec))
                item.update(available=True, stamp_sec=seconds(transform.header.stamp),
                            translation=vector(transform.transform.translation),
                            rotation=vector(transform.transform.rotation, "xyzw"))
            except Exception as exc:
                item.update(available=False, error=str(exc))
            report["tf_queries"].append(item)
        if args.cloud_topic in latest and odom is not None:
            report["last_odom_cloud_stamp_difference_sec"] = abs(
                seconds(odom.header.stamp) - seconds(latest[args.cloud_topic].header.stamp))
        report["last_stamp_difference_note"] = (
            "The last-message difference is not a synchronized-pair timing test.")
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                          allow_nan=False) + "\n")
        print("Report saved: " + str(args.output.resolve()))
        for topic, item in streams.items():
            print("%s: %d samples, frames=%s" %
                  (topic, item["received_count"], item["frame_ids"]))
        print("body_height, full/incremental map semantics and lease access remain unverified.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
