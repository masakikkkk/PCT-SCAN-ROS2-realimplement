"""Read-only Go2 sensor/schema check. Never creates a control publisher."""

import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request

from .core import finite, normalize_quaternion, stamp_is_fresh
from .state_adapter import stamp_seconds, xyz, xyzw


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--timeout', type=float, default=10.0)
    parsed = parser.parse_args(args)
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    config_path = parsed.config or Path(get_package_share_directory('go2_pct_adapter')) / 'config/go2.yaml'
    with config_path.open() as handle:
        settings = yaml.safe_load(handle)
    config = settings['go2_state_adapter']['ros__parameters']
    sport = settings['go2_sport_bridge']['ros__parameters']
    rclpy.init(args=[])
    node = rclpy.create_node('go2_pct_preflight')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received = {}
    subscriptions = [
        node.create_subscription(Odometry, config['odom_topic'],
                                 lambda msg: received.update(odom=msg), qos_profile_sensor_data),
        node.create_subscription(PointCloud2, config['cloud_topic'],
                                 lambda msg: received.update(cloud=msg), qos_profile_sensor_data),
    ]
    report = {'config': str(config_path), 'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
              'control_commands_sent': False}
    try:
        deadline = time.monotonic() + parsed.timeout
        transform = None
        tf_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not {'odom', 'cloud'}.issubset(received):
                continue
            if not config['use_tf_extrinsic']:
                break
            try:
                transform = buffer.lookup_transform(
                    config['base_frame'], config['sensor_frame'],
                    Time.from_msg(received['odom'].header.stamp), timeout=Duration(seconds=0.0))
                break
            except Exception as exc:
                tf_error = str(exc)
        errors = []
        graph = dict(node.get_topic_names_and_types())
        report['topic_types'] = {name: graph.get(name, []) for name in (
            config['odom_topic'], config['cloud_topic'], sport['sport_topic'])}
        for key, topic, expected in (
                ('odom', config['odom_topic'], 'nav_msgs/msg/Odometry'),
                ('cloud', config['cloud_topic'], 'sensor_msgs/msg/PointCloud2')):
            if expected not in graph.get(topic, []):
                errors.append(topic + ': expected type ' + expected)
            if key not in received:
                errors.append(topic + ': no message received before timeout')
        if 'unitree_api/msg/Request' not in graph.get(sport['sport_topic'], []):
            errors.append('Sport topic not found or type differs from unitree_api/msg/Request')
        now_sec = node.get_clock().now().nanoseconds * 1e-9
        for name in ('planning_frame', 'base_frame'):
            if config[name] != sport[name]:
                errors.append('State / Sport configuration differs on ' + name)
        if 'odom' in received:
            odom = received['odom']
            report['odom'] = {'frame_id': odom.header.frame_id,
                              'child_frame_id': odom.child_frame_id,
                              'position': xyz(odom.pose.pose.position),
                              'stamp_sec': stamp_seconds(odom.header.stamp)}
            if odom.header.frame_id != config['planning_frame']:
                errors.append('Set planning_frame to the actual odom frame; align tomogram to it')
            if odom.child_frame_id != config['base_frame']:
                errors.append('Check body origin and set base_frame to the actual body child_frame_id')
            if not finite((*xyz(odom.pose.pose.position), *xyz(odom.twist.twist.linear),
                           *xyz(odom.twist.twist.angular))):
                errors.append('Non-finite odometry')
            try:
                normalize_quaternion(xyzw(odom.pose.pose.orientation))
            except ValueError as exc:
                errors.append(str(exc))
        if 'cloud' in received:
            cloud = received['cloud']
            report['cloud'] = {'frame_id': cloud.header.frame_id,
                               'points': cloud.width * cloud.height,
                               'fields': [field.name for field in cloud.fields],
                               'stamp_sec': stamp_seconds(cloud.header.stamp)}
            expected = config['planning_frame'] if config['cloud_is_world'] else config['sensor_frame']
            if cloud.header.frame_id != expected:
                errors.append('Cloud frame mismatch: expected ' + expected)
            if not {'x', 'y', 'z'}.issubset(report['cloud']['fields']) or not report['cloud']['points']:
                errors.append('Cloud must be nonempty and contain XYZ fields')
        for key, msg in received.items():
            if not stamp_is_fresh(now_sec, stamp_seconds(msg.header.stamp), config['max_input_age_sec']):
                errors.append(key + ': stale/zero/future timestamp; check clock and data latency')
        if {'odom', 'cloud'}.issubset(received):
            skew = abs(stamp_seconds(received['odom'].header.stamp) -
                       stamp_seconds(received['cloud'].header.stamp))
            report['pose_cloud_skew_sec'] = skew
            if skew > config['max_pose_cloud_skew_sec']:
                errors.append('Pose/cloud skew exceeds tolerance')
        if config['use_tf_extrinsic']:
            if transform is None:
                errors.append('Missing base-to-sensor TF: ' + str(tf_error))
            else:
                report['body_to_sensor_tf'] = {
                    'xyz': xyz(transform.transform.translation),
                    'xyzw': xyzw(transform.transform.rotation)}
        else:
            if not config['extrinsics_confirmed']:
                errors.append('Measured extrinsics have not been confirmed')
            try:
                normalize_quaternion(config['body_to_sensor_xyzw'])
                if len(config['body_to_sensor_xyz']) != 3 or not finite(config['body_to_sensor_xyz']):
                    raise ValueError('Invalid body_to_sensor_xyz')
            except ValueError as exc:
                errors.append(str(exc))
        request = Request()
        request.header.identity.api_id = 1008
        request.header.identity.id = 1
        request.header.lease.id = 0
        request.parameter = '{"x":0.0,"y":0.0,"z":0.0}'
        serialize_message(request)  # Serialize locally; do not publish.
        report['unitree_request_serialization'] = 'PASS'
        report['lease_required_by_config'] = sport['require_lease']
        report['lease_acquisition_verified'] = False
        report['errors'] = errors
        report['result'] = 'PASS' if not errors else 'FAIL'
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if not errors else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
