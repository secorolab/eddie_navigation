#!/usr/bin/env python3
"""Drive Eddie through waypoints with nav2 and show them as markers on /nav_goals."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

PRESETS = {
    # straight back, around the box ahead, through the doorway
    'nav_test': [('straight', -1.0, 0.0, math.pi),
                 ('around box', 2.5, 0.0, 0.0),
                 ('doorway', 5.0, -1.5, math.pi / 2)],
    # off the corridor through three of the lab's 0.8 m doorways, then back
    'secoro': [('right room', 3.5, 3.8, 0.0),
               ('left room', -3.5, 2.0, math.pi),
               ('top-left room', -3.0, 9.8, math.pi),
               ('back', 0.0, 0.0, math.pi / 2)],
}

# the sim's spawn pose, as eddie_sim_nav.launch.py passes it to the driver
START = {'nav_test': (0.0, 0.0, 0.0), 'secoro': (0.0, 0.0, math.pi / 2)}

PENDING = (0.2, 0.6, 1.0)
ACTIVE = (1.0, 0.8, 0.1)
DONE = (0.3, 0.9, 0.3)
FAILED = (1.0, 0.2, 0.2)


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def markers(nav, goals, colours):
    out = MarkerArray()
    for i, ((name, x, y, yaw), rgb) in enumerate(zip(goals, colours)):
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.ns, arrow.id, arrow.type = 'goal', i, Marker.ARROW
        arrow.pose = pose(nav, x, y, yaw).pose
        arrow.pose.position.z = 0.05
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.5, 0.1, 0.1
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = *rgb, 1.0
        label = Marker()
        label.header.frame_id = 'map'
        label.ns, label.id, label.type = 'label', i, Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.5
        label.scale.z = 0.25
        label.text = f'{i + 1} {name}'
        label.color.r, label.color.g, label.color.b, label.color.a = *rgb, 1.0
        out.markers += [arrow, label]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='nav_test', choices=sorted(PRESETS),
                        help='the preset waypoints of the sim world')
    parser.add_argument('--goal', nargs=3, type=float, action='append', metavar=('X', 'Y', 'YAW'),
                        help='a waypoint in map [m, m, rad]; repeat for more (replaces --world)')
    parser.add_argument('--timeout', type=float, default=90.0, help='[s] per waypoint')
    args, _ = parser.parse_known_args()
    if args.goal:
        goals = [(f'goal {i + 1}', *g) for i, g in enumerate(args.goal)]
    else:
        goals = PRESETS[args.world]

    rclpy.init()
    nav = BasicNavigator()
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = nav.create_publisher(MarkerArray, '/nav_goals', latched)
    colours = [PENDING] * len(goals)
    pub.publish(markers(nav, goals, colours))
    nav.setInitialPose(pose(nav, *START[args.world]))
    nav.waitUntilNav2Active()

    for i, (name, x, y, yaw) in enumerate(goals):
        colours[i] = ACTIVE
        pub.publish(markers(nav, goals, colours))
        start = time.monotonic()
        nav.goToPose(pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            if time.monotonic() - start > args.timeout:
                nav.cancelTask()
            rclpy.spin_once(nav, timeout_sec=0.1)
        ok = nav.getResult() == TaskResult.SUCCEEDED
        colours[i] = DONE if ok else FAILED
        pub.publish(markers(nav, goals, colours))
        print(f'{i + 1} {name} ({x}, {y}, {yaw:.2f}): '
              f'{"reached" if ok else "FAILED"} in {time.monotonic() - start:.1f} s', flush=True)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
