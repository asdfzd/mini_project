#!/usr/bin/env python3

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

# ======================
# 초기 설정 (파일 안에서 직접 정의)
# ======================
INITIAL_POSE_POSITION = [0.01, 0.01]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES = [
    ([-2.087830066680908, 1.9390721321105957], TurtleBot4Directions.NORTH),
]
# ======================

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    navigator.undock()

    goal_pose = navigator.getPoseStamped(*GOAL_POSES[0])
    # Blocking utility call: the original file only referenced goToPose without
    # calling it, so docking could start immediately after an asynchronous goal.
    navigator.goToPose(goal_pose)

    navigator.dock()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
