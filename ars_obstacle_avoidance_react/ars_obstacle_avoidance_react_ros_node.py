#!/usr/bin/env python3

import rclpy

from ars_obstacle_avoidance_react.ars_obstacle_avoidance_react_ros import ArsObstacleAvoidanceReactRos


def main(args=None):

  rclpy.init(args=args)

  ars_obstacle_avoidance_react_ros = ArsObstacleAvoidanceReactRos()

  ars_obstacle_avoidance_react_ros.open()

  try:
      ars_obstacle_avoidance_react_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_obstacle_avoidance_react_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()