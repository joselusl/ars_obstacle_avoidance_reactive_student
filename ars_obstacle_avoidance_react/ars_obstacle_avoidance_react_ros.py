#!/usr/bin/env python

import numpy as np
from numpy import *

import os

import time

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers

#
from ars_obstacle_avoidance_react.ars_obstacle_avoidance_react import *





class ArsObstacleAvoidanceReactRos(Node):

  #######


  # Robot frame
  robot_frame = None


  # Ctr loop freq 
  # time step
  ctr_loop_freq = None
  # Timer
  ctr_loop_timer = None


  # Robot command raw subscriber
  robot_vel_cmd_raw_stamped_sub = None


  # Obstacles subscriber
  obstacles_detected_sub = None


  # Robot command avoidance publisher
  robot_vel_cmd_avoidance_stamped_pub = None


  #
  config_param_yaml_file_name = None


  # Obstacle avoidance reactive
  obstacle_avoidance_react = ArsObstacleAvoidanceReact()
  


  #########

  def __init__(self, node_name='ars_obstacle_avoidance_react_node'):

    # Init ROS
    super().__init__(node_name)

    # Robot frame
    self.robot_frame = ""


    # Ctr loop freq 
    # time step
    self.ctr_loop_freq = 50.0
    # Timer
    self.ctr_loop_timer = None


    # Robot command raw subscriber
    self.robot_vel_cmd_raw_stamped_sub = None


    # Obstacles subscriber
    self.obstacles_detected_sub = None


    # Robot command avoidance publisher
    self.robot_vel_cmd_avoidance_stamped_pub = None

    #
    self.__init(node_name)

    #
    return


  def __init(self, node_name='ars_obstacle_avoidance_react_node'):
        
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_obstacle_avoidance_react')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")
    
    

    #### READING PARAMETERS ###
    
    # Config param
    default_config_param_yaml_file_name = os.path.join(pkg_path,'config','config_obstacle_avoidance_react.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('config_param_obstacle_avoidance_react_yaml_file', default_config_param_yaml_file_name)
    # Get the parameter value
    config_param_yaml_file_name_str = self.get_parameter('config_param_obstacle_avoidance_react_yaml_file').get_parameter_value().string_value
    self.get_logger().info(config_param_yaml_file_name_str)
    #
    self.sim_obstacles_detector_params_yaml_file_name = os.path.abspath(config_param_yaml_file_name_str)

    ###


    # Load config param
    with open(self.sim_obstacles_detector_params_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.config_param = yaml.load(file, Loader=SafeLoader)['obstacle_avoidance_react']

    if(self.config_param is None):
      self.get_logger().info("Error loading config param obstacle avoidance react")
    else:
      self.get_logger().info("Config param obstacle avoidance react:")
      self.get_logger().info(str(self.config_param))


    # Parameters
    #
    self.ctr_loop_freq = self.config_param['ctr_loop_freq']
    
    #
    self.obstacle_avoidance_react.setConfigParameters(self.config_param['algorithm'])

    
    # End
    return


  def open(self):

    # Subscriber
    #
    self.robot_vel_cmd_raw_stamped_sub = self.create_subscription(TwistStamped, 'robot_cmd_raw_stamped', self.robotVelCmdRawStampedCallback, qos_profile=10)
    #
    self.obstacles_detected_sub = self.create_subscription(MarkerArray, 'obstacles_detected', self.obstaclesDetectedCallback, qos_profile=10)
    

    # Publisher
    # Robot cmd stamped
    self.robot_vel_cmd_avoidance_stamped_pub = self.create_publisher(TwistStamped, 'robot_cmd_avoidance_stamped', qos_profile=10)


    # Timers
    #
    self.ctr_loop_timer = self.create_timer(1.0/self.ctr_loop_freq, self.ctrLoopTimerCallback)


    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def stop(self):

    # Sleep to allow time to finish
    time.sleep(0.5)

    #
    self.publishEmptyCmd(self.get_clock().now())

    #
    return


  def close(self):

    

    return


  def publishEmptyCmd(self, time_stamp=Time()):

    #
    robot_velo_cmd_stamped_msg = TwistStamped()

    robot_velo_cmd_stamped_msg.header.stamp = time_stamp
    robot_velo_cmd_stamped_msg.header.frame_id = self.robot_frame

    robot_velo_cmd_stamped_msg.twist.linear.x = 0.0
    robot_velo_cmd_stamped_msg.twist.linear.y = 0.0
    robot_velo_cmd_stamped_msg.twist.linear.z = 0.0

    robot_velo_cmd_stamped_msg.twist.angular.x = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.y = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.z = 0.0

    #
    if(self.robot_vel_cmd_avoidance_stamped_pub):
      self.robot_vel_cmd_avoidance_stamped_pub.publish(robot_velo_cmd_stamped_msg)

    return


  def robotVelCmdRawStampedCallback(self, robot_vel_cmd_raw_stamped_msg):

    # Timestamp
    robot_velo_cmd_raw_time_stamp = robot_vel_cmd_raw_stamped_msg.header.stamp

    # Baselink
    robot_frame = robot_vel_cmd_raw_stamped_msg.header.frame_id
    if not isinstance(robot_frame, str):
      robot_frame = str(robot_frame)
    self.robot_frame = robot_frame

    # Linear
    lin_vel_cmd_raw_ref = np.zeros((3,), dtype=float)
    lin_vel_cmd_raw_ref[0] = robot_vel_cmd_raw_stamped_msg.twist.linear.x
    lin_vel_cmd_raw_ref[1] = robot_vel_cmd_raw_stamped_msg.twist.linear.y
    lin_vel_cmd_raw_ref[2] = robot_vel_cmd_raw_stamped_msg.twist.linear.z

    # Angular
    ang_vel_cmd_raw_ref = np.zeros((1,), dtype=float)
    ang_vel_cmd_raw_ref[0] = robot_vel_cmd_raw_stamped_msg.twist.angular.z

    #
    self.obstacle_avoidance_react.setRobotVeloCmdRaw(robot_velo_cmd_raw_time_stamp, lin_vel_cmd_raw_ref, ang_vel_cmd_raw_ref)

    #
    return


  def obstaclesDetectedCallback(self, obstacles_detected_msg):
    
    # Save
    self.obstacle_avoidance_react.obstacles_detected_msg = obstacles_detected_msg

    #
    return


  def robotVelCmdAvoidancePub(self):

    # Get
    robot_velo_cmd_time_stamp = self.obstacle_avoidance_react.getRobotVeloCmdAvoidTimeStamp()
    robot_velo_lin_cmd = self.obstacle_avoidance_react.getRobotVeloLinCmdAvoid()
    robot_velo_ang_cmd = self.obstacle_avoidance_react.getRobotVeloAngCmdAvoid()

    #
    robot_velo_cmd_stamped_msg = TwistStamped()

    robot_velo_cmd_stamped_msg.header.stamp = robot_velo_cmd_time_stamp.to_msg()
    robot_velo_cmd_stamped_msg.header.frame_id = self.robot_frame

    robot_velo_cmd_stamped_msg.twist.linear.x = robot_velo_lin_cmd[0]
    robot_velo_cmd_stamped_msg.twist.linear.y = robot_velo_lin_cmd[1]
    robot_velo_cmd_stamped_msg.twist.linear.z = robot_velo_lin_cmd[2]

    robot_velo_cmd_stamped_msg.twist.angular.x = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.y = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.z = robot_velo_ang_cmd[0]

    #
    if(self.robot_vel_cmd_avoidance_stamped_pub):
      self.robot_vel_cmd_avoidance_stamped_pub.publish(robot_velo_cmd_stamped_msg)

    #
    return


  def ctrLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    #
    self.obstacle_avoidance_react.ctrLoopObstacleAvoidanceReact(time_stamp_current)

    # Publish
    self.robotVelCmdAvoidancePub()
    
    # End
    return


  