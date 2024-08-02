#!/usr/bin/env python3

import numpy as np
from numpy import *
import math
import sys


# ROS
import rclpy
from rclpy.time import Time

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers



class ArsObstacleAvoidanceReact:

  #######

  # Config parameters
  # Robot size radius
  robot_size_radius = None


  # Input: References
  flag_set_robot_velo_cmd_raw = False
  robot_velo_cmd_raw_time_stamp = Time()
  robot_velo_lin_cmd_raw = None
  robot_velo_ang_cmd_raw = None


  # Input: Obstacles detected
  obstacles_detected_msg = None


  # Output: Commands
  robot_velo_cmd_avoid_time_stamp = Time()
  robot_velo_lin_cmd_avoid = None
  robot_velo_ang_cmd_avoid = None
  

 

  #########

  def __init__(self):

    # Config parameters
    # Robot size radius
    self.robot_size_radius = 0.3

    # Input: References
    self.flag_set_robot_velo_cmd_raw = False
    self.robot_velo_cmd_raw_time_stamp = Time()
    self.robot_velo_lin_cmd_raw = None
    self.robot_velo_ang_cmd_raw = None

    # Input: Obstacles detected
    self.obstacles_detected_msg = MarkerArray()

    # Output: Commands
    self.robot_velo_cmd_avoid_time_stamp = Time()
    self.robot_velo_lin_cmd_avoid = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_avoid = np.zeros((1,), dtype=float)

    # End
    return


  def setConfigParameters(self, config_param):

    # Robot size radius
    self.robot_size_radius = config_param['robot_size_radius']

    return


  def setRobotVeloCmdRaw(self, robot_velo_cmd_raw_time_stamp, robot_velo_lin_cmd_raw, robot_velo_ang_cmd_raw):

    self.flag_set_robot_velo_cmd_raw = True

    self.robot_velo_cmd_raw_time_stamp = robot_velo_cmd_raw_time_stamp

    self.robot_velo_lin_cmd_raw = robot_velo_lin_cmd_raw
    self.robot_velo_ang_cmd_raw = robot_velo_ang_cmd_raw

    #
    return


  def getRobotVeloCmdAvoidTimeStamp(self):
    return self.robot_velo_cmd_avoid_time_stamp

  def getRobotVeloLinCmdAvoid(self):
    return self.robot_velo_lin_cmd_avoid

  def getRobotVeloAngCmdAvoid(self):
    return self.robot_velo_ang_cmd_avoid


  def ctrLoopObstacleAvoidanceReact(self, time_stamp_ros):

    # Time stamp
    self.robot_velo_cmd_avoid_time_stamp = time_stamp_ros


    # Check
    if(self.flag_set_robot_velo_cmd_raw == False):
      # Set to zero
      self.robot_velo_lin_cmd_avoid = np.zeros((3,), dtype=float)
      self.robot_velo_ang_cmd_avoid = np.zeros((1,), dtype=float)
      #
      return

    # Initialize with the raw value
    self.robot_velo_lin_cmd_avoid[:] = self.robot_velo_lin_cmd_raw
    self.robot_velo_ang_cmd_avoid[:] = self.robot_velo_ang_cmd_raw

    # Iterate for all obstacles
    for obst_detected_i in self.obstacles_detected_msg.markers:

      # Size of obst i
      rad_obst_i = obst_detected_i.scale.x/2.0

      # Position of obst i wrt robot
      posi_2d_obst_i_wrt_robot = np.zeros((2,), dtype=float)
      posi_2d_obst_i_wrt_robot[0] = obst_detected_i.pose.position.x
      posi_2d_obst_i_wrt_robot[1] = obst_detected_i.pose.position.y


      ###### TODO STUDENT
      
      # Useful variable
      # self.robot_size_radius


      # Command (complete)
      self.robot_velo_lin_cmd_avoid[0] 
      self.robot_velo_lin_cmd_avoid[1]

      ###### END TODO BY STUDENT

    # End
    return
